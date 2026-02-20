#define _XOPEN_SOURCE 700
/*
 * avr_pwm_monitor.c
 *
 * PWM Monitor for SimAVR
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "avr_pwm_monitor.h"
#include "avr_timer.h"
#include "avr_ioport.h"

static int pwm_pipe_fd = -1;
static const char * pipe_name = "/tmp/simavr_pwm.pipe";

// Cache for Port addresses to map to Arduino pins
static avr_io_addr_t addr_portb = 0;
static avr_io_addr_t addr_portc = 0;
static avr_io_addr_t addr_portd = 0;

static void resolve_port_addrs(avr_t * avr) {
    avr_io_t * port = avr->io_port;
    while (port) {
        if (strcmp(port->kind, "port") == 0) {
            avr_ioport_t * p = (avr_ioport_t*)port;
            if (p->name == 'B') addr_portb = p->r_port;
            if (p->name == 'C') addr_portc = p->r_port;
            if (p->name == 'D') addr_portd = p->r_port;
        }
        port = port->next;
    }
}

static int get_arduino_pin(avr_io_addr_t port_addr, int bit) {
    if (port_addr == 0) return -1;
    if (port_addr == addr_portd) return bit;       // 0-7
    if (port_addr == addr_portb) return 8 + bit;   // 8-13
    if (port_addr == addr_portc) return 14 + bit;  // A0-A5
    return -1;
}

// Track state to dedup events
typedef struct {
    avr_timer_t * timer;
    int comp_index; // 0..2 (A, B, C)
    int arduino_pin;
    uint32_t last_freq;
    uint32_t last_duty;
    uint32_t last_top;
    uint8_t last_enabled;
} pwm_channel_t;

#define MAX_CHANNELS 16
static pwm_channel_t channels[MAX_CHANNELS];
static int channel_count = 0;

static void send_update(pwm_channel_t * ch) {
    if (pwm_pipe_fd < 0 || ch->arduino_pin < 0) return;

    char buf[128];
    // Protocol: P<pin>,<freq>,<duty>,<top>,<enabled>\n
    int len = snprintf(buf, sizeof(buf), "P%d,%u,%u,%u,%d\n",
        ch->arduino_pin,
        ch->last_freq,
        ch->last_duty,
        ch->last_top,
        ch->last_enabled);
    
    if (write(pwm_pipe_fd, buf, len) < 0) {
        // Ignore EPIPE, it might mean backend disconnected
    }
}

static void check_channel_state(avr_t * avr, pwm_channel_t * ch) {
    avr_timer_t * t = ch->timer;
    avr_timer_comp_t * comp = &t->comp[ch->comp_index];

    // Check if output is enabled in TCCR (COMnx bits)
    // avr_regbit_get returns the value of the bits.
    // COM mode 0 is usually "Normal port operation, OCn disconnected"
    uint8_t mode = avr_regbit_get(avr, comp->com);
    uint8_t enabled = (mode != 0);

    // Calculate Freq
    uint32_t freq = 0;
    if (t->tov_cycles > 0 && t->cs_div_value > 0) {
        freq = avr->frequency / t->tov_cycles;
    } else {
        enabled = 0; // Timer stopped
    }

    // Get Duty
    // Reading directly from data[] to avoid side effects of io_read if any
    uint16_t duty = avr->data[comp->r_ocr];
    if (comp->r_ocrh) {
        duty |= (avr->data[comp->r_ocrh] << 8);
    }

    uint16_t top = t->tov_top;

    // Detect changes
    if (freq != ch->last_freq || duty != ch->last_duty || 
        top != ch->last_top || enabled != ch->last_enabled) {
        
        ch->last_freq = freq;
        ch->last_duty = duty;
        ch->last_top = top;
        ch->last_enabled = enabled;
        
        send_update(ch);
    }
}

static void pwm_write_hook(struct avr_t * avr, avr_io_addr_t addr, uint8_t v, void * param)
{
    // Check all channels associated with this timer
    // param is avr_timer_t*
    avr_timer_t * timer = (avr_timer_t*)param;
    
    for (int i = 0; i < channel_count; i++) {
        if (channels[i].timer == timer) {
            check_channel_state(avr, &channels[i]);
        }
    }
}

void avr_pwm_monitor_init(avr_t * avr) {
    // Open Pipe
    mkfifo(pipe_name, 0666);
    pwm_pipe_fd = open(pipe_name, O_WRONLY | O_NONBLOCK);
    if (pwm_pipe_fd < 0) {
        fprintf(stderr, "PWM: Failed to open pipe %s\n", pipe_name);
        return;
    }

    resolve_port_addrs(avr);

    // Find timers and registers
    avr_io_t * port = avr->io_port;
    while (port) {
        if (strcmp(port->kind, "timer") == 0) {
            avr_timer_t * t = (avr_timer_t*)port;
            
            // Register Write Hooks for Config Registers
            // We hook TCNT/TCCR/ICR to detect frequency changes
            // We can iterate the timer's register bits to find addresses?
            // Simpler: Hook common config registers if we can find their addresses.
            // wgm[0] -> reg. Hook that reg.
            
            // Hook WGM/CS registers (TCCR)
            for (int i = 0; i < 4; i++) {
                if (t->wgm[i].reg) avr_register_io_write(avr, t->wgm[i].reg, pwm_write_hook, t);
                if (t->cs[i].reg)  avr_register_io_write(avr, t->cs[i].reg,  pwm_write_hook, t);
            }

            // For each comparator (Channel)
            for (int i = 0; i < AVR_TIMER_COMP_COUNT; i++) {
                if (t->comp[i].r_ocr) {
                    // Hook OCR register (Duty change)
                    avr_register_io_write(avr, t->comp[i].r_ocr, pwm_write_hook, t);
                    if (t->comp[i].r_ocrh) avr_register_io_write(avr, t->comp[i].r_ocrh, pwm_write_hook, t);
                    
                    // Hook COM bits (Enable/Disable change)
                    if (t->comp[i].com.reg) avr_register_io_write(avr, t->comp[i].com.reg, pwm_write_hook, t);

                    // Map pin
                    int pin = -1;
                    if (t->comp[i].com_pin.reg) {
                        pin = get_arduino_pin(t->comp[i].com_pin.reg, t->comp[i].com_pin.bit);
                    }

                    if (pin >= 0 && channel_count < MAX_CHANNELS) {
                        channels[channel_count].timer = t;
                        channels[channel_count].comp_index = i;
                        channels[channel_count].arduino_pin = pin;
                        channel_count++;
                    }
                }
            }
        }
        port = port->next;
    }
    
    fprintf(stderr, "PWM: Initialized %d channels\n", channel_count);
}
