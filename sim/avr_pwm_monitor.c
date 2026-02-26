#define _XOPEN_SOURCE 700
/*
 * avr_pwm_monitor.c
 *
 * PWM Monitor for SimAVR — Unified Binary Protocol
 *
 * Sends 14-byte binary messages via named pipe:
 *   [PWM\0]  magic      (4 bytes)
 *   [pin]    Arduino#   (1 byte)
 *   [en]     enabled    (1 byte)
 *   [duty]   OCR value  (2 bytes LE, uint16)
 *   [top]    timer TOP  (2 bytes LE, uint16)
 *   [freq]   Hz         (4 bytes LE, uint32)
 *
 * Uses a pthread for blocking pipe open (like GPIO monitor),
 * then non-blocking writes from the main SimAVR thread.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <pthread.h>
#include "avr_pwm_monitor.h"
#include "avr_timer.h"
#include "avr_ioport.h"

#define PWM_MSG_SIZE 14

typedef struct {
    int          pipe_fd;
    int          pipe_ready;
    int          running;
    char         pipe_path[256];
    pthread_t    pipe_thread;
    pthread_mutex_t mutex;
} pwm_monitor_state_t;

static pwm_monitor_state_t *pwm_state = NULL;

/* ── Port address cache for Arduino pin mapping ──────────── */
static avr_io_addr_t addr_portb = 0;
static avr_io_addr_t addr_portc = 0;
static avr_io_addr_t addr_portd = 0;

static void resolve_port_addrs(avr_t *avr) {
    avr_io_t *port = avr->io_port;
    while (port) {
        if (strcmp(port->kind, "port") == 0) {
            avr_ioport_t *p = (avr_ioport_t *)port;
            if (p->name == 'B') addr_portb = p->r_port;
            if (p->name == 'C') addr_portc = p->r_port;
            if (p->name == 'D') addr_portd = p->r_port;
        }
        port = port->next;
    }
}

static int get_arduino_pin(avr_io_addr_t port_addr, int bit) {
    if (port_addr == 0) return -1;
    if (port_addr == addr_portd) return bit;       /* D0-D7  */
    if (port_addr == addr_portb) return 8 + bit;   /* D8-D13 */
    if (port_addr == addr_portc) return 14 + bit;  /* A0-A5  */
    return -1;
}

/* ── Per-channel state for change detection ──────────────── */
typedef struct {
    avr_timer_t *timer;
    int          comp_index;  /* 0..2 (A, B, C) */
    int          arduino_pin;
    uint32_t     last_freq;
    uint16_t     last_duty;
    uint16_t     last_top;
    uint8_t      last_enabled;
} pwm_channel_t;

#define MAX_CHANNELS 16
static pwm_channel_t channels[MAX_CHANNELS];
static int channel_count = 0;

/* ── Pipe thread: blocking open, keep alive ──────────────── */
static void *pwm_pipe_thread_func(void *arg)
{
    pwm_monitor_state_t *st = (pwm_monitor_state_t *)arg;

    fprintf(stderr, "🧵 PWM pipe thread started\n");
    fprintf(stderr, "🔧 PWM thread: About to open(%s, O_WRONLY) [blocking]\n",
            st->pipe_path);
    fflush(stderr);

    int fd = open(st->pipe_path, O_WRONLY);

    if (fd < 0) {
        fprintf(stderr, "❌ PWM pipe thread: open() failed: %s (errno=%d)\n",
                strerror(errno), errno);
        fflush(stderr);
        return NULL;
    }

    /* Switch to non-blocking for writes */
    fcntl(fd, F_SETFL, O_NONBLOCK);

    pthread_mutex_lock(&st->mutex);
    st->pipe_fd = fd;
    st->pipe_ready = 1;
    pthread_mutex_unlock(&st->mutex);

    fprintf(stderr, "✅ PWM pipe thread: Pipe opened: %s (fd=%d)\n",
            st->pipe_path, fd);
    fflush(stderr);

    /* Keep alive until told to stop */
    while (st->running) {
        usleep(500000);
    }

    pthread_mutex_lock(&st->mutex);
    if (st->pipe_fd >= 0) {
        close(st->pipe_fd);
        st->pipe_fd = -1;
    }
    st->pipe_ready = 0;
    pthread_mutex_unlock(&st->mutex);

    fprintf(stderr, "📪 PWM pipe thread: Pipe closed\n");
    fflush(stderr);
    return NULL;
}

/* ── Send 14-byte binary PWM message ─────────────────────── */
static void send_update(pwm_channel_t *ch)
{
    if (!pwm_state || ch->arduino_pin < 0) return;

    pthread_mutex_lock(&pwm_state->mutex);
    if (!pwm_state->pipe_ready || pwm_state->pipe_fd < 0) {
        pthread_mutex_unlock(&pwm_state->mutex);
        return;
    }

    uint8_t msg[PWM_MSG_SIZE] = {
        0x50, 0x57, 0x4D, 0x00,                     /* Magic: "PWM\0" */
        (uint8_t)ch->arduino_pin,                    /* Pin */
        ch->last_enabled,                            /* Enabled */
        (uint8_t)(ch->last_duty & 0xFF),             /* Duty lo */
        (uint8_t)((ch->last_duty >> 8) & 0xFF),      /* Duty hi */
        (uint8_t)(ch->last_top & 0xFF),              /* Top lo  */
        (uint8_t)((ch->last_top >> 8) & 0xFF),       /* Top hi  */
        (uint8_t)(ch->last_freq & 0xFF),             /* Freq byte 0 */
        (uint8_t)((ch->last_freq >>  8) & 0xFF),     /* Freq byte 1 */
        (uint8_t)((ch->last_freq >> 16) & 0xFF),     /* Freq byte 2 */
        (uint8_t)((ch->last_freq >> 24) & 0xFF)      /* Freq byte 3 */
    };

    ssize_t written = write(pwm_state->pipe_fd, msg, PWM_MSG_SIZE);
    if (written < 0 && errno == EPIPE) {
        close(pwm_state->pipe_fd);
        pwm_state->pipe_fd = -1;
        pwm_state->pipe_ready = 0;
    }

    pthread_mutex_unlock(&pwm_state->mutex);
}

/* ── Check a channel for config changes ──────────────────── */
static void check_channel_state(avr_t *avr, pwm_channel_t *ch)
{
    avr_timer_t *t = ch->timer;
    avr_timer_comp_t *comp = &t->comp[ch->comp_index];

    uint8_t mode = avr_regbit_get(avr, comp->com);
    uint8_t enabled = (mode != 0);

    uint32_t freq = 0;
    if (t->tov_cycles > 0 && t->cs_div_value > 0) {
        freq = avr->frequency / t->tov_cycles;
    } else {
        enabled = 0;
    }

    uint16_t duty = avr->data[comp->r_ocr];
    if (comp->r_ocrh) {
        duty |= (avr->data[comp->r_ocrh] << 8);
    }

    uint16_t top = t->tov_top;

    if (freq != ch->last_freq || duty != ch->last_duty ||
        top != ch->last_top || enabled != ch->last_enabled) {
        ch->last_freq = freq;
        ch->last_duty = duty;
        ch->last_top = top;
        ch->last_enabled = enabled;
        send_update(ch);
    }
}

/* ── IO write hook (runs on SimAVR main thread) ──────────── */
static void pwm_write_hook(struct avr_t *avr, avr_io_addr_t addr,
                           uint8_t v, void *param)
{
    avr_timer_t *timer = (avr_timer_t *)param;

    for (int i = 0; i < channel_count; i++) {
        if (channels[i].timer == timer) {
            check_channel_state(avr, &channels[i]);
        }
    }
}

/* ── Public: init ────────────────────────────────────────── */
void avr_pwm_monitor_init(avr_t *avr)
{
    if (pwm_state) return;  /* already initialised */

    const char *env_path = getenv("SIMAVR_PWM_PIPE");
    const char *path = env_path ? env_path : "/tmp/simavr_pwm.pipe";

    /* Ensure FIFO exists */
    struct stat st;
    if (stat(path, &st) == 0) {
        if (!S_ISFIFO(st.st_mode)) {
            unlink(path);
            mkfifo(path, 0666);
        }
    } else {
        mkfifo(path, 0666);
    }

    /* Allocate state */
    pwm_state = calloc(1, sizeof(pwm_monitor_state_t));
    if (!pwm_state) return;

    pwm_state->pipe_fd = -1;
    pwm_state->pipe_ready = 0;
    pwm_state->running = 1;
    strncpy(pwm_state->pipe_path, path, sizeof(pwm_state->pipe_path) - 1);
    pthread_mutex_init(&pwm_state->mutex, NULL);

    /* Start pipe thread (blocking open) */
    if (pthread_create(&pwm_state->pipe_thread, NULL,
                       pwm_pipe_thread_func, pwm_state) != 0) {
        fprintf(stderr, "❌ PWM: Failed to create pipe thread\n");
        free(pwm_state);
        pwm_state = NULL;
        return;
    }

    /* Resolve port addresses for Arduino pin mapping */
    resolve_port_addrs(avr);

    /* Discover timers and register hooks */
    avr_io_t *port = avr->io_port;
    while (port) {
        if (strcmp(port->kind, "timer") == 0) {
            avr_timer_t *t = (avr_timer_t *)port;

            /* Hook WGM/CS registers (TCCR) */
            for (int i = 0; i < 4; i++) {
                if (t->wgm[i].reg)
                    avr_register_io_write(avr, t->wgm[i].reg, pwm_write_hook, t);
                if (t->cs[i].reg)
                    avr_register_io_write(avr, t->cs[i].reg, pwm_write_hook, t);
            }

            /* For each comparator channel */
            for (int i = 0; i < AVR_TIMER_COMP_COUNT; i++) {
                if (t->comp[i].r_ocr) {
                    avr_register_io_write(avr, t->comp[i].r_ocr, pwm_write_hook, t);
                    if (t->comp[i].r_ocrh)
                        avr_register_io_write(avr, t->comp[i].r_ocrh, pwm_write_hook, t);
                    if (t->comp[i].com.reg)
                        avr_register_io_write(avr, t->comp[i].com.reg, pwm_write_hook, t);

                    int pin = -1;
                    if (t->comp[i].com_pin.reg) {
                        pin = get_arduino_pin(t->comp[i].com_pin.reg,
                                              t->comp[i].com_pin.bit);
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

    fprintf(stderr, "✅ PWM monitor initialized: %d channels, pipe=%s\n",
            channel_count, path);
    fflush(stderr);
}

/* ── Public: cleanup ─────────────────────────────────────── */
void avr_pwm_monitor_cleanup(void)
{
    if (!pwm_state) return;

    pwm_state->running = 0;
    pthread_join(pwm_state->pipe_thread, NULL);
    pthread_mutex_destroy(&pwm_state->mutex);
    free(pwm_state);
    pwm_state = NULL;
    channel_count = 0;

    fprintf(stderr, "📪 PWM monitor cleaned up\n");
    fflush(stderr);
}
