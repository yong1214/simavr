/*
 * avr_gpio_inject.c
 *
 * GPIO input injection for SimAVR - Unified GPIO Input Injection Architecture
 * Reads GPIO input injection commands from named pipe and sets pin input levels
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "sim_avr.h"
#include "sim_io.h"
#include "avr_ioport.h"
#include "avr_adc.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <stdbool.h>

// GPIO Inject state
typedef struct {
    avr_t *avr;
    FILE *gpio_inject_pipe;    // Use FILE* like UART pipe for consistency
    pthread_t pipe_thread;      // Thread for reading from pipe
    pthread_mutex_t mutex;      // Mutex to protect shared state (pipe, flags)
    volatile int pipe_ready;    // Flag to indicate pipe is ready
    volatile int running;       // Thread running flag
    char pipe_path[256];        // Path to named pipe
    int initialized;            // Flag to indicate if injector is fully initialized
} avr_gpio_inject_t;

static avr_gpio_inject_t *gpio_inject = NULL;

// Pin mappings for different AVR boards (same as monitor)
static const uint8_t portb_pins_uno[] = {8, 9, 10, 11, 12, 13, 255, 255};
static const uint8_t portb_pins_leonardo[] = {17, 255, 255, 255, 8, 9, 10, 11};
static const uint8_t portc_pins_uno[] = {14, 15, 16, 17, 18, 19, 255, 255};
static const uint8_t portc_pins_leonardo[] = {14, 15, 16, 17, 18, 19, 255, 13};
static const uint8_t portd_pins[] = {0, 1, 2, 3, 4, 5, 6, 7};
static const uint8_t porte_pins[] = {255, 255, 255, 255, 255, 255, 7, 255};

// Map Arduino pin number to PORT and bit
static int map_pin_to_port_bit(avr_t *avr, uint8_t arduino_pin, char *port, int *bit) {
    // Detect MCU type
    int is_leonardo = (strstr(avr->mmcu, "32u4") != NULL);
    
    // PORTD pins (PD0-PD7) -> D0-D7
    if (arduino_pin < 8) {
        *port = 'D';
        *bit = arduino_pin;
        return 0;
    }
    
    // PORTB pins
    if (arduino_pin >= 8 && arduino_pin <= 13) {
        *port = 'B';
        if (is_leonardo) {
            // Leonardo PORTB mapping
            for (int i = 0; i < 8; i++) {
                if (portb_pins_leonardo[i] == arduino_pin) {
                    *bit = i;
                    return 0;
                }
            }
        } else {
            // Uno PORTB mapping
            for (int i = 0; i < 8; i++) {
                if (portb_pins_uno[i] == arduino_pin) {
                    *bit = i;
                    return 0;
                }
            }
        }
    }
    
    // PORTC pins (analog)
    if (arduino_pin >= 14 && arduino_pin <= 19) {
        *port = 'C';
        if (is_leonardo) {
            for (int i = 0; i < 8; i++) {
                if (portc_pins_leonardo[i] == arduino_pin) {
                    *bit = i;
                    return 0;
                }
            }
        } else {
            for (int i = 0; i < 8; i++) {
                if (portc_pins_uno[i] == arduino_pin) {
                    *bit = i;
                    return 0;
                }
            }
        }
    }
    
    // PORTE pins (Leonardo only)
    if (is_leonardo && arduino_pin == 7) {
        *port = 'E';
        *bit = 6;
        return 0;
    }
    
    return -1; // Pin not found
}

// Thread function to read from pipe and inject GPIO inputs
static void *gpio_inject_thread_func(void *arg) {
    avr_gpio_inject_t *inject = (avr_gpio_inject_t *)arg;
    uint8_t buffer[8]; // Max message size: 8 bytes (analog with 2-byte value)
    
    fprintf(stderr, "🔧 GPIO inject thread: Started\n");
    fflush(stderr);
    
    // Open pipe for reading (blocking, like UART DMA thread)
    fprintf(stderr, "🔧 GPIO inject thread: About to fopen\n");
    fflush(stderr);
    FILE *pipe = fopen(inject->pipe_path, "r");
    
    // Lock mutex before updating shared state
    pthread_mutex_lock(&inject->mutex);
    
    if (!pipe) {
        fprintf(stderr, "❌ GPIO inject thread: Failed to open pipe: %s\n", strerror(errno));
        fflush(stderr);
        inject->gpio_inject_pipe = NULL;
        inject->pipe_ready = 0;
        inject->running = 0;
        pthread_mutex_unlock(&inject->mutex);
        return NULL;
    }
    
    fprintf(stderr, "✅ GPIO inject pipe opened: %s\n", inject->pipe_path);
    fflush(stderr);
    inject->gpio_inject_pipe = pipe;
    inject->pipe_ready = 1;
    
    // Unlock mutex after updating shared state
    pthread_mutex_unlock(&inject->mutex);
    
    while (inject->running) {
        // Get pipe reference (protected by mutex)
        pthread_mutex_lock(&inject->mutex);
        FILE *pipe = inject->gpio_inject_pipe;
        pthread_mutex_unlock(&inject->mutex);
        
        if (!pipe) {
            // Pipe not ready yet, wait a bit
            usleep(10000); // 10ms
            continue;
        }
        
        // Read header (6 bytes: magic + type + pin) to determine message length
        size_t bytes_read = fread(buffer, 1, 6, pipe);
        
        if (bytes_read < 6) {
            if (feof(pipe) || ferror(pipe)) {
                // Pipe closed or error
                fprintf(stderr, "🔧 GPIO inject thread: Pipe closed, exiting\n");
                fflush(stderr);
                break;
            }
            // Partial read, continue
            continue;
        }
        
        // Verify magic bytes
        if (buffer[0] != 0x47 || buffer[1] != 0x50 || buffer[2] != 0x49 || buffer[3] != 0x4F) {
            // Invalid magic - skip one byte and try again
            // Move buffer back by 5 bytes and try again
            memmove(buffer, buffer + 1, 5);
            continue;
        }
        
        // Parse header: [Magic: 4 bytes] [Type: 1 byte] [Pin: 1 byte]
        uint8_t type = buffer[4];
        uint8_t arduino_pin = buffer[5];
        
        // Determine message length based on type
        int value_length = (type == 2) ? 2 : 1; // Analog (2) = 2 bytes, Digital/PWM (0/1) = 1 byte
        int total_length = 6 + value_length; // Header (6) + value
        
        // Read remaining bytes (value) - always read, even if value_length is 1
        size_t value_bytes_read = fread(buffer + 6, 1, value_length, pipe);
        if (value_bytes_read < (size_t)value_length) {
            if (feof(pipe) || ferror(pipe)) {
                // Pipe closed or error
                fprintf(stderr, "🔧 GPIO inject thread: Pipe closed during value read, exiting\n");
                fflush(stderr);
                break;
            }
            // Partial read - skip this message
            continue;
        }
        
        // Parse value based on type
        uint16_t value = 0;
        if (type == 2) {
            // Analog: 2 bytes (little-endian)
            value = buffer[6] | (buffer[7] << 8);
        } else {
            // Digital/PWM: 1 byte
            value = buffer[6];
        }
        
        // Debug: Log the raw value being read
        fprintf(stderr, "🔍 GPIO inject: Read value from buffer[6] = 0x%02X (%u)\n", buffer[6], value);
        fflush(stderr);
        
        // Check initialization state (protected by mutex)
        pthread_mutex_lock(&inject->mutex);
        int is_initialized = inject->initialized;
        avr_t *avr = inject->avr;
        pthread_mutex_unlock(&inject->mutex);
        
        if (!is_initialized || !avr) {
            continue;
        }
        
        // Map Arduino pin to PORT and bit
        char port;
        int bit;
        if (map_pin_to_port_bit(avr, arduino_pin, &port, &bit) != 0) {
            // Pin not found
            continue;
        }
        
        // Get PORT register address (data space address)
        avr_io_addr_t port_data_addr = 0;
        if (port == 'B') port_data_addr = 0x25;
        else if (port == 'C') port_data_addr = 0x28;
        else if (port == 'D') port_data_addr = 0x2B;
        else if (port == 'E') port_data_addr = 0x2E;
        else continue;
        
        // Use IOPORT ioctl to safely get the IRQ for this port
        // This is the recommended way to access IOPORT IRQs and handles shared IO slots correctly
        avr_irq_t *pin_irq = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ(port), IOPORT_IRQ_PIN0 + bit);
        
        if (!pin_irq) {
            fprintf(stderr, "⚠️  GPIO inject: Failed to get IRQ for PORT%c pin %d\n", port, bit);
            fflush(stderr);
            continue;
        }
        
        // Validate bit is within bounds (0-7)
        if (bit < 0 || bit > 7) {
            fprintf(stderr, "⚠️  GPIO inject: Invalid bit number %d for PORT%c (must be 0-7)\n", bit, port);
            fflush(stderr);
            continue;
        }
        
        // Get PIN register address to verify it gets updated
        // PORTD PIN register is at 0x29 (for PORTD), PORTB at 0x23, PORTC at 0x26
        avr_io_addr_t pin_data_addr = 0;
        if (port == 'B') pin_data_addr = 0x23;
        else if (port == 'C') pin_data_addr = 0x26;
        else if (port == 'D') pin_data_addr = 0x29;
        else if (port == 'E') pin_data_addr = 0x2C;
        else pin_data_addr = 0;
        
        // Read current PIN register value before injection
        uint8_t pin_before = (pin_data_addr > 0 && avr->data) ? avr->data[pin_data_addr] : 0;
        
        // Raise IRQ on the pin (this simulates external input)
        // IRQ index: IOPORT_IRQ_PIN0 (0) + bit (0-7)
        // This should trigger avr_ioport_irq_notify which updates the PIN register
        if (type == 2) {
            // Analog: Inject ADC value via ADC peripheral
            // Arduino analog pins A0-A5 map to ADC channels 0-5
            int adc_channel = arduino_pin - 14; // A0 = D14 -> channel 0, A1 = D15 -> channel 1, etc.
            
            if (adc_channel >= 0 && adc_channel <= 5) {
                // Get ADC peripheral from AVR
                avr_irq_t *adc_irq = avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0 + adc_channel);
                
                if (adc_irq) {
                    // Convert ADC value (0-1023) to millivolts
                    // AVR ADC uses 5V reference, so: millivolts = (value / 1023) * 5000
                    uint32_t millivolts = (value * 5000) / 1023;
                    
                    // Inject ADC value via IRQ (SimAVR expects millivolts)
                    avr_raise_irq(adc_irq, millivolts);
                    
                    fprintf(stderr, "🔌 ADC inject: A%d (channel %d) = %d (0x%04X) = %d mV\n", 
                            adc_channel, adc_channel, value, value, millivolts);
                    fflush(stderr);
                } else {
                    fprintf(stderr, "⚠️ ADC channel %d IRQ not available\n", adc_channel);
                    fflush(stderr);
                }
            } else {
                fprintf(stderr, "⚠️ Invalid ADC pin: A%d (Arduino pin %d)\n", adc_channel, arduino_pin);
                fflush(stderr);
            }
        } else {
            // Digital/PWM: Use value directly (0 or 1 for digital, 0-255 for PWM)
            bool high = (value != 0);
            avr_raise_irq(pin_irq, high ? 1 : 0);
            
            // Read PIN register value after injection to verify it was updated
            uint8_t pin_after = (pin_data_addr > 0 && avr->data) ? avr->data[pin_data_addr] : 0;
            uint8_t pin_bit_before = (pin_before >> bit) & 1;
            uint8_t pin_bit_after = (pin_after >> bit) & 1;
            
            fprintf(stderr, "🔌 GPIO inject: D%d (PORT%c%d) = %s (PIN reg: 0x%02X->0x%02X, bit %d: %d->%d)\n", 
                    arduino_pin, port, bit, high ? "HIGH" : "LOW",
                    pin_before, pin_after, bit, pin_bit_before, pin_bit_after);
            fflush(stderr);
        }
    }
    
    // Close pipe when thread exits (protected by mutex)
    pthread_mutex_lock(&inject->mutex);
    if (inject->gpio_inject_pipe) {
        fclose(inject->gpio_inject_pipe);
        inject->gpio_inject_pipe = NULL;
    }
    inject->pipe_ready = 0;
    pthread_mutex_unlock(&inject->mutex);
    fprintf(stderr, "🔧 GPIO inject thread: Exiting\n");
    fflush(stderr);
    return NULL;
}

// Initialize GPIO injector
void avr_gpio_inject_init(avr_t *avr, const char *pipe_path) {
    if (gpio_inject) {
        // Already initialized
        return;
    }
    
    if (!avr || !pipe_path) {
        fprintf(stderr, "❌ GPIO inject: Invalid parameters\n");
        return;
    }
    
    gpio_inject = calloc(1, sizeof(avr_gpio_inject_t));
    if (!gpio_inject) {
        fprintf(stderr, "❌ GPIO inject: Failed to allocate memory\n");
        return;
    }
    
    // Initialize mutex for thread synchronization
    if (pthread_mutex_init(&gpio_inject->mutex, NULL) != 0) {
        fprintf(stderr, "❌ Failed to initialize GPIO inject mutex\n");
        free(gpio_inject);
        gpio_inject = NULL;
        return;
    }
    
    gpio_inject->avr = avr;
    strncpy(gpio_inject->pipe_path, pipe_path, sizeof(gpio_inject->pipe_path) - 1);
    gpio_inject->pipe_path[sizeof(gpio_inject->pipe_path) - 1] = '\0';
    gpio_inject->pipe_ready = 0;
    gpio_inject->running = 0;
    gpio_inject->initialized = 0;
    gpio_inject->gpio_inject_pipe = NULL;
    
    fprintf(stderr, "🔌 Initializing GPIO injector...\n");
    fflush(stderr);
    
    // Mark as initialized (thread will start in reset)
    gpio_inject->initialized = 1;
}

// Start GPIO inject thread (called from avr_reset or similar)
void avr_gpio_inject_reset(void) {
    if (!gpio_inject || !gpio_inject->initialized) {
        return;
    }
    
    if (gpio_inject->running) {
        // Already running
        return;
    }
    
    gpio_inject->running = 1;
    
    // Create thread to read from pipe
    if (pthread_create(&gpio_inject->pipe_thread, NULL, gpio_inject_thread_func, gpio_inject) != 0) {
        fprintf(stderr, "❌ GPIO inject: Failed to create thread: %s\n", strerror(errno));
        gpio_inject->running = 0;
        return;
    }
    
    fprintf(stderr, "✅ GPIO inject thread started\n");
    fflush(stderr);
}

// Cleanup GPIO injector
void avr_gpio_inject_cleanup(void) {
    if (!gpio_inject) {
        return;
    }
    
    gpio_inject->initialized = 0;
    gpio_inject->running = 0;
    
    // Wait for thread to exit
    if (gpio_inject->pipe_thread) {
        pthread_join(gpio_inject->pipe_thread, NULL);
    }
    
    // Close pipe (protected by mutex)
    pthread_mutex_lock(&gpio_inject->mutex);
    if (gpio_inject->gpio_inject_pipe) {
        fclose(gpio_inject->gpio_inject_pipe);
        gpio_inject->gpio_inject_pipe = NULL;
    }
    pthread_mutex_unlock(&gpio_inject->mutex);
    
    // Destroy mutex
    pthread_mutex_destroy(&gpio_inject->mutex);
    
    free(gpio_inject);
    gpio_inject = NULL;
}

