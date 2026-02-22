/*
 * avr_gpio_monitor.c
 *
 * GPIO monitoring for SimAVR - Unified GPIO Watcher Architecture
 * Monitors PORT register writes and sends pin state changes via named pipe
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "sim_avr.h"
#include "sim_io.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>

// GPIO Monitor state
typedef struct {
    avr_t *avr;
    FILE *gpio_pipe;    // Use FILE* like UART pipe for consistency
    pthread_t pipe_thread;  // Thread for opening pipe (like UART DMA)
    pthread_mutex_t mutex;  // Mutex to protect shared state (pipe, flags, prev values)
    volatile int pipe_ready;  // Flag to indicate pipe is ready
    volatile int running;     // Thread running flag
    char pipe_path[256];      // Path to named pipe
    uint8_t prev_portb;
    uint8_t prev_portc;
    uint8_t prev_portd;
    uint8_t prev_porte;  // For Leonardo
    uint8_t prev_portf;  // For Leonardo (analog)
    int initialized;     // Flag to indicate if monitor is fully initialized
} avr_gpio_monitor_t;

static avr_gpio_monitor_t *gpio_monitor = NULL;

// Pin mappings for different AVR boards
// PORTB pins (0x25)
static const uint8_t portb_pins_uno[] = {8, 9, 10, 11, 12, 13, 255, 255};  // PB0-PB5: D8-D13
// Leonardo PORTB mapping: PB0=17(A3), PB1=?, PB2=?, PB3=?, PB4=8, PB5=9, PB6=10, PB7=11
// Based on: D9=PB5, D10=PB6, D11=PB7, D8=PB4, D17=PB0
static const uint8_t portb_pins_leonardo[] = {17, 255, 255, 255, 8, 9, 10, 11};  // PB0=17(A3), PB4=8, PB5=9, PB6=10, PB7=11

// PORTC pins (0x28) - Analog pins
static const uint8_t portc_pins_uno[] = {14, 15, 16, 17, 18, 19, 255, 255};  // PC0-PC5 (A0-A5)
static const uint8_t portc_pins_leonardo[] = {14, 15, 16, 17, 18, 19, 255, 13};  // PC0-PC5 (A0-A5), PC7 = D13

// PORTD pins (0x2B)
static const uint8_t portd_pins[] = {0, 1, 2, 3, 4, 5, 6, 7};  // PD0-PD7

// PORTE pins (0x2E) - Leonardo only
static const uint8_t porte_pins[] = {255, 255, 255, 255, 255, 255, 7, 255};  // PE6 = D7

// PORTF pins (0x2F) - Leonardo analog
static const uint8_t portf_pins[] = {19, 18, 17, 16, 15, 14, 13, 12};  // PF0-PF7 (A5-A0, reversed)

// Thread function to open GPIO pipe (runs in separate thread, like UART DMA)
// This prevents blocking the main SimAVR process
static void* gpio_pipe_thread_func(void *param)
{
    avr_gpio_monitor_t *monitor = (avr_gpio_monitor_t *)param;
    
    fprintf(stderr, "🧵 GPIO pipe thread started\n");
    fflush(stderr);
    
    fprintf(stderr, "🔧 GPIO thread: About to fopen(%s, \"w\")\n", monitor->pipe_path);
    fflush(stderr);
    
    // Open pipe for writing (blocking, same as UART pipe)
    // This blocks until reader opens the pipe, but only blocks THIS thread
    // The main SimAVR process continues running
    FILE *pipe = fopen(monitor->pipe_path, "w");
    
    fprintf(stderr, "🔧 GPIO thread: fopen() returned\n");
    fflush(stderr);
    
    // Lock mutex before updating shared state
    pthread_mutex_lock(&monitor->mutex);
    
    if (!pipe) {
        fprintf(stderr, "❌ GPIO pipe thread: Failed to open pipe: %s (errno: %d)\n", 
                monitor->pipe_path, errno);
        fflush(stderr);
        monitor->gpio_pipe = NULL;
        monitor->pipe_ready = 0;
    } else {
        fprintf(stderr, "✅ GPIO pipe thread: Pipe opened: %s\n", monitor->pipe_path);
        fflush(stderr);
        // Disable buffering for immediate output (same as UART pipe)
        setbuf(pipe, NULL);
        monitor->gpio_pipe = pipe;
        monitor->pipe_ready = 1;
        fprintf(stderr, "✅ GPIO pipe thread: Pipe ready flag set\n");
        fflush(stderr);
    }
    
    // Unlock mutex after updating shared state
    pthread_mutex_unlock(&monitor->mutex);
    
    // Keep thread alive while monitor is running
    while (monitor->running) {
        usleep(100000);  // Sleep 100ms, check if still running
    }
    
    // Close pipe when thread exits (protected by mutex)
    pthread_mutex_lock(&monitor->mutex);
    if (monitor->gpio_pipe) {
        fclose(monitor->gpio_pipe);
        monitor->gpio_pipe = NULL;
        monitor->pipe_ready = 0;
        fprintf(stderr, "📪 GPIO pipe thread: Pipe closed\n");
    }
    pthread_mutex_unlock(&monitor->mutex);
    
    fprintf(stderr, "🧵 GPIO pipe thread stopped\n");
    return NULL;
}

// Write binary GPIO message to pipe
// Format: [Magic: 4 bytes] [Type: 1 byte] [Pin: 1 byte] [Value: 1 byte]
// Called from main thread (PORT register callbacks)
static void write_gpio_message(uint8_t pin, uint8_t value)
{
    if (!gpio_monitor) {
        return;
    }
    
    // Lock mutex to protect shared state (pipe_ready, gpio_pipe)
    pthread_mutex_lock(&gpio_monitor->mutex);
    
    // Check if pipe is ready and valid (protected by mutex)
    if (!gpio_monitor->pipe_ready || !gpio_monitor->gpio_pipe) {
        pthread_mutex_unlock(&gpio_monitor->mutex);
        return;
    }
    
    uint8_t msg[7] = {
        0x47, 0x50, 0x49, 0x4F,  // Magic: "GPIO"
        0x00,                     // Type: digital (0)
        pin,                      // Pin number
        value                     // Value (0 or 1)
    };
    
    // Write to pipe using fwrite (same approach as UART pipe)
    // Pipe is unbuffered (setbuf NULL), so writes are immediate
    // FILE* operations are internally synchronized, but we protect the FILE* pointer itself
    size_t written = fwrite(msg, 1, 7, gpio_monitor->gpio_pipe);
    
    // Unlock mutex before handling errors
    pthread_mutex_unlock(&gpio_monitor->mutex);
    
    // Handle errors gracefully - don't crash if pipe is closed
    if (written != 7) {
        // Pipe might be closed - that's OK, backend might have closed it
        // Just continue - next write will be a new message
    }
    // Success case - message written, continue normally
}

// Handle PORT register write
static void handle_port_write(uint8_t port_addr, uint8_t value, uint8_t prev_value)
{
    if (!gpio_monitor) return;
    
    uint8_t changed_bits = prev_value ^ value;
    if (changed_bits == 0) return;  // No changes
    
    // Debug: Log PORT register writes
    const char *port_name = "UNKNOWN";
    if (port_addr == 0x25) port_name = "PORTB";
    else if (port_addr == 0x28) port_name = "PORTC";
    else if (port_addr == 0x2B) port_name = "PORTD";
    else if (port_addr == 0x2E) port_name = "PORTE";
    else if (port_addr == 0x2F) port_name = "PORTF";
    
    fprintf(stderr, "🔍 GPIO Monitor: %s write detected: 0x%02X -> 0x%02X (changed bits: 0x%02X)\n", 
            port_name, prev_value, value, changed_bits);
    
    const uint8_t *pin_map = NULL;
    int pin_count = 8;
    
    // Determine which port and pin mapping
    if (port_addr == 0x25) {  // PORTB
        // Detect board type based on MCU
        // Check if we have access to avr to determine board type
        if (gpio_monitor && gpio_monitor->avr && gpio_monitor->avr->mmcu) {
            const char *mmcu = gpio_monitor->avr->mmcu;
            // Leonardo uses ATmega32U4
            if (strstr(mmcu, "32u4") || strstr(mmcu, "32U4")) {
                pin_map = portb_pins_leonardo;
            } else {
                // Default to Uno mapping (ATmega328P, etc.)
                pin_map = portb_pins_uno;
            }
        } else {
            // Fallback: use Uno mapping if we can't detect
            pin_map = portb_pins_uno;
        }
    } else if (port_addr == 0x28) {  // PORTC (analog pins)
        // Detect board type for PORTC mapping
        if (gpio_monitor && gpio_monitor->avr && gpio_monitor->avr->mmcu) {
            const char *mmcu = gpio_monitor->avr->mmcu;
            // Leonardo uses ATmega32U4 - D13 is on PORTC bit 7
            if (strstr(mmcu, "32u4") || strstr(mmcu, "32U4")) {
                pin_map = portc_pins_leonardo;
            } else {
                // Default to Uno mapping (ATmega328P, etc.)
                pin_map = portc_pins_uno;
            }
        } else {
            // Fallback: use Uno mapping if we can't detect
            pin_map = portc_pins_uno;
        }
    } else if (port_addr == 0x2B) {  // PORTD
        pin_map = portd_pins;
    } else if (port_addr == 0x2E) {  // PORTE (Leonardo)
        pin_map = porte_pins;
    } else if (port_addr == 0x2F) {  // PORTF (Leonardo analog)
        pin_map = portf_pins;
    } else {
        return;  // Unknown port
    }
    
    // Check each bit for changes
    for (int bit = 0; bit < pin_count; bit++) {
        if (changed_bits & (1 << bit)) {
            uint8_t pin = pin_map[bit];
            if (pin != 255) {  // Valid pin
                uint8_t high = (value >> bit) & 1;
                fprintf(stderr, "🔍 GPIO Monitor: %s bit %d -> Pin D%d = %s\n", 
                        port_name, bit, pin, high ? "HIGH" : "LOW");
                write_gpio_message(pin, high);
            } else {
                fprintf(stderr, "⚠️  GPIO Monitor: %s bit %d -> Pin 255 (invalid/mapped)\n", port_name, bit);
            }
        }
    }
}

// PORTB write callback
// Note: This is called when PORTB register is written
// We just monitor the write, don't modify it
static void portb_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t value, void *param)
{
    // Safety checks - CRITICAL: Check if monitor is fully initialized
    if (!gpio_monitor || !avr || addr != 0x25 || !gpio_monitor->initialized) {
        if (!gpio_monitor) fprintf(stderr, "⚠️  PORTB callback: gpio_monitor is NULL\n");
        if (!avr) fprintf(stderr, "⚠️  PORTB callback: avr is NULL\n");
        if (addr != 0x25) fprintf(stderr, "⚠️  PORTB callback: wrong addr 0x%02X\n", addr);
        if (!gpio_monitor->initialized) fprintf(stderr, "⚠️  PORTB callback: not initialized\n");
        return;
    }
    
    // Lock mutex to protect prev_portb access
    pthread_mutex_lock(&gpio_monitor->mutex);
    uint8_t prev = gpio_monitor->prev_portb;
    pthread_mutex_unlock(&gpio_monitor->mutex);
    
    // Only process if value actually changed
    if (prev == value) {
        return;
    }
    
    fprintf(stderr, "🔍 PORTB callback: 0x%02X -> 0x%02X (MCU: %s)\n", prev, value, avr->mmcu ? avr->mmcu : "unknown");
    
    // Don't write to register - let ioport module handle it
    // Just monitor the write value
    handle_port_write(0x25, value, prev);
    
    // Update our tracking (protected by mutex)
    pthread_mutex_lock(&gpio_monitor->mutex);
    gpio_monitor->prev_portb = value;
    pthread_mutex_unlock(&gpio_monitor->mutex);
}

// PORTC write callback
static void portc_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t value, void *param)
{
    if (!gpio_monitor || !avr || addr != 0x28 || !gpio_monitor->initialized) {
        return;
    }
    
    // Lock mutex to protect prev_portc access
    pthread_mutex_lock(&gpio_monitor->mutex);
    uint8_t prev = gpio_monitor->prev_portc;
    pthread_mutex_unlock(&gpio_monitor->mutex);
    
    if (prev == value) {
        return;
    }
    
    handle_port_write(0x28, value, prev);
    
    // Update our tracking (protected by mutex)
    pthread_mutex_lock(&gpio_monitor->mutex);
    gpio_monitor->prev_portc = value;
    pthread_mutex_unlock(&gpio_monitor->mutex);
}

// PORTD write callback
static void portd_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t value, void *param)
{
    if (!gpio_monitor || !avr || addr != 0x2B || !gpio_monitor->initialized) {
        return;
    }
    
    // Lock mutex to protect prev_portd access
    pthread_mutex_lock(&gpio_monitor->mutex);
    uint8_t prev = gpio_monitor->prev_portd;
    pthread_mutex_unlock(&gpio_monitor->mutex);
    
    if (prev == value) {
        return;
    }
    
    handle_port_write(0x2B, value, prev);
    
    // Update our tracking (protected by mutex)
    pthread_mutex_lock(&gpio_monitor->mutex);
    gpio_monitor->prev_portd = value;
    pthread_mutex_unlock(&gpio_monitor->mutex);
}

// PORTE write callback (Leonardo)
static void porte_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t value, void *param)
{
    if (!gpio_monitor || !avr || addr != 0x2E || !gpio_monitor->initialized) {
        return;
    }
    
    // Lock mutex to protect prev_porte access
    pthread_mutex_lock(&gpio_monitor->mutex);
    uint8_t prev = gpio_monitor->prev_porte;
    pthread_mutex_unlock(&gpio_monitor->mutex);
    
    if (prev == value) {
        return;
    }
    
    handle_port_write(0x2E, value, prev);
    
    // Update our tracking (protected by mutex)
    pthread_mutex_lock(&gpio_monitor->mutex);
    gpio_monitor->prev_porte = value;
    pthread_mutex_unlock(&gpio_monitor->mutex);
}

// PORTF write callback (Leonardo analog)
static void portf_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t value, void *param)
{
    if (!gpio_monitor || addr != 0x2F) return;
    
    // Lock mutex to protect prev_portf access
    pthread_mutex_lock(&gpio_monitor->mutex);
    uint8_t prev = gpio_monitor->prev_portf;
    pthread_mutex_unlock(&gpio_monitor->mutex);
    
    handle_port_write(0x2F, value, prev);
    
    // Update our tracking (protected by mutex)
    pthread_mutex_lock(&gpio_monitor->mutex);
    gpio_monitor->prev_portf = value;
    pthread_mutex_unlock(&gpio_monitor->mutex);
}

// Initialize GPIO monitor
void avr_gpio_monitor_init(avr_t *avr, const char *pipe_path)
{
    // DIAGNOSTIC: Log function entry with call location
    fprintf(stderr, "🔍 DIAG: avr_gpio_monitor_init() ENTERED\n");
    fprintf(stderr, "🔍 DIAG:   pipe_path=%s\n", pipe_path ? pipe_path : "(NULL)");
    fprintf(stderr, "🔍 DIAG:   gpio_monitor=%p (NULL means not initialized yet)\n", (void*)gpio_monitor);
    fflush(stderr);
    
    if (gpio_monitor) {
        // Already initialized
        fprintf(stderr, "⚠️  DIAG: avr_gpio_monitor_init() called but already initialized! (gpio_monitor=%p)\n", (void*)gpio_monitor);
        fprintf(stderr, "⚠️  DIAG: This is a DOUBLE INITIALIZATION - returning early\n");
        fflush(stderr);
        return;
    }
    
    fprintf(stderr, "🔍 DIAG: Allocating GPIO monitor structure...\n");
    fflush(stderr);
    gpio_monitor = calloc(1, sizeof(avr_gpio_monitor_t));
    if (!gpio_monitor) {
        fprintf(stderr, "❌ Failed to allocate GPIO monitor\n");
        return;
    }
    fprintf(stderr, "🔍 DIAG: GPIO monitor allocated at %p\n", (void*)gpio_monitor);
    fflush(stderr);
    
    // Initialize mutex for thread synchronization
    if (pthread_mutex_init(&gpio_monitor->mutex, NULL) != 0) {
        fprintf(stderr, "❌ Failed to initialize GPIO monitor mutex\n");
        free(gpio_monitor);
        gpio_monitor = NULL;
        return;
    }
    
    gpio_monitor->avr = avr;
    gpio_monitor->initialized = 0;  // Not initialized yet - set after callbacks registered
    gpio_monitor->gpio_pipe = NULL;  // Initialize gpio_pipe
    gpio_monitor->pipe_ready = 0;    // Pipe not ready yet
    gpio_monitor->running = 1;       // Thread running flag
    strncpy(gpio_monitor->pipe_path, pipe_path, sizeof(gpio_monitor->pipe_path) - 1);
    gpio_monitor->pipe_path[sizeof(gpio_monitor->pipe_path) - 1] = '\0';
    
    // Create named pipe if it doesn't exist
    // If pipe exists, try to remove it first (might be stale from previous run)
    if (access(pipe_path, F_OK) == 0) {
        // Pipe exists - try to remove it (might be stale)
        if (unlink(pipe_path) != 0 && errno != ENOENT) {
            fprintf(stderr, "⚠️  Could not remove existing GPIO pipe: %s (errno: %d)\n", 
                    pipe_path, errno);
            // Continue anyway - might still work
        }
    }
    
    // Create new pipe
    if (mkfifo(pipe_path, 0666) != 0) {
        if (errno != EEXIST) {
            fprintf(stderr, "❌ Failed to create GPIO pipe: %s (errno: %d)\n", 
                    pipe_path, errno);
            free(gpio_monitor);
            gpio_monitor = NULL;
            return;
        }
        // EEXIST means pipe already exists (race condition), that's OK
    }
    
    // NOTE: Thread creation is moved to avr_gpio_monitor_reset()
    // This is called during avr_reset(), same timing as UART DMA thread
    
    // Read initial PORT register values
    // Safety check: ensure avr->data is valid before accessing
    if (avr && avr->data) {
        gpio_monitor->prev_portb = avr->data[0x25];
        gpio_monitor->prev_portc = avr->data[0x28];
        gpio_monitor->prev_portd = avr->data[0x2B];
        
        // Check if PORTE and PORTF exist (Leonardo)
        if (avr->ramend >= 0x2E) {
            gpio_monitor->prev_porte = avr->data[0x2E];
        }
        if (avr->ramend >= 0x2F) {
            gpio_monitor->prev_portf = avr->data[0x2F];
        }
    } else {
        // Initialize to 0 if avr->data is not ready yet
        gpio_monitor->prev_portb = 0;
        gpio_monitor->prev_portc = 0;
        gpio_monitor->prev_portd = 0;
        gpio_monitor->prev_porte = 0;
        gpio_monitor->prev_portf = 0;
    }
    
    // Register callbacks for PORT registers
    // Note: We register callbacks that will be called when PORT registers are written
    // The ioport module also registers for these, so both callbacks will be called
    // We just monitor the write value, we don't modify it
    // 
    // IMPORTANT: Check shared IO slot limit before registering to avoid abort()
    // SimAVR has a limit of 16 shared IO register slots
    // Each PORT register that's already registered by ioport will use one slot
    
    int registered_count = 0;
    
    // Register PORTB (only if we have room for shared IO slots)
    // PORTB is already registered by ioport, so this will use a shared IO slot
    if (avr->io_shared_io_count < 16) {
        fprintf(stderr, "🔍 DIAG: Registering PORTB callback (0x25)...\n");
        fflush(stderr);
        avr_register_io_write(avr, 0x25, portb_write_callback, NULL);  // PORTB
        registered_count++;
        fprintf(stderr, "🔍 DIAG: PORTB callback registered successfully\n");
        fflush(stderr);
    } else {
        fprintf(stderr, "⚠️  GPIO monitor: Skipping PORTB callback (shared IO slot limit reached)\n");
        fflush(stderr);
    }
    
    // Register PORTC (only if we have room)
    if (avr->io_shared_io_count < 16) {
        fprintf(stderr, "🔍 DIAG: Registering PORTC callback (0x28)...\n");
        fflush(stderr);
        avr_register_io_write(avr, 0x28, portc_write_callback, NULL);  // PORTC
        registered_count++;
        fprintf(stderr, "🔍 DIAG: PORTC callback registered successfully\n");
        fflush(stderr);
    } else {
        fprintf(stderr, "⚠️  GPIO monitor: Skipping PORTC callback (shared IO slot limit reached)\n");
        fflush(stderr);
    }
    
    // Register PORTD (only if we have room)
    if (avr->io_shared_io_count < 16) {
        fprintf(stderr, "🔍 DIAG: Registering PORTD callback (0x2B)...\n");
        fflush(stderr);
        avr_register_io_write(avr, 0x2B, portd_write_callback, NULL);  // PORTD
        registered_count++;
        fprintf(stderr, "🔍 DIAG: PORTD callback registered successfully\n");
        fflush(stderr);
    } else {
        fprintf(stderr, "⚠️  GPIO monitor: Skipping PORTD callback (shared IO slot limit reached)\n");
        fflush(stderr);
    }
    
    fprintf(stderr, "✅ Registered %d PORT callbacks (B, C, D)\n", registered_count);
    fflush(stderr);
    
    // Mark as initialized AFTER all callbacks are registered
    // This prevents callbacks from being triggered during registration
    gpio_monitor->initialized = 1;
    
    // Only register PORTE and PORTF if they exist AND we have room in shared IO slots
    // SimAVR has a limit of 16 shared IO register slots
    // PORTB, PORTC, PORTD might each need a shared slot (if ioport already registered)
    // So we need to be conservative - only register PORTE/PORTF if we have room
    // 
    // Check if register address is valid and if ramend suggests these registers exist
    // For now, only register if ramend is high enough (suggests Leonardo/32U4)
    // and we haven't used up all shared IO slots yet
    avr_io_addr_t io_porte = AVR_DATA_TO_IO(0x2E);
    avr_io_addr_t io_portf = AVR_DATA_TO_IO(0x2F);
    
    // Conservative check: SimAVR has 4 shared IO slots (indices 0-3)
    // PORTB, PORTC, PORTD might each need a slot if ioport already registered
    // That's potentially 3 slots. We can safely register PORTE (slot 3) if available
    // But PORTF would need slot 4, which doesn't exist, so skip it
    // Only register PORTE if ramend clearly indicates it exists (Leonardo/32U4)
    // and we haven't exceeded the limit
    if (io_porte < MAX_IOs && avr->ramend >= 0x2E && avr->io_shared_io_count < 16) {
        avr_register_io_write(avr, 0x2E, porte_write_callback, NULL);  // PORTE
    }
    // Skip PORTF registration to avoid exceeding the 16-slot limit
    // Most projects don't need PORTF monitoring anyway
    
    fprintf(stderr, "✅ GPIO monitor initialized\n");
    fprintf(stderr, "🔍 DIAG: avr_gpio_monitor_init() COMPLETED successfully\n");
    fprintf(stderr, "🔍 DIAG:   gpio_monitor=%p\n", (void*)gpio_monitor);
    fprintf(stderr, "🔍 DIAG:   initialized=%d\n", gpio_monitor->initialized);
    fflush(stderr);
}

// Reset GPIO monitor (start pipe thread)
// Called during avr_reset() to start the pipe thread (like UART DMA)
void avr_gpio_monitor_reset(void)
{
    fprintf(stderr, "🔧 avr_gpio_monitor_reset() called\n");
    fflush(stderr);
    
    if (!gpio_monitor) {
        fprintf(stderr, "⚠️  avr_gpio_monitor_reset(): gpio_monitor is NULL (not initialized)\n");
        fflush(stderr);
        return;
    }
    
    // Check if thread already exists
    if (gpio_monitor->pipe_thread != 0) {
        fprintf(stderr, "⚠️  avr_gpio_monitor_reset(): Thread already exists (pipe_thread = %lu)\n", 
                (unsigned long)gpio_monitor->pipe_thread);
        fflush(stderr);
        return;
    }
    
    // Reset flags
    gpio_monitor->running = 1;
    gpio_monitor->pipe_ready = 0;
    
    fprintf(stderr, "🔧 avr_gpio_monitor_reset(): Creating thread...\n");
    fflush(stderr);
    
    // Create thread to open pipe (like UART DMA thread)
    // This prevents blocking the main SimAVR process
    int ret = pthread_create(&gpio_monitor->pipe_thread, NULL, gpio_pipe_thread_func, gpio_monitor);
    if (ret != 0) {
        fprintf(stderr, "❌ Failed to create GPIO pipe thread: %s\n", strerror(errno));
        fflush(stderr);
        return;
    }
    
    fprintf(stderr, "✅ GPIO pipe thread started (pipe will open in thread, thread ID: %lu)\n", 
            (unsigned long)gpio_monitor->pipe_thread);
    fflush(stderr);
}

// Cleanup GPIO monitor
void avr_gpio_monitor_cleanup(void)
{
    if (gpio_monitor) {
        // Mark as not initialized to prevent callbacks from running during cleanup
        gpio_monitor->initialized = 0;
        
        // Stop the pipe thread
        gpio_monitor->running = 0;
        
        // Wait for thread to exit (like UART DMA thread)
        if (gpio_monitor->pipe_thread) {
            pthread_join(gpio_monitor->pipe_thread, NULL);
            gpio_monitor->pipe_thread = 0;
        }
        
        // Note: We don't unlink the pipe here because:
        // 1. Backend might still be reading from it
        // 2. Backend will clean it up when simulation stops
        // 3. Multiple simulations might share the same pipe path
        
        free(gpio_monitor);
        gpio_monitor = NULL;
    }
}

