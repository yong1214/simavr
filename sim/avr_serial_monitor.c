/*
 * avr_serial_monitor.c
 *
 * Serial interface monitoring for SimAVR
 * Monitors SPI, I2C, and UART register writes and sends transaction data via named pipe
 *
 * Copyright (c) 2025 Dustalon Project
 */

#include "sim_avr.h"
#include "sim_io.h"
#include "avr_twi.h"
#include "../include/sim_log.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>

// Serial Monitor state
typedef struct {
    avr_t *avr;
    FILE *serial_pipe;    // Use FILE* like UART pipe for consistency
    pthread_t pipe_thread;  // Thread for opening pipe (like UART DMA)
    volatile int pipe_ready;  // Flag to indicate pipe is ready
    volatile int running;     // Thread running flag
    char pipe_path[256];      // Path to named pipe
    int initialized;           // Flag to indicate if monitor is fully initialized
    
    // I2C state tracking
    uint8_t i2c_prev_twcr;     // Previous TWCR value (for START/STOP detection)
    uint8_t i2c_address;       // Current I2C address
    uint8_t i2c_direction;     // Current I2C direction (0=write, 1=read)
    avr_irq_t *twi_irq_output; // TWI IRQ output for monitoring I2C transactions
} avr_serial_monitor_t;

static avr_serial_monitor_t *serial_monitor = NULL;

// Thread function to open serial pipe (runs in separate thread, like UART DMA)
static void* serial_pipe_thread_func(void *param)
{
    avr_serial_monitor_t *monitor = (avr_serial_monitor_t *)param;
    
    fprintf(stderr, "🧵 Serial pipe thread started\n");
    fflush(stderr);
    
    fprintf(stderr, "🔧 Serial thread: About to fopen(%s, \"w\")\n", monitor->pipe_path);
    fflush(stderr);
    
    // Open pipe for writing (blocking, same as UART pipe)
    monitor->serial_pipe = fopen(monitor->pipe_path, "w");
    
    fprintf(stderr, "🔧 Serial thread: fopen() returned\n");
    fflush(stderr);
    
    if (!monitor->serial_pipe) {
        fprintf(stderr, "❌ Serial pipe thread: Failed to open pipe: %s (errno: %d)\n", 
                monitor->pipe_path, errno);
        fflush(stderr);
        monitor->pipe_ready = 0;
    } else {
        fprintf(stderr, "✅ Serial pipe thread: Pipe opened: %s\n", monitor->pipe_path);
        fflush(stderr);
        // Disable buffering for immediate output
        setbuf(monitor->serial_pipe, NULL);
        monitor->pipe_ready = 1;
        fprintf(stderr, "✅ Serial pipe thread: Pipe ready flag set\n");
        fflush(stderr);
    }
    
    // Keep thread alive while monitor is running
    while (monitor->running) {
        usleep(100000);  // Sleep 100ms, check if still running
    }
    
    // Close pipe when thread exits
    if (monitor->serial_pipe) {
        fclose(monitor->serial_pipe);
        monitor->serial_pipe = NULL;
        monitor->pipe_ready = 0;
        fprintf(stderr, "📪 Serial pipe thread: Pipe closed\n");
    }
    
    fprintf(stderr, "🧵 Serial pipe thread stopped\n");
    return NULL;
}

// Get current timestamp in microseconds
static uint64_t get_timestamp_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

// Write binary serial message to pipe
// Format: [Magic: 4 bytes] [Type: 1 byte] [Interface: 1 byte] [Direction: 1 byte] [Address: 1 byte] [Data: 1 byte] [Timestamp: 8 bytes]
// Type: 0x00=SPI, 0x01=I2C, 0x02=UART
// Interface: Interface number (SPI0=0, I2C0=0, UART0=0, etc.)
// Direction: SPI (0x00=MOSI, 0x01=MISO), I2C (0x00=Write, 0x01=Read), UART (0x00=TX, 0x01=RX)
// Address: I2C address (7-bit, 0x00-0x7F) for I2C transactions, 0xFF for SPI/UART (not applicable)
// Data: Data byte
// Note: Message size increased from 15 to 16 bytes to include I2C address
static void write_serial_message(uint8_t type, uint8_t interface, uint8_t direction, uint8_t address, uint8_t data)
{
    if (!serial_monitor || !serial_monitor->pipe_ready || !serial_monitor->serial_pipe) {
        return;
    }
    
    uint64_t timestamp = get_timestamp_us();
    
    uint8_t msg[17] = {
        0x53, 0x45, 0x52, 0x4C,  // Magic: "SERL" (4 bytes)
        type,                     // Type: SPI/I2C/UART (1 byte)
        interface,                // Interface number (1 byte)
        direction,                // Direction (1 byte)
        address,                  // I2C address (7-bit) or 0xFF for SPI/UART (1 byte)
        data,                     // Data byte (1 byte)
        // Timestamp (8 bytes, little-endian)
        (uint8_t)(timestamp & 0xFF),
        (uint8_t)((timestamp >> 8) & 0xFF),
        (uint8_t)((timestamp >> 16) & 0xFF),
        (uint8_t)((timestamp >> 24) & 0xFF),
        (uint8_t)((timestamp >> 32) & 0xFF),
        (uint8_t)((timestamp >> 40) & 0xFF),
        (uint8_t)((timestamp >> 48) & 0xFF),
        (uint8_t)((timestamp >> 56) & 0xFF)
    };
    
    // Write to pipe using fwrite
    // Total: 4 (magic) + 1 (type) + 1 (interface) + 1 (direction) + 1 (address) + 1 (data) + 8 (timestamp) = 17 bytes
    size_t written = fwrite(msg, 1, 17, serial_monitor->serial_pipe);
    
    // Handle errors gracefully
    if (written != 15) {
        // Pipe might be closed - that's OK
    }
}

// SPI Data Register (SPDR) write callback
static void spdr_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    if (!serial_monitor || !serial_monitor->initialized) return;
    
    // Read final value after all handlers have processed
    uint8_t final_value = avr->data[addr];
    
    // Determine direction: MOSI (master out) or MISO (master in)
    // For now, assume MOSI (master writes to SPDR)
    // TODO: Detect master/slave mode from SPCR register
    uint8_t direction = 0x00;  // MOSI
    
    // Write serial transaction message
    write_serial_message(0x00, 0x00, direction, 0xFF, final_value);  // SPI, SPI0, MOSI, address=0xFF (N/A), data
}

// I2C Data Register (TWDR) write callback
static void twdr_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    if (!serial_monitor || !serial_monitor->initialized) return;
    
    // Read final value
    uint8_t final_value = avr->data[addr];
    
    // Debug: Log TWDR writes
    fprintf(stderr, "🔍 Serial Monitor: TWDR write = 0x%02X, i2c_address=0x%02X\n", final_value, serial_monitor->i2c_address);
    fflush(stderr);
    
    // Check if this is the address byte (first byte after START)
    // If i2c_address is 0xFF, this is likely the address byte
    if (serial_monitor->i2c_address == 0xFF) {
        // Address format: 7-bit address + R/W bit (bit 0)
        // R/W = 0 means write, R/W = 1 means read
        uint8_t address = final_value >> 1;  // 7-bit address
        uint8_t direction = (final_value & 0x01) ? 0x01 : 0x00;  // Read=1, Write=0
        serial_monitor->i2c_address = address;
        serial_monitor->i2c_direction = direction;
        fprintf(stderr, "📡 Serial Monitor: I2C address detected: 0x%02X, direction=%s\n", address, direction ? "read" : "write");
        fflush(stderr);
        // Don't send address byte as data - it's metadata
        return;
    }
    
    // Use stored I2C direction and address for data bytes
    uint8_t direction = serial_monitor->i2c_direction;
    uint8_t address = serial_monitor->i2c_address;  // Include I2C address in message
    
    fprintf(stderr, "📡 Serial Monitor: I2C data byte: address=0x%02X, direction=%s, data=0x%02X\n", address, direction ? "read" : "write", final_value);
    fflush(stderr);
    
    // Write serial transaction message
    write_serial_message(0x01, 0x00, direction, address, final_value);  // I2C, I2C0, direction, address, data
}

// I2C Control Register (TWCR) write callback - detect START/STOP
static void twcr_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    if (!serial_monitor || !serial_monitor->initialized) return;
    
    uint8_t prev_twcr = serial_monitor->i2c_prev_twcr;
    uint8_t new_twcr = v;
    
    // Check for START condition (TWSTA bit set)
    if ((new_twcr & 0x20) && !(prev_twcr & 0x20)) {
        // START condition detected
        fprintf(stderr, "🚀 Serial Monitor: I2C START detected\n");
        fflush(stderr);
        // Reset address tracking - next TWDR write will contain the address
        serial_monitor->i2c_address = 0xFF;
        serial_monitor->i2c_direction = 0x00;  // Default to write
    }
    
    // Check for STOP condition (TWSTO bit set)
    if ((new_twcr & 0x10) && !(prev_twcr & 0x10)) {
        // STOP condition detected
        fprintf(stderr, "🛑 Serial Monitor: I2C STOP detected\n");
        fflush(stderr);
        // Transaction complete - reset address tracking
        serial_monitor->i2c_address = 0xFF;
        serial_monitor->i2c_direction = 0x00;
    }
    
    serial_monitor->i2c_prev_twcr = new_twcr;
}

// I2C Data Register (TWDR) read callback
// Monitor when data is read from TWDR (I2C read transactions)
static uint8_t twdr_read_callback(avr_t *avr, avr_io_addr_t addr, void *param)
{
    if (!serial_monitor || !serial_monitor->initialized) {
        // Return the register value if monitor not ready
        return avr->data[addr];
    }
    
    // Read the value from the register
    uint8_t value = avr->data[addr];
    
    // Monitor I2C read transaction
    // Use stored I2C direction (should be read direction for this callback)
    uint8_t direction = 0x01;  // Read direction
    uint8_t address = serial_monitor->i2c_address;  // Include I2C address in message
    
    // Write serial transaction message
    write_serial_message(0x01, 0x00, direction, address, value);  // I2C, I2C0, Read, address, data
    
    return value;
}

// I2C IRQ-based monitoring (preferred method - captures all I2C transactions)
// This monitors TWI_IRQ_OUTPUT to capture I2C transactions at the IRQ level
// This works even when virtual devices are handling I2C transactions
static void twi_irq_output_notify(struct avr_irq_t *irq, uint32_t value, void *param)
{
    if (!serial_monitor || !serial_monitor->initialized) return;
    
    avr_twi_msg_irq_t msg;
    msg.u.v = value;
    
    // Extract I2C transaction details
    uint8_t msg_type = msg.u.twi.msg;
    uint8_t address = msg.u.twi.addr >> 1;  // 7-bit address (remove R/W bit)
    uint8_t direction = (msg.u.twi.addr & 0x01) ? 0x01 : 0x00;  // Read=1, Write=0
    uint8_t data = msg.u.twi.data;
    
    // Handle START condition
    if (msg_type & TWI_COND_START) {
        // DISABLED: Too verbose
        // fprintf(stderr, "🚀 Serial Monitor (IRQ): I2C START - address=0x%02X, direction=%s\n", 
        //         address, direction ? "read" : "write");
        serial_monitor->i2c_address = address;
        serial_monitor->i2c_direction = direction;
        // Don't send START as data - it's metadata
        return;
    }
    
    // Handle STOP condition
    if (msg_type & TWI_COND_STOP) {
        // DISABLED: Too verbose
        // fprintf(stderr, "🛑 Serial Monitor (IRQ): I2C STOP\n");
        serial_monitor->i2c_address = 0xFF;
        serial_monitor->i2c_direction = 0x00;
        return;
    }
    
    // Handle data transactions (WRITE or READ)
    if ((msg_type & TWI_COND_WRITE) || (msg_type & TWI_COND_READ)) {
        // DISABLED: Too verbose
        // fprintf(stderr, "📡 Serial Monitor (IRQ): I2C %s - address=0x%02X, data=0x%02X\n", 
        //         (msg_type & TWI_COND_WRITE) ? "write" : "read", address, data);
        
        // Send serial transaction message
        write_serial_message(0x01, 0x00, direction, address, data);  // I2C, I2C0, direction, address, data
    }
}

// UART Data Register (UDR) write callback
// Note: This is separate from the UART DMA pipe which handles Serial.println output
// This monitors UDR register writes for transaction visualization
static void udr_write_callback(avr_t *avr, avr_io_addr_t addr, uint8_t v, void *param)
{
    if (!serial_monitor || !serial_monitor->initialized) return;
    
    // Read final value
    uint8_t final_value = avr->data[addr];
    
    // Determine UART number from address
    // UDR0 = 0xC6, UDR1 = varies by MCU
    uint8_t uart_num = 0x00;  // Default to UART0
    if (addr == 0xC6) {
        uart_num = 0x00;  // UART0
    } else if (addr == 0xCE) {
        uart_num = 0x01;  // UART1 (Leonardo)
    }
    
    // Write serial transaction message (TX direction)
    write_serial_message(0x02, uart_num, 0x00, 0xFF, final_value);  // UART, UARTn, TX, address=0xFF (N/A), data
}

// Initialize serial monitor
void avr_serial_monitor_init(avr_t *avr, const char *pipe_path)
{
    if (serial_monitor) {
        // Already initialized
        return;
    }
    
    if (!avr || !pipe_path) {
        fprintf(stderr, "❌ Serial monitor: Invalid parameters\n");
        return;
    }
    
    serial_monitor = calloc(1, sizeof(avr_serial_monitor_t));
    if (!serial_monitor) {
        fprintf(stderr, "❌ Serial monitor: Failed to allocate memory\n");
        return;
    }
    
    serial_monitor->avr = avr;
    strncpy(serial_monitor->pipe_path, pipe_path, sizeof(serial_monitor->pipe_path) - 1);
    serial_monitor->pipe_path[sizeof(serial_monitor->pipe_path) - 1] = '\0';
    serial_monitor->pipe_ready = 0;
    serial_monitor->running = 0;
    serial_monitor->initialized = 0;
    serial_monitor->i2c_prev_twcr = 0;
    serial_monitor->i2c_address = 0xFF;
    serial_monitor->i2c_direction = 0x00;
    serial_monitor->twi_irq_output = NULL;
    
    // Register I2C IRQ-based monitoring (preferred method - works with virtual devices)
    // This monitors TWI_IRQ_OUTPUT to capture all I2C transactions at the IRQ level
    serial_monitor->twi_irq_output = avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_OUTPUT);
    if (serial_monitor->twi_irq_output) {
        avr_irq_register_notify(serial_monitor->twi_irq_output, twi_irq_output_notify, serial_monitor);
        SIM_LOG_INFO(LOG_CAT_SERIAL, "I2C IRQ-based monitoring registered (TWI_IRQ_OUTPUT)");
    } else {
        SIM_LOG_WARN(LOG_CAT_SERIAL, "Failed to get TWI_IRQ_OUTPUT, falling back to register monitoring");
    }
    
    // Register callbacks for SPI, I2C, and UART registers (fallback method)
    // Note: Register addresses vary by MCU, so we need to detect them
    // IMPORTANT: Check shared IO slot limit before registering to avoid abort()
    // SimAVR has a limit of 4 shared IO register slots
    // Each register that needs a muxer (because it's already registered) uses one slot
    
    // SPI: SPDR register (varies by MCU)
    // ATmega328P: SPDR = 0x2E
    // ATmega32U4: SPDR = 0x4E
    avr_io_addr_t spdr_addr = 0x2E;  // Default for Uno
    if (strstr(avr->mmcu, "32u4")) {
        spdr_addr = 0x4E;  // Leonardo
    }
    
    // Register SPI callback (only if we have room for shared IO slots)
    // SPDR might already be registered by SPI module, so this could use a slot
    if (avr->io_shared_io_count < 4) {
        avr_register_io_write(avr, spdr_addr, spdr_write_callback, NULL);
    } else {
        SIM_LOG_WARN(LOG_CAT_SERIAL, "Skipping SPDR callback (shared IO slot limit reached)");
    }
    
    // I2C: TWDR register (varies by MCU)
    // ATmega328P: TWDR = 0xBB
    // ATmega32U4: TWDR = 0xBB
    avr_io_addr_t twdr_addr = 0xBB;
    
    fprintf(stderr, "🔍 Serial monitor: Checking shared IO slots (count=%d, limit=4)\n", avr->io_shared_io_count);
    fflush(stderr);
    
    // Register I2C callbacks (only if we have room)
    // TWDR might already be registered by I2C module, so this could use slots
    if (avr->io_shared_io_count < 3) {  // Need 2 slots (write + read)
        fprintf(stderr, "🔍 Serial monitor: Registering TWDR write callback...\n");
        fflush(stderr);
        avr_register_io_write(avr, twdr_addr, twdr_write_callback, NULL);
        fprintf(stderr, "🔍 Serial monitor: Registering TWDR read callback...\n");
        fflush(stderr);
        avr_register_io_read(avr, twdr_addr, twdr_read_callback, NULL);
        fprintf(stderr, "✅ Serial monitor: TWDR callbacks registered (shared_io_count=%d after registration)\n", avr->io_shared_io_count);
        fflush(stderr);
    } else {
        fprintf(stderr, "⚠️  Serial monitor: Skipping TWDR callbacks (shared IO slot limit reached, count=%d)\n", avr->io_shared_io_count);
        fflush(stderr);
    }
    
    // TWCR register for START/STOP detection
    avr_io_addr_t twcr_addr = 0xBC;
    if (avr->io_shared_io_count < 4) {
        avr_register_io_write(avr, twcr_addr, twcr_write_callback, NULL);
    } else {
        SIM_LOG_WARN(LOG_CAT_SERIAL, "Skipping TWCR callback (shared IO slot limit reached)");
    }
    
    // UART: UDR register (varies by UART number and MCU)
    // UDR0: ATmega328P = 0xC6, ATmega32U4 = 0xC6
    // UDR1: ATmega32U4 = 0xCE
    // NOTE: UDR0 is already registered by UART DMA, so this will use a shared IO slot
    avr_io_addr_t udr0_addr = 0xC6;
    if (avr->io_shared_io_count < 4) {
        avr_register_io_write(avr, udr0_addr, udr_write_callback, NULL);
    } else {
        fprintf(stderr, "⚠️  Serial monitor: Skipping UDR0 callback (shared IO slot limit reached)\n");
    }
    
    // UDR1 for Leonardo
    if (strstr(avr->mmcu, "32u4")) {
        if (avr->io_shared_io_count < 4) {
            avr_io_addr_t udr1_addr = 0xCE;
            avr_register_io_write(avr, udr1_addr, udr_write_callback, NULL);
        } else {
            fprintf(stderr, "⚠️  Serial monitor: Skipping UDR1 callback (shared IO slot limit reached)\n");
        }
    }
    
    serial_monitor->initialized = 1;
    fprintf(stderr, "✅ Serial monitor initialized\n");
}

// Reset serial monitor (start pipe thread)
void avr_serial_monitor_reset(void)
{
    if (!serial_monitor || !serial_monitor->initialized) {
        fprintf(stderr, "⚠️  avr_serial_monitor_reset(): serial_monitor is NULL (not initialized)\n");
        return;
    }
    
    if (serial_monitor->running) {
        // Already running
        return;
    }
    
    serial_monitor->running = 1;
    serial_monitor->pipe_ready = 0;
    
    // Start pipe thread
    if (pthread_create(&serial_monitor->pipe_thread, NULL, serial_pipe_thread_func, serial_monitor) != 0) {
        fprintf(stderr, "❌ Failed to create serial pipe thread\n");
        serial_monitor->running = 0;
        return;
    }
    
    fprintf(stderr, "✅ Serial monitor pipe thread created\n");
}

// Cleanup serial monitor
void avr_serial_monitor_cleanup(void)
{
    if (!serial_monitor) {
        return;
    }
    
    serial_monitor->initialized = 0;
    serial_monitor->running = 0;
    
    // Wait for thread to exit
    if (serial_monitor->pipe_thread) {
        pthread_join(serial_monitor->pipe_thread, NULL);
        serial_monitor->pipe_thread = 0;
    }
    
    // Close pipe
    if (serial_monitor->serial_pipe) {
        fclose(serial_monitor->serial_pipe);
        serial_monitor->serial_pipe = NULL;
    }
    
    free(serial_monitor);
    serial_monitor = NULL;
    
    fprintf(stderr, "✅ Serial monitor cleaned up\n");
}

