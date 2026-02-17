/*
 * uart_dma.c
 *
 * DMA-style UART transmission simulation using a separate thread.
 * This makes UART transmission autonomous, just like real hardware with DMA.
 *
 * Copyright 2025 Kate
 * This file is part of simavr.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include "uart_dma.h"
#include "avr_uart.h"
#include "sim_avr.h"

// External callback for UART output (defined in avr_uart.c)
extern void em_uart_output_callback(struct avr_irq_t * irq, uint32_t value, void * param);

// Forward declaration
static void* uart_dma_thread_func(void *param);

/*
 * Initialize FIFO
 */
void uart_dma_fifo_init(uart_dma_fifo_t *fifo) {
	memset(fifo->buffer, 0, UART_DMA_FIFO_SIZE);
	fifo->read_idx = 0;
	fifo->write_idx = 0;
	fifo->count = 0;
	pthread_mutex_init(&fifo->mutex, NULL);
	pthread_cond_init(&fifo->has_data, NULL);
	pthread_cond_init(&fifo->has_space, NULL);
}

/*
 * Cleanup FIFO resources
 */
void uart_dma_fifo_cleanup(uart_dma_fifo_t *fifo) {
	pthread_mutex_destroy(&fifo->mutex);
	pthread_cond_destroy(&fifo->has_data);
	pthread_cond_destroy(&fifo->has_space);
}

/*
 * Push byte to FIFO (called from main thread)
 * Returns: 0 on success, -1 if FIFO is full
 */
int uart_dma_fifo_push(uart_dma_fifo_t *fifo, uint8_t byte) {
	pthread_mutex_lock(&fifo->mutex);
	
	if (fifo->count >= UART_DMA_FIFO_SIZE) {
		// FIFO full
		pthread_mutex_unlock(&fifo->mutex);
		return -1;
	}
	
	fifo->buffer[fifo->write_idx] = byte;
	fifo->write_idx = (fifo->write_idx + 1) % UART_DMA_FIFO_SIZE;
	fifo->count++;
	
	// DEBUG logging disabled for production
	// static int push_count = 0;
	// push_count++;
	// if (push_count <= 100 || byte == '\n') {
	// 	char display = (byte >= 32 && byte <= 126) ? byte : '.';
	// 	fprintf(stderr, "🟢 FIFO PUSH [%d]: 0x%02X '%c' (count: %u)\n", 
	// 	        push_count, byte, display, fifo->count);
	// }
	
	// Signal DMA thread that data is available
	pthread_cond_signal(&fifo->has_data);
	
	pthread_mutex_unlock(&fifo->mutex);
	return 0;
}

/*
 * Pop byte from FIFO (called from DMA thread)
 * Blocks until data is available or thread should exit
 * Returns: 0 on success, -1 if thread should exit
 */
int uart_dma_fifo_pop(uart_dma_fifo_t *fifo, uint8_t *byte, volatile int *running) {
	pthread_mutex_lock(&fifo->mutex);
	
	// Wait for data if FIFO is empty
	while (fifo->count == 0 && *running) {
		pthread_cond_wait(&fifo->has_data, &fifo->mutex);
	}
	
	// Check if we should exit
	if (!*running) {
		pthread_mutex_unlock(&fifo->mutex);
		return -1;
	}
	
	*byte = fifo->buffer[fifo->read_idx];
	fifo->read_idx = (fifo->read_idx + 1) % UART_DMA_FIFO_SIZE;
	fifo->count--;
	
	// Signal that space is available
	pthread_cond_signal(&fifo->has_space);
	
	pthread_mutex_unlock(&fifo->mutex);
	return 0;
}

/*
 * Check if FIFO has space (non-blocking)
 */
int uart_dma_fifo_has_space(uart_dma_fifo_t *fifo) {
	pthread_mutex_lock(&fifo->mutex);
	int has_space = (fifo->count < UART_DMA_FIFO_SIZE);
	pthread_mutex_unlock(&fifo->mutex);
	return has_space;
}

/*
 * Get FIFO count (non-blocking)
 */
uint32_t uart_dma_fifo_count(uart_dma_fifo_t *fifo) {
	pthread_mutex_lock(&fifo->mutex);
	uint32_t count = fifo->count;
	pthread_mutex_unlock(&fifo->mutex);
	return count;
}

/*
 * DMA thread main function
 * This runs independently and transmits bytes at the configured baud rate
 */
static void* uart_dma_thread_func(void *param) {
	uart_dma_thread_t *dma = (uart_dma_thread_t *)param;
	uint8_t byte;
	
	fprintf(stderr, "🧵 UART%c DMA thread started (baud: %u)\n", 
	        dma->name, dma->baud_rate);
	
	// Only UART0 uses the named pipe for output
	// UART1+ are silent unless explicitly connected to hardware
	// (ATmega328P only has UART0; Mega has UART0-3; Leonardo uses UART1 for USB)
	if (dma->name == '0') {
		// Open named pipe for writing UART data
		// This blocks until reader opens the pipe
		dma->uart_pipe = fopen("/tmp/simavr_uart.pipe", "w");
		if (!dma->uart_pipe) {
			fprintf(stderr, "❌ UART%c DMA: Failed to open UART pipe\n", dma->name);
		} else {
			fprintf(stderr, "✅ UART%c DMA: Pipe opened for writing\n", dma->name);
			// Disable buffering for immediate output
			setbuf(dma->uart_pipe, NULL);
		}
	} else {
		// UART2-3: Silent mode (no output unless hardware connected)
		fprintf(stderr, "ℹ️  UART%c DMA: Running in silent mode (no pipe)\n", dma->name);
		dma->uart_pipe = NULL;
	}
	
	while (dma->running) {
		// Wait for and pop next byte from FIFO
		if (uart_dma_fifo_pop(&dma->tx_fifo, &byte, &dma->running) != 0) {
			// Thread should exit
			fprintf(stderr, "🧵 UART%c DMA: Thread exiting (running=%d)\n", 
			        dma->name, dma->running);
			break;
		}
		
		// Calculate transmission delay based on baud rate
		// Each byte = 10 bits (1 start + 8 data + 1 stop)
		// Time per byte = 10,000,000 / baud_rate microseconds
		uint32_t baud = dma->baud_rate;
		if (baud == 0) baud = 9600; // Fallback
		
		uint32_t delay_usec = 10000000 / baud;
		
		// Output to named pipe (dedicated UART channel)
		if (dma->uart_pipe) {
			// Write to named pipe - clean, no mixing with debug!
			int rc = fputc(byte, dma->uart_pipe);
			if (rc == EOF) {
				// Pipe reader closed (EPIPE) — close our end and
				// continue consuming FIFO bytes silently so the
				// main simulation loop doesn't stall.
				fprintf(stderr, "⚠️  UART%c DMA: Pipe write failed (reader closed), "
				        "closing pipe\n", dma->name);
				fclose(dma->uart_pipe);
				dma->uart_pipe = NULL;
			}
			// Pipe is unbuffered (setbuf NULL), so no fflush needed
		}
		// Note: If uart_pipe is NULL (UART1-3 or closed), bytes are consumed but not output
		
		// DISABLED: Too verbose, floods stderr buffer
		// Log line completion to stderr
		// if (byte == '\n') {
		// 	fprintf(stderr, "🔵 UART%c DMA: Line transmitted (total: %llu bytes)\n",
		// 	        dma->name, (unsigned long long)dma->bytes_transmitted + 1);
		// }
		
		// Update statistics
		dma->bytes_transmitted++;
		
		// Simulate transmission delay (this is the autonomous DMA behavior!)
		// Using usleep for simplicity; could use nanosleep for more precision
		if (delay_usec > 0) {
			usleep(delay_usec);
		}
	}
	
	// Close named pipe if it was opened
	if (dma->uart_pipe) {
		fclose(dma->uart_pipe);
		dma->uart_pipe = NULL;
		fprintf(stderr, "📪 UART%c DMA: Pipe closed\n", dma->name);
	}
	
	fprintf(stderr, "🧵 UART%c DMA thread stopped (transmitted %llu bytes)\n", 
	        dma->name, (unsigned long long)dma->bytes_transmitted);
	
	return NULL;
}

/*
 * Start DMA thread for UART
 */
int uart_dma_thread_start(avr_uart_t *uart, uint32_t baud_rate) {
	if (!uart) return -1;
	
	// Allocate DMA structure
	uart_dma_thread_t *dma = (uart_dma_thread_t *)malloc(sizeof(uart_dma_thread_t));
	if (!dma) {
		fprintf(stderr, "❌ UART%c: Failed to allocate DMA structure\n", uart->name);
		return -1;
	}
	
	// Initialize structure
	memset(dma, 0, sizeof(uart_dma_thread_t));
	dma->baud_rate = baud_rate ? baud_rate : 9600;
	dma->uart = uart;
	dma->avr = uart->io.avr;
	dma->name = uart->name;
	dma->running = 1;
	dma->bytes_transmitted = 0;
	dma->fifo_overflows = 0;
	dma->uart_pipe = NULL;
	
	// Initialize FIFO
	uart_dma_fifo_init(&dma->tx_fifo);
	
	// Create thread
	int ret = pthread_create(&dma->thread, NULL, uart_dma_thread_func, dma);
	if (ret != 0) {
		fprintf(stderr, "❌ UART%c: Failed to create DMA thread: %s\n", 
		        uart->name, strerror(errno));
		uart_dma_fifo_cleanup(&dma->tx_fifo);
		free(dma);
		return -1;
	}
	
	// Store reference in UART structure
	uart->dma_thread = dma;
	
	fprintf(stderr, "✅ UART%c: DMA thread started (baud: %u, FIFO size: %d bytes)\n", 
	        uart->name, dma->baud_rate, UART_DMA_FIFO_SIZE);
	
	return 0;
}

/*
 * Stop DMA thread
 */
void uart_dma_thread_stop(uart_dma_thread_t *dma) {
	if (!dma) return;
	
	fprintf(stderr, "🛑 UART%c: Stopping DMA thread...\n", dma->name);
	
	// Signal thread to stop
	dma->running = 0;
	
	// Wake up thread if it's waiting on condition variable
	pthread_mutex_lock(&dma->tx_fifo.mutex);
	pthread_cond_signal(&dma->tx_fifo.has_data);
	pthread_mutex_unlock(&dma->tx_fifo.mutex);
	
	// Wait for thread to finish
	pthread_join(dma->thread, NULL);
	
	// Print statistics
	fprintf(stderr, "📊 UART%c DMA stats: %llu bytes transmitted, %u FIFO overflows\n",
	        dma->name, 
	        (unsigned long long)dma->bytes_transmitted,
	        dma->fifo_overflows);
	
	// Cleanup resources
	uart_dma_fifo_cleanup(&dma->tx_fifo);
	free(dma);
	
	fprintf(stderr, "✅ UART%c: DMA thread stopped\n", dma->name);
}

/*
 * Update baud rate (thread-safe)
 */
void uart_dma_thread_set_baud(uart_dma_thread_t *dma, uint32_t baud_rate) {
	if (!dma) return;
	
	// Simple atomic write (volatile uint32_t)
	// No mutex needed as we're just writing a single value
	uint32_t old_baud = dma->baud_rate;
	dma->baud_rate = baud_rate ? baud_rate : 9600;
	
	if (old_baud != dma->baud_rate) {
		fprintf(stderr, "⚙️  UART%c: DMA baud rate changed: %u → %u\n",
		        dma->name, old_baud, dma->baud_rate);
	}
}

/*
 * Get DMA statistics
 */
void uart_dma_thread_get_stats(uart_dma_thread_t *dma, 
                               uint64_t *bytes_tx, 
                               uint32_t *overflows) {
	if (!dma) return;
	
	if (bytes_tx) {
		*bytes_tx = dma->bytes_transmitted;
	}
	if (overflows) {
		*overflows = dma->fifo_overflows;
	}
}

