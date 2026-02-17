/*
 * uart_dma.h
 *
 * DMA-style UART transmission simulation using a separate thread.
 * This makes UART transmission autonomous, just like real hardware with DMA.
 *
 * Copyright 2025 Kate
 * This file is part of simavr.
 */

#ifndef __UART_DMA_H__
#define __UART_DMA_H__

#include <stdint.h>
#include <pthread.h>
#include "sim_avr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Forward declarations
struct avr_uart_t;
struct avr_irq_t;

#define UART_DMA_FIFO_SIZE 256

/*
 * Thread-safe FIFO for UART TX data
 */
typedef struct uart_dma_fifo_t {
	uint8_t buffer[UART_DMA_FIFO_SIZE];
	volatile uint32_t read_idx;
	volatile uint32_t write_idx;
	volatile uint32_t count;
	pthread_mutex_t mutex;
	pthread_cond_t has_data;		// Signaled when data is available
	pthread_cond_t has_space;		// Signaled when space is available
} uart_dma_fifo_t;

/*
 * DMA thread structure
 */
typedef struct uart_dma_thread_t {
	pthread_t thread;
	volatile int running;			// Thread running flag
	uart_dma_fifo_t tx_fifo;		// Transmit FIFO
	volatile uint32_t baud_rate;	// Current baud rate
	avr_t *avr;						// Reference to AVR (for logging)
	struct avr_uart_t *uart;		// Reference to UART peripheral
	char name;						// UART name ('0', '1', etc.)
	
	// Statistics (optional)
	volatile uint64_t bytes_transmitted;
	volatile uint32_t fifo_overflows;
	
	// Named pipe for UART output
	FILE *uart_pipe;
} uart_dma_thread_t;

/*
 * Initialize FIFO
 */
void uart_dma_fifo_init(uart_dma_fifo_t *fifo);

/*
 * Cleanup FIFO resources
 */
void uart_dma_fifo_cleanup(uart_dma_fifo_t *fifo);

/*
 * Push byte to FIFO (called from main thread)
 * Returns: 0 on success, -1 if FIFO is full
 */
int uart_dma_fifo_push(uart_dma_fifo_t *fifo, uint8_t byte);

/*
 * Pop byte from FIFO (called from DMA thread)
 * Blocks until data is available
 * Returns: 0 on success, -1 if thread should exit
 */
int uart_dma_fifo_pop(uart_dma_fifo_t *fifo, uint8_t *byte, volatile int *running);

/*
 * Check if FIFO has space (non-blocking)
 * Returns: 1 if space available, 0 if full
 */
int uart_dma_fifo_has_space(uart_dma_fifo_t *fifo);

/*
 * Get FIFO count (non-blocking)
 */
uint32_t uart_dma_fifo_count(uart_dma_fifo_t *fifo);

/*
 * Start DMA thread for UART
 * Returns: 0 on success, -1 on error
 */
int uart_dma_thread_start(struct avr_uart_t *uart, uint32_t baud_rate);

/*
 * Stop DMA thread (blocks until thread exits)
 */
void uart_dma_thread_stop(uart_dma_thread_t *dma);

/*
 * Update baud rate (thread-safe)
 */
void uart_dma_thread_set_baud(uart_dma_thread_t *dma, uint32_t baud_rate);

/*
 * Get DMA statistics
 */
void uart_dma_thread_get_stats(uart_dma_thread_t *dma, 
                               uint64_t *bytes_tx, 
                               uint32_t *overflows);

#ifdef __cplusplus
};
#endif

#endif /* __UART_DMA_H__ */

