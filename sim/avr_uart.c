/*
	avr_uart.c

	Handles UART access
	Right now just handle "write" to the serial port at any speed
	and printf to the console when '\n' is written.

	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef NO_COLOR
	#define FONT_GREEN		
	#define FONT_DEFAULT	
#else
	#define FONT_GREEN		"\e[32m"
	#define FONT_DEFAULT	"\e[0m"
#endif

#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include "avr_uart.h"
#include "uart_dma.h"
#include "sim_hex.h"
#include "sim_time.h"
#include "sim_gdb.h"
#include "sim_io.h"
#include "../cores/sim_megax8.h"

// ============================================================================
// UART OUTPUT BUFFER MANAGEMENT - Integrated with SimAVR UART System
// ============================================================================

// Global UART output buffer for capturing transmitted characters
#define UART_OUTPUT_BUFFER_SIZE 4096
static char uart_output_buffer[UART_OUTPUT_BUFFER_SIZE];
static int uart_buffer_position = 0;
static int last_read_position = 0;  // Track what JavaScript has already read

/**
 * UART output callback function
 * Called when a character is transmitted via UART_IRQ_OUTPUT
 * Integrated with SimAVR's UART IRQ system
 * @param irq Pointer to the IRQ that was raised
 * @param value The character value that was transmitted
 * @param param User parameter (not used)
 */
void em_uart_output_callback(struct avr_irq_t * irq, uint32_t value, void * param) {
    // DEBUG: Print to console to verify callback is being called
    // printf("🎯 UART CALLBACK TRIGGERED! IRQ=%p, value=0x%02X, param=%p\n", irq, value, param);
    // printf("🎯 UART CALLBACK: This means SimAVR's UART IRQ system IS working!\n");
    // printf("🎯 UART CALLBACK: Character being transmitted: '%c' (0x%02X)\n", 
    //        (value >= 32 && value <= 126) ? (char)value : '.', value);
    
    // Store character in buffer WITHOUT null terminator between characters
    if (uart_buffer_position < UART_OUTPUT_BUFFER_SIZE - 1) {
        uart_output_buffer[uart_buffer_position++] = (char)value;
        // Don't add null terminator here - let complete strings be detected later
        // printf("🎯 UART CALLBACK: Added char '%c' to buffer, position: %d\n", 
        //        (char)value, uart_buffer_position);
    } else {
        // printf("🎯 UART CALLBACK: Buffer full! Cannot add char '%c'\n", (char)value);
    }
}

/**
 * Get next complete string literal from UART output
 * Returns NULL if no new complete string available
 * This is the ONLY function JavaScript needs to call
 */
// Native build - export function for Node.js
const char* em_get_next_uart_string() {
    printf("🔧 UART: Getting next UART string, buffer position: %d, last read: %d\n", uart_buffer_position, last_read_position);
    
    // Check if we have new data since last read
    if (last_read_position >= uart_buffer_position) {
        printf("🔧 UART: No new data available\n");
        return NULL;  // No new data
    }
    
    // Look for complete string delimiters (newline, carriage return)
    for (int i = last_read_position; i < uart_buffer_position; i++) {
        char c = uart_output_buffer[i];
        
        // Check for string delimiters - return complete strings when found
        if (c == '\n' || c == '\r') {
            // Found complete string delimiter
            // Temporarily null-terminate the string for return
            char original_char = uart_output_buffer[i];
            uart_output_buffer[i] = '\0';
            
            const char* result = &uart_output_buffer[last_read_position];
            last_read_position = i + 1;  // Update read position
            
            // Restore original character
            uart_output_buffer[i] = original_char;
            
            return result;
        }
    }
    
    // COMPLETE STRING OUTPUT: Only return complete strings with delimiters
    // This ensures we get complete Serial.println() output as intended
    if (uart_buffer_position > last_read_position) {
        // Only return if we have a delimiter (complete string)
        char last_char = uart_output_buffer[uart_buffer_position - 1];
        if (last_char == '\n' || last_char == '\r') {
            // Temporarily null-terminate at current position for return
            // SAFETY: Check bounds before accessing buffer
            if (uart_buffer_position < UART_OUTPUT_BUFFER_SIZE) {
                char original_char = uart_output_buffer[uart_buffer_position];
                uart_output_buffer[uart_buffer_position] = '\0';
                
                const char* result = &uart_output_buffer[last_read_position];
                last_read_position = uart_buffer_position;  // Mark all as read
                
                // Restore original character
                uart_output_buffer[uart_buffer_position] = original_char;
                
                return result;
            } else {
                // Buffer is full, return what we have
                uart_output_buffer[UART_OUTPUT_BUFFER_SIZE - 1] = '\0';
                const char* result = &uart_output_buffer[last_read_position];
                last_read_position = uart_buffer_position;
                return result;
            }
        }
    }
    
    return NULL;  // No data available
}

/**
 * Get all pending UART output as single string
 * Returns NULL if no new data
 */
// Native build - export function for Node.js
const char* em_get_all_uart_output() {
    if (last_read_position >= uart_buffer_position) {
        return NULL;  // No new data
    }
    
    // SAFETY: Check bounds before accessing buffer
    if (uart_buffer_position < UART_OUTPUT_BUFFER_SIZE) {
        // Temporarily null-terminate at current position for return
        char original_char = uart_output_buffer[uart_buffer_position];
        uart_output_buffer[uart_buffer_position] = '\0';
        
        const char* result = &uart_output_buffer[last_read_position];
        last_read_position = uart_buffer_position;  // Mark all as read
        
        // Restore original character
        uart_output_buffer[uart_buffer_position] = original_char;
        
        return result;
    } else {
        // Buffer is full, return what we have
        uart_output_buffer[UART_OUTPUT_BUFFER_SIZE - 1] = '\0';
        const char* result = &uart_output_buffer[last_read_position];
        last_read_position = uart_buffer_position;
        return result;
    }
}

/**
 * Clear UART buffer (for reset scenarios)
 */
// Native build - export function for Node.js
void em_clear_uart_buffer() {
    uart_buffer_position = 0;
    last_read_position = 0;
    uart_output_buffer[0] = '\0';
    printf("UART output buffer cleared\n");
}

/**
 * Get buffer statistics (for debugging)
 */
// Native build - export function for Node.js
int em_get_uart_buffer_stats() {
    return uart_buffer_position - last_read_position;  // Pending characters
}

#define TRACE(_w) _w
#ifndef TRACE
#define TRACE(_w)
#endif

DEFINE_FIFO(uint16_t, uart_fifo);

static inline void
avr_uart_clear_interrupt(
		avr_t * avr,
		avr_int_vector_t * vector)
{
	if (!vector->vector)
		return;
	// clear the interrupt flag even it's 'sticky'
	if (avr_regbit_get(avr, vector->raised))
		avr_clear_interrupt_if(avr, vector, 0);
	if (avr_regbit_get(avr, vector->raised))
		avr_regbit_clear(avr, vector->raised);
}

static avr_cycle_count_t
avr_uart_txc_raise(
		struct avr_t * avr,
		avr_cycle_count_t when,
		void * param)
{
	avr_uart_t * p = (avr_uart_t *)param;
	if (p->tx_cnt) {
		// Even if the interrupt is disabled, still raise the TXC flag
		if (p->tx_cnt == 1)
			avr_raise_interrupt(avr, &p->txc);
		p->tx_cnt--;
	}
	if (p->udrc.vector) {// UDRE is disabled in the LIN mode
		if (p->tx_cnt) {
			if (avr_regbit_get(avr, p->udrc.raised)) {
				avr_uart_clear_interrupt(avr, &p->udrc);
			}
		} else {
			if (avr_regbit_get(avr, p->txen)) {
				// Even if the interrupt is disabled, still raise the UDRE flag
				avr_raise_interrupt(avr, &p->udrc);
				if (!avr_regbit_get(avr, p->udrc.enable)) {
					return 0; //polling mode: stop TX pump
				} else // udrc (alias udre) should be rased repeatedly while output buffer is empty
					return when + p->cycles_per_byte;
			} else
				return 0; // transfer disabled: stop TX pump
		}
	}
	if (p->tx_cnt)
		return when + p->cycles_per_byte;
	return 0; // stop TX pump
}

static avr_cycle_count_t
avr_uart_rxc_raise(
		struct avr_t * avr,
		avr_cycle_count_t when,
		void * param)
{
	avr_uart_t * p = (avr_uart_t *)param;
	if (avr_regbit_get(avr, p->rxen)) {
		// rxc should be rased continiosly untill input buffer is empty
		if (!uart_fifo_isempty(&p->input)) {
			if (!avr_regbit_get(avr, p->rxc.raised)) {
				p->rxc_raise_time = when;
				p->rx_cnt = 0;
			}
			avr_raise_interrupt(avr, &p->rxc);
			return when + p->cycles_per_byte;
		}
	}
	return 0;
}

static uint8_t
avr_uart_status_read(
		struct avr_t * avr,
		avr_io_addr_t addr,
		void * param)
{
	avr_uart_t * p = (avr_uart_t *)param;

	if (addr == p->fe.reg) {
		if (!uart_fifo_isempty(&p->input)) {
			uint16_t d = uart_fifo_read_at(&p->input, 0);

			uint8_t st = avr->data[addr];

			st &= ~(p->fe.mask << p->fe.bit);
			if (d & UART_INPUT_FE) {
				st |= p->fe.mask << p->fe.bit;
			}

			avr->data[addr] = st;
		}
	}

	uint8_t v = avr_core_watch_read(avr, addr);

	if (addr == p->rxc.raised.reg) {
		//static uint8_t old = 0xff; if (v != old) printf("UCSRA read %02x\n", v); old = v;
		//
		// if RX is enabled, and there is nothing to read, and
		// the AVR core is reading this register, it's probably
		// to poll the RXC TXC flag and spinloop
		// so here we introduce a usleep to make it a bit lighter
		// on CPU and let data arrive
		//
		uint8_t ri = !avr_regbit_get(avr, p->rxen) || !avr_regbit_get(avr, p->rxc.raised);
		uint8_t ti = !avr_regbit_get(avr, p->txen) || !avr_regbit_get(avr, p->txc.raised);

		if (p->flags & AVR_UART_FLAG_POLL_SLEEP) {

			if (ri && ti)
				usleep(1);
		}
		// if reception is idle and the fifo is empty, tell whomever there is room
		if (avr_regbit_get(avr, p->rxen) && uart_fifo_isempty(&p->input)) {
			avr_raise_irq(p->io.irq + UART_IRQ_OUT_XOFF, 0);
			avr_raise_irq(p->io.irq + UART_IRQ_OUT_XON, 1);
		}
	}

	return v;
}

uint8_t
avr_uart_read(
		struct avr_t * avr,
		avr_io_addr_t addr,
		void * param) // Reverted signature
{
	avr_uart_t * p = (avr_uart_t *)param;

	uint8_t v = 0;

	if (!avr_regbit_get(avr, p->rxen) ||
			!avr_regbit_get(avr, p->rxc.raised) // rxc flag not raised - nothing to read!
			) {
		AVR_LOG(avr, LOG_TRACE, "UART%c: attempt to read empty rx buffer\n", p->name);
		avr->data[addr] = 0;
		// made to trigger potential watchpoints
		avr_core_watch_read(avr, addr);
		//return 0;
		goto avr_uart_read_check;
	}
	if (!uart_fifo_isempty(&p->input)) { // probably redundant check
		v = (uint8_t)uart_fifo_read(&p->input) & 0xFF;
		p->rx_cnt++;
		if ((p->rx_cnt > 1) && // UART actually has 2-character rx buffer
				((avr->cycle-p->rxc_raise_time)/p->rx_cnt < p->cycles_per_byte)) {
			// prevent the firmware from reading input characters with non-realistic high speed
			avr_uart_clear_interrupt(avr, &p->rxc);
			p->rx_cnt = 0;
		}
	} else {
		AVR_LOG(avr, LOG_TRACE, "UART%c: BUG: rxc raised with empty rx buffer\n", p->name);
	}

//	TRACE(printf("UART read %02x %s\n", v, uart_fifo_isempty(&p->input) ? "EMPTY!" : "");)
	avr->data[addr] = v;
	// made to trigger potential watchpoints
	v = avr_core_watch_read(avr, addr);

avr_uart_read_check:
	if (uart_fifo_isempty(&p->input)) {
		avr_cycle_timer_cancel(avr, avr_uart_rxc_raise, p);
		avr_uart_clear_interrupt(avr, &p->rxc);
		avr_raise_irq(p->io.irq + UART_IRQ_OUT_XOFF, 0);
		avr_raise_irq(p->io.irq + UART_IRQ_OUT_XON, 1);
	}
	if (!uart_fifo_isfull(&p->input)) {
		avr_regbit_clear(avr, p->dor);
	}

	return v;
}

static void
avr_uart_baud_write(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param)
{
	avr_uart_t * p = (avr_uart_t *)param;
	avr_core_watch_write(avr, addr, v);
	uint32_t val = avr_regbit_get(avr,p->ubrrl) | (avr_regbit_get(avr,p->ubrrh) << 8);

	const int databits[] = { 5,6,7,8,  /* 'reserved', assume 8 */8,8,8, 9 };
	int db = databits[avr_regbit_get(avr, p->ucsz) | (avr_regbit_get(avr, p->ucsz2) << 2)];
	int sb = 1 + avr_regbit_get(avr, p->usbs);
	int word_size = 1 /* start */ + db /* data bits */ + 1 /* parity */ + sb /* stops */;
	int cycles_per_bit = (val+1)*8;
	if (!avr_regbit_get(avr, p->u2x))
		cycles_per_bit *= 2;
	double baud = ((double)avr->frequency) / cycles_per_bit; // can be less than 1
	p->cycles_per_byte = cycles_per_bit * word_size;

	AVR_LOG(avr, LOG_TRACE, "UART: %c configured to %04x = %.4f bps (x%d), %d data %d stop\n",
			p->name, val, baud, avr_regbit_get(avr, p->u2x)?2:1, db, sb);
	AVR_LOG(avr, LOG_TRACE, "UART: Roughly %d usec per byte\n",
			avr_cycles_to_usec(avr, p->cycles_per_byte));
	
	// Update DMA thread baud rate if running
	if (p->dma_thread) {
		uart_dma_thread_set_baud(p->dma_thread, (uint32_t)baud);
	}
}

static void
avr_uart_udr_write(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param)
{
	avr_uart_t * p = (avr_uart_t *)param;

	// DEBUG logging disabled for production
	// static int udr_write_count = 0;
	// udr_write_count++;
	// if (udr_write_count <= 50) {
	// 	char display = (v >= 32 && v <= 126) ? v : '.';
	// 	fprintf(stderr, "🔴 UDR_WRITE [%d]: 0x%02X '%c'\n", udr_write_count, v, display);
	// }

	// TRACE(printf("UDR%c(%02x) = %02x\n", p->name, addr, v);)

	// The byte to be sent should NOT be written there,
	// the value written could never be read back.
	if (avr->gdb) {
		avr_gdb_handle_watchpoints(avr, addr, AVR_GDB_WATCH_WRITE);
	}

	// Clear UDRC if raised
	if (p->udrc.vector && avr_regbit_get(avr, p->udrc.raised)) {
		avr_uart_clear_interrupt(avr, &p->udrc);
	}

	// Check if TX is enabled
	if (!avr_regbit_get(avr, p->txen)) {
		return;
	}

	// ========================================
	// DMA MODE: Use autonomous DMA thread
	// ========================================
	if (p->dma_thread && p->dma_thread->running) {
		// Push byte to DMA FIFO for autonomous transmission
		if (uart_dma_fifo_push(&p->dma_thread->tx_fifo, v) == 0) {
			// Success - byte queued for DMA transmission
			// Immediately raise UDRE (Data Register Empty) interrupt
			// From AVR's perspective, the byte was "sent" to hardware
			if (p->udrc.vector) {
				avr_raise_interrupt(avr, &p->udrc);
			}
		} else {
			// FIFO full - increment overflow counter
			p->dma_thread->fifo_overflows++;
			AVR_LOG(avr, LOG_WARNING, 
				"UART%c: DMA FIFO full, dropping byte 0x%02X\n", 
				p->name, v);
		}
		// NOTE: Do NOT raise UART_IRQ_OUTPUT in DMA mode.
		// DMA thread handles output via named pipe (/tmp/simavr_uart.pipe).
		// Raising IRQ here adds overhead in the tight UDRE loop and can
		// contribute to interrupt timing issues.
		return;  // ← IMPORTANT: Exit early, don't call fallback!
	}

	// ========================================
	// FALLBACK MODE: Original cycle-timer method
	// ========================================
	fprintf(stderr, "⚠️  UART%c: Using FALLBACK mode (DMA not available)\n", p->name);
	avr_raise_irq(p->io.irq + UART_IRQ_OUTPUT, v);
	
	p->tx_cnt++;
	if (p->tx_cnt > 2) { // AVR has 1-character UART tx buffer + shift register
		AVR_LOG(avr, LOG_TRACE,
				"UART%c: tx buffer overflow %d\n",
				p->name, (int)p->tx_cnt);
	}
	
	if (avr_cycle_timer_status(avr, avr_uart_txc_raise, p) == 0) {
		avr_cycle_timer_register(avr, p->cycles_per_byte,
				avr_uart_txc_raise, p); // start the tx pump
	}
}


void
avr_uart_write(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param) // Reverted signature
{
	avr_uart_t * p = (avr_uart_t *)param;

	// DEBUG: Log UART register writes from Arduino framework
	// printf("🔧 UART WRITE: %s-%c addr=0x%02X value=0x%02X (Arduino framework configuring UART)\n", 
	//	   __func__, p->name, addr, v);

	uint8_t masked_v = v;
	uint8_t clear_txc = 0;
	uint8_t clear_rxc = 0;

	// exclude these locations from direct write:
	if (p->udrc.raised.reg == addr) {
		masked_v &= ~(p->udrc.raised.mask << p->udrc.raised.bit);
		masked_v |= avr_regbit_get_raw(avr, p->udrc.raised);
	}
	if (p->txc.raised.reg == addr) {
		uint8_t mask = p->txc.raised.mask << p->txc.raised.bit;
		masked_v &= ~(mask);
		masked_v |= avr_regbit_get_raw(avr, p->txc.raised);
		// it can be cleared by writing a one to its bit location
		if (v & mask)
			clear_txc = 1;
	}
	if (p->rxc.raised.reg == addr) {
		uint8_t mask = p->rxc.raised.mask << p->rxc.raised.bit;
		masked_v &= ~(mask);
		masked_v |= avr_regbit_get_raw(avr, p->rxc.raised);
		if (!p->udrc.vector) {
			// In the LIN mode it can be cleared by writing a one to its bit location
			if (v & mask)
				clear_rxc = 1;
		}
	}
	// mainly to prevent application to confuse itself
	// by writing something there and reading it back:
	if (p->fe.reg == addr) {
		masked_v &= ~(p->fe.mask << p->fe.bit);
		masked_v |= avr_regbit_get_raw(avr, p->fe);
	}
	if (p->dor.reg == addr) {
		masked_v &= ~(p->dor.mask << p->dor.bit);
		//masked_v |= avr_regbit_get_raw(avr, p->dor);
	}
	if (p->upe.reg == addr) {
		masked_v &= ~(p->upe.mask << p->upe.bit);
		masked_v |= avr_regbit_get_raw(avr, p->upe);
	}
	if (p->rxb8.reg == addr) {
		masked_v &= ~(p->rxb8.mask << p->rxb8.bit);
		masked_v |= avr_regbit_get_raw(avr, p->rxb8);
	}

	uint8_t txen = avr_regbit_get(avr, p->txen);
	uint8_t rxen = avr_regbit_get(avr, p->rxen);
	uint8_t udrce = avr_regbit_get(avr, p->udrc.enable);
	// Now write whatever bits could be writen directly.
	// It is necessary anyway, to trigger potential watchpoints.
	avr_core_watch_write(avr, addr, masked_v);
	
	// DEBUG: Log UART register writes to verify storage
	// if (addr == 0xC0 || addr == 0xC1 || addr == 0xC6) {
	//	printf("🔍 UART WRITE: addr=0x%02X masked_v=0x%02X -> avr->data[0x%02X]=0x%02X\n", 
	//		   addr, masked_v, addr, avr->data[addr]);
	// }
	
	uint8_t new_txen = avr_regbit_get(avr, p->txen);
	uint8_t new_rxen = avr_regbit_get(avr, p->rxen);
	uint8_t new_udrce = avr_regbit_get(avr, p->udrc.enable);
	if (p->udrc.vector && (!udrce && new_udrce) && new_txen) {
		// If enabling the UDRC (alias is UDRE) interrupt, raise it immediately if FIFO is empty.
		// If the FIFO is not empty (clear timer is flying) we don't
		// need to raise the interrupt, it will happen when the timer
		// is fired.
		if (avr_cycle_timer_status(avr, avr_uart_txc_raise, p) == 0)
			avr_raise_interrupt(avr, &p->udrc);
	}
	if (clear_txc)
		avr_uart_clear_interrupt(avr, &p->txc);
	if (clear_rxc)
		avr_uart_clear_interrupt(avr, &p->rxc);

	///TODO: handle the RxD & TxD pins function override

	if (new_rxen != rxen) {
		if (new_rxen) {
			if (uart_fifo_isempty(&p->input)) {
				// if reception is enabled and the fifo is empty, tell whomever there is room
				avr_raise_irq(p->io.irq + UART_IRQ_OUT_XOFF, 0);
				avr_raise_irq(p->io.irq + UART_IRQ_OUT_XON, 1);
			}
		} else {
			avr_raise_irq(p->io.irq + UART_IRQ_OUT_XOFF, 1);
			avr_cycle_timer_cancel(avr, avr_uart_rxc_raise, p);
			// flush the Receive Buffer
			uart_fifo_reset(&p->input);
			// clear the rxc interrupt flag
			avr_uart_clear_interrupt(avr, &p->rxc);
		}
	}
	if (new_txen != txen) {
		if (p->udrc.vector && !new_txen) {
			avr_uart_clear_interrupt(avr, &p->udrc);
		} else {
			avr_regbit_set(avr, p->udrc.raised);
		}
	}
	
	// Check if this is a UDR write and call the UDR write handler
	if (addr == p->r_udr) {
		avr_uart_udr_write(avr, addr, v, p);
	}
}

static void
avr_uart_irq_input(
		struct avr_irq_t * irq,
		uint32_t value,
		void * param)
{
	avr_uart_t * p = (avr_uart_t *)param;
	avr_t * avr = p->io.avr;

	// check to see if receiver is enabled
	if (!avr_regbit_get(avr, p->rxen))
		return;

	// reserved/not implemented:
	//avr_regbit_clear(avr, p->fe);
	//avr_regbit_clear(avr, p->upe);
	//avr_regbit_clear(avr, p->rxb8);

	if (uart_fifo_isempty(&p->input) &&
			(avr_cycle_timer_status(avr, avr_uart_rxc_raise, p) == 0)
			) {
		avr_cycle_timer_register(avr, p->cycles_per_byte, avr_uart_rxc_raise, p); // start the rx pump
		p->rx_cnt = 0;
		avr_regbit_clear(avr, p->dor);
	} else if (uart_fifo_isfull(&p->input)) {
		avr_regbit_setto(avr, p->dor, 1);
	}
	if (!avr_regbit_get(avr, p->dor)) { // otherwise newly received character must be rejected
		uart_fifo_write(&p->input, value); // add to fifo
	} else {
		AVR_LOG(avr, LOG_ERROR, "UART%c: %s: RX buffer overrun, lost char=%c=0x%02X\n", p->name, __func__,
				(char)value, (uint8_t)value );
	}

	TRACE(printf("UART IRQ in %02x (%d/%d) %s\n", value, p->input.read, p->input.write, uart_fifo_isfull(&p->input) ? "FULL!!" : "");)

	if (uart_fifo_isfull(&p->input))
		avr_raise_irq(p->io.irq + UART_IRQ_OUT_XOFF, 1);
}


void
avr_uart_reset(
		struct avr_io_t *io)
{
	avr_uart_t * p = (avr_uart_t *)io;
	avr_t * avr = p->io.avr;
	
	fprintf(stderr, "🔧 UART%c: Reset called\n", p->name);
	
	if (p->udrc.vector) {
		avr_regbit_set(avr, p->udrc.raised);
		avr_regbit_clear(avr, p->dor);
	}
	avr_uart_clear_interrupt(avr, &p->txc);
	avr_uart_clear_interrupt(avr, &p->rxc);
	avr_irq_register_notify(p->io.irq + UART_IRQ_INPUT, avr_uart_irq_input, p);
	
	// NOTE: Don't register UART_IRQ_OUTPUT callback when using DMA mode
	// DMA thread outputs directly via printf(), not via IRQ callback
	// avr_irq_register_notify(p->io.irq + UART_IRQ_OUTPUT, em_uart_output_callback, NULL);
	
	avr_cycle_timer_cancel(avr, avr_uart_rxc_raise, p);
	avr_cycle_timer_cancel(avr, avr_uart_txc_raise, p);
	uart_fifo_reset(&p->input);
	p->tx_cnt =  0;

	avr_regbit_set(avr, p->ucsz);
	avr_regbit_clear(avr, p->ucsz2);

	// DEBUG allow printf without fiddling with enabling the uart
	avr_regbit_set(avr, p->txen);
	p->cycles_per_byte = avr_usec_to_cycles(avr, 100);
	
	// Start DMA thread for autonomous UART transmission
	// Calculate default baud rate (Arduino default is 9600)
	uint32_t default_baud = 9600;
	if (!p->dma_thread) {
		if (uart_dma_thread_start(p, default_baud) == 0) {
			fprintf(stderr, "✅ UART%c: Reset complete with DMA mode\n", p->name);
		} else {
			fprintf(stderr, "⚠️  UART%c: DMA thread start failed, using fallback mode\n", p->name);
		}
	} else {
		fprintf(stderr, "✅ UART%c: Reset complete (DMA already running)\n", p->name);
	}
}

static int
avr_uart_ioctl(
		struct avr_io_t * port,
		uint32_t ctl,
		void * io_param)
{
	avr_uart_t * p = (avr_uart_t *)port;
	int res = -1;

	if (!io_param)
		return res;

	if (ctl == AVR_IOCTL_UART_SET_FLAGS(p->name)) {
		p->flags = *(uint32_t*)io_param;
		res = 0;
	}
	if (ctl == AVR_IOCTL_UART_GET_FLAGS(p->name)) {
		*(uint32_t*)io_param = p->flags;
		res = 0;
	}

	return res;
}

static const char * irq_names[UART_IRQ_COUNT] = {
	[UART_IRQ_INPUT] = "8<in",
	[UART_IRQ_OUTPUT] = "8>out",
	[UART_IRQ_OUT_XON] = ">xon",
	[UART_IRQ_OUT_XOFF] = ">xoff",
};

static	avr_io_t	_io = {
	.kind = "uart",
	.reset = avr_uart_reset,
	.ioctl = avr_uart_ioctl,
	.irq_names = irq_names,
};

// Internal wrapper for avr_uart_read to match avr_io_read_t signature
static uint8_t _avr_uart_io_read_wrapper(struct avr_t * avr, avr_io_addr_t addr, void * param) {
    avr_uart_t * p = (avr_uart_t *)param;
    uint8_t value = avr_uart_read(avr, addr, p); // Call original with addr and param
    //avr_core_watch_read(avr, addr); // This is handled inside avr_uart_read now
    //avr->data[addr] = value; // This is handled inside avr_uart_read now
    return value;
}

// Internal wrapper for avr_uart_write to match avr_io_write_t signature
static void _avr_uart_io_write_wrapper(struct avr_t * avr, avr_io_addr_t addr, uint8_t v, void * param) {
    avr_uart_t * p = (avr_uart_t *)param;
    avr_uart_write(avr, addr, v, p); // Call original with addr and param
    //avr_core_watch_write(avr, addr, v); // This is handled inside avr_uart_write now
}

// JavaScript callback for UART output
extern void js_uart_output_callback(char c);

// Native build - export function for Node.js
uint8_t em_avr_uart_read(avr_t * avr, char uart_num) {
    // Access UART directly from MCU structure instead of through I/O system
    struct mcu_t * mcu = (struct mcu_t *)avr;
    avr_uart_t * p = &mcu->uart;

    if (!p) {
        AVR_LOG(avr, LOG_ERROR, "UART%c: Could not find UART instance for read (MCU access failed)\n", uart_num);
        return 0;
    }

    // For external calls, we need to provide dummy addr and param to the original function
    // Assuming reading from UDR (p->r_udr) is the main intent for external read.
    return avr_uart_read(avr, p->r_udr, p);
}


// Native build - export function for Node.js
void em_enable_uart_stdio(avr_t * avr) {
    struct mcu_t * mcu = (struct mcu_t *)avr;
    avr_uart_t * p = &mcu->uart;
    if (p) {
        p->flags |= AVR_UART_FLAG_STDIO;
        AVR_LOG(avr, LOG_OUTPUT, "UART stdio capture enabled\n");
    }
}

// Native build - export function for Node.js
void em_avr_uart_write(avr_t * avr, char uart_num, uint8_t value) {
    // Access UART directly from MCU structure instead of through I/O system
    struct mcu_t * mcu = (struct mcu_t *)avr;
    avr_uart_t * p = &mcu->uart;

    if (!p) {
        AVR_LOG(avr, LOG_ERROR, "UART%c: Could not find UART instance for write (MCU access failed)\n", uart_num);
        return;
    }

    // For external calls, we need to provide dummy addr and param to the original function
    // Assuming writing to UDR (p->r_udr) is the main intent for external write.
    avr_uart_write(avr, p->r_udr, value, p);
}

void
avr_uart_init(
		avr_t * avr,
		avr_uart_t * p)
{
	p->io = _io;

//	printf("%s UART%c UDR=%02x\n", __FUNCTION__, p->name, p->r_udr);

	p->flags = AVR_UART_FLAG_POLL_SLEEP|AVR_UART_FLAG_STDIO;
	p->dma_thread = NULL; // Initialize to NULL

	avr_register_io(avr, &p->io);
	avr_register_vector(avr, &p->rxc);
	avr_register_vector(avr, &p->txc);
	avr_register_vector(avr, &p->udrc);

	// allocate this module's IRQ
	avr_io_setirqs(&p->io, AVR_IOCTL_UART_GETIRQ(p->name), UART_IRQ_COUNT, NULL);
	// Only call callbacks when the value change...
	p->io.irq[UART_IRQ_OUT_XOFF].flags |= IRQ_FLAG_FILTERED;

	avr_register_io_write(avr, p->r_udr, _avr_uart_io_write_wrapper, p); // Use wrapper
	avr_register_io_read(avr, p->r_udr, _avr_uart_io_read_wrapper, p); // Use wrapper

	// status bits
	// monitor code that reads the rxc flag, and delay it a bit
	avr_register_io_read(avr, p->rxc.raised.reg, avr_uart_status_read, p);
    if (p->fe.reg) {
        if (p->fe.reg != p->rxc.raised.reg) {
			avr_register_io_read(avr, p->fe.reg, avr_uart_status_read, p);
        }
    }

	if (p->udrc.vector)
		avr_register_io_write(avr, p->udrc.enable.reg, _avr_uart_io_write_wrapper, p); // Use wrapper
	if (p->r_ucsra)
		avr_register_io_write(avr, p->r_ucsra, _avr_uart_io_write_wrapper, p); // Use wrapper
	if (p->ubrrl.reg)
		avr_register_io_write(avr, p->ubrrl.reg, avr_uart_baud_write, p);
	avr_register_io_write(avr, p->rxen.reg, _avr_uart_io_write_wrapper, p); // Use wrapper
}

