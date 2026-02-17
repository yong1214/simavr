/*
	sim_avr.c

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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/time.h>
#include "sim_avr.h"
#include "sim_core.h"
#include "sim_time.h"
#include "sim_gdb.h"
#include "avr_uart.h"
#include "sim_vcd_file.h"
#include "avr/avr_mcu_section.h"
#include "avr_gpio_monitor.h"

#define AVR_KIND_DECL
#include "cores/sim_core_declare.h"

extern avr_kind_t * avr_kind[];

static void
std_logger(
		avr_t * avr,
		const int level,
		const char * format,
		va_list ap);
static avr_logger_p _avr_global_logger = std_logger;

void
avr_global_logger(
		struct avr_t* avr,
		const int level,
		const char * format,
		... )
{
	va_list args;
	va_start(args, format);
	if (_avr_global_logger)
		_avr_global_logger(avr, level, format, args);
	va_end(args);
}

void
avr_global_logger_set(
		avr_logger_p logger)
{
	_avr_global_logger = logger ? logger : std_logger;
}

avr_logger_p
avr_global_logger_get(void)
{
	return _avr_global_logger;
}

uint64_t
avr_get_time_stamp(
		avr_t * avr )
{
	uint64_t stamp;
#ifndef CLOCK_MONOTONIC_RAW
	/* CLOCK_MONOTONIC_RAW isn't portable, here is the POSIX alternative.
	 * Only downside is that it will drift if the system clock changes */
	struct timeval tv;
	gettimeofday(&tv, NULL);
	stamp = (((uint64_t)tv.tv_sec) * 1E9) + (tv.tv_usec * 1000);
#else
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	stamp = (tp.tv_sec * 1E9) + tp.tv_nsec;
#endif
	if (!avr->time_base)
		avr->time_base = stamp;
	return stamp - avr->time_base;
}

int
avr_init(
		avr_t * avr)
{
	printf("🔧 avr_init: ENTER function\n");
	fflush(stdout);
	static const char *names[] = { ">avr.core.bad_opcode", }; // IRQs

	printf("🔧 avr_init: About to allocate flash memory\n");
	printf("🔧 avr_init: avr->flashend = 0x%04X (%u bytes)\n", avr->flashend, avr->flashend);
	printf("🔧 avr_init: malloc size = %u bytes\n", avr->flashend + 4);
	fflush(stdout);
	avr->flash = malloc(avr->flashend + 4);
	printf("🔧 avr_init: Flash allocated at %p\n", (void*)avr->flash);
	fflush(stdout);
	printf("🔧 avr_init: About to memset flash\n");
	fflush(stdout);
	memset(avr->flash, 0xff, avr->flashend + 1);
	printf("🔧 avr_init: Flash memset completed\n");
	fflush(stdout);
	*((uint16_t*)&avr->flash[avr->flashend + 1]) = AVR_OVERFLOW_OPCODE;
	avr->codeend = avr->flashend;
	printf("🔧 avr_init: About to allocate data memory\n");
	fflush(stdout);
	avr->data = malloc(avr->ramend + 1);
	printf("🔧 avr_init: Data allocated at %p\n", (void*)avr->data);
	fflush(stdout);
	memset(avr->data, 0, avr->ramend + 1);
	printf("🔧 avr_init: Data memset completed\n");
	fflush(stdout);
#ifdef CONFIG_SIMAVR_TRACE
	avr->trace_data = calloc(1, sizeof(struct avr_trace_data_t));
        avr->trace_data->data_names_size = avr->ioend + 1;
#endif
	avr->data_names = calloc(avr->ioend + 1, sizeof (char *));
	/* put "something" in the serial number */
#ifdef _WIN32
	uint32_t r = getpid() + (uint32_t) rand();
#else
	uint32_t r = getpid() + random();
#endif
	for (int i = 0; i < ARRAY_SIZE(avr->serial); i++)
		avr->serial[i] = r >> (i * 3);
	AVR_LOG(avr, LOG_TRACE, "%s init\n", avr->mmcu);
	AVR_LOG(avr, LOG_TRACE, "   serial %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			avr->serial[0], avr->serial[1], avr->serial[2], avr->serial[3],
			avr->serial[4], avr->serial[5], avr->serial[6], avr->serial[7],
			avr->serial[8]);

	// cpu is in limbo before init is finished.
	avr->state = cpu_Limbo;
	avr->frequency = 1000000;	// can be overridden via avr_mcu_section
	avr->irq_pool.avr = avr;
	avr->irq = avr_alloc_irq(&avr->irq_pool, 0, AVR_CORE_IRQ_COUNT, names);
	avr_cmd_init(avr);
	avr_interrupt_init(avr);
	if (avr->custom.init)
		avr->custom.init(avr, avr->custom.data);
	if (avr->init)
		avr->init(avr);
	// set default (non gdb) fast callbacks
	avr->run = avr_callback_run_raw;
	avr->sleep = avr_callback_sleep_raw;
	// number of address bytes to push/pull on/off the stack
	avr->address_size = avr->eind ? 3 : 2;
	avr->log = LOG_ERROR;
	avr_reset(avr);
	avr_regbit_set(avr, avr->reset_flags.porf);		// by  default set to power-on reset
	return 0;
}

void
avr_terminate(
		avr_t * avr)
{
	if (avr->custom.deinit)
		avr->custom.deinit(avr, avr->custom.data);
	if (avr->gdb) {
		avr_deinit_gdb(avr);
		avr->gdb = NULL;
	}
	if (avr->vcd) {
		avr_vcd_close(avr->vcd);
		avr->vcd = NULL;
	}
	avr_deallocate_ios(avr);

	if (avr->flash) free(avr->flash);
	if (avr->data) free(avr->data);
	if (avr->io_console_buffer.buf) {
		avr->io_console_buffer.len = 0;
		avr->io_console_buffer.size = 0;
		free(avr->io_console_buffer.buf);
		avr->io_console_buffer.buf = NULL;
	}
	avr->flash = avr->data = NULL;
}

void
avr_reset(
		avr_t * avr)
{
	printf("🔧 avr_reset: ENTER function\n");
	fflush(stdout);
	AVR_LOG(avr, LOG_TRACE, "%s reset\n", avr->mmcu);

	printf("🔧 avr_reset: Setting state\n");
	fflush(stdout);
	avr->resetting = 1;
	avr->state = cpu_Running;
	
	printf("🔧 POWER-ON RESET: Initializing AVR core\n");
	
	// Clear all I/O registers
	for(int i = 0x20; i <= avr->ioend; i++)
		avr->data[i] = 0;
	
	// Set up stack pointer and program counter
	_avr_sp_set(avr, avr->ramend);
	avr->pc = avr->reset_pc;	// Likely to be zero
	
	// Clear status register
	for (int i = 0; i < 8; i++)
		avr->sreg[i] = 0;
	
	// Reset interrupt system
	avr_interrupt_reset(avr);
	
	// Reset cycle timer
	avr_cycle_timer_reset(avr);
	
	printf("✅ POWER-ON RESET: Core initialization complete\n");
	if (avr->reset)
		avr->reset(avr);
	avr_io_t * port = avr->io_port;
	while (port) {
		printf("🔧 RESET: Calling reset for I/O port: %s\n", port->kind);
		if (port->reset)
			port->reset(port);
		port = port->next;
	}
	
        // Set up UART stdout redirection for native backend
        // TEMPORARILY DISABLED FOR DEBUGGING
        // avr_set_uart_stdout(avr);
	
	// Bootloader management: Force reset vector to 0x0000 to prevent bootloader crash
	printf("🔧 BOOTLOADER: Forcing reset vector to 0x0000 to prevent bootloader crash\n");
	avr->reset_pc = 0x0000;
	avr->pc = 0x0000;
	
	// Fix stack pointer to prevent stack corruption
	printf("🔧 STACK: Fixing stack pointer initialization\n");
	uint16_t safe_sp = avr->ramend - 32; // Leave 32 bytes headroom
	avr->data[0x5D] = safe_sp & 0xFF;        // SPL
	avr->data[0x5E] = (safe_sp >> 8) & 0xFF; // SPH
	printf("✅ STACK: Stack pointer set to 0x%04X (RAMEND: 0x%04X)\n", safe_sp, avr->ramend);
	
	// Bootloader management: Set up bootloader timeout
	printf("🔧 BOOTLOADER: Setting up bootloader timeout\n");
	avr->bootloader_timeout = 1000000; // 1M cycles timeout
	avr->bootloader_active = 0;
	
	printf("✅ BOOTLOADER: Bootloader management configured\n");
	
	avr->cycle = 0; // Prevent crash
	avr->resetting = 0;
}

// Native implementation of UART stdout redirection
void
avr_set_uart_stdout(avr_t *avr)
{
	if (!avr) return;
	
	printf("🔧 Setting up UART stdout redirection for native backend\n");
	
	// For native backend, we need to redirect stderr to stdout
	// This ensures UART output goes to stdout instead of stderr
	if (dup2(STDOUT_FILENO, STDERR_FILENO) == -1) {
		perror("Failed to redirect stderr to stdout");
	}
	
	printf("✅ UART stdout redirection configured\n");
}

void
avr_sadly_crashed(
		avr_t *avr,
		uint8_t signal)
{
	AVR_LOG(avr, LOG_ERROR, "%s\n", __FUNCTION__);
	// Diagnostics: log cycle, PC and opcode at crash
	{
		extern uint16_t _avr_flash_read16le(avr_t * avr, avr_flashaddr_t addr);
		uint16_t opcode = 0;
		if (avr) {
			opcode = _avr_flash_read16le(avr, avr->pc);
			AVR_LOG(avr, LOG_ERROR, "CRASH DIAG: cycle=%llu, PC=0x%08x, opcode=0x%04x\n",
					(unsigned long long)avr->cycle, (unsigned)avr->pc, (unsigned)opcode);
		}
	}
	avr->state = cpu_Stopped;
	if (avr->gdb_port) {
		// enable gdb server, and wait
		if (!avr->gdb)
			avr_gdb_init(avr);
	}
	if (!avr->gdb)
		avr->state = cpu_Crashed;
}

// Native build - no Emscripten dependencies
#include <string.h>
#include "sim_hex.h"

void em_avr_set_pc(avr_t *avr, avr_flashaddr_t pc)
{
	if (!avr) return;
	avr->reset_pc = pc;
	avr->pc = pc;
}


void em_avr_disable_boot_reset(avr_t *avr)
{
	if (!avr) return;
	// Force reset vector to 0, ignore any boot section setting
	avr->reset_pc = 0;
	avr->pc = 0;
}


void em_avr_force_reset_vector_zero(avr_t *avr)
{
	if (!avr) return;
	
	avr->reset_pc = 0;
	avr->pc = 0;
	
	// Also force the AVR to running state
	avr->state = cpu_Running;
}


void em_avr_set_uart_stdout(avr_t *avr)
{
	if (!avr) return;
	// Force UART output to use stdout (fd=1) instead of stderr (fd=2)
	// Native build - no file descriptor redirection needed
}


int em_avr_check_memory_corruption(avr_t *avr)
{
	if (!avr) return -1;
	
	int corruption_count = 0;
	
	// Check stack pointer bounds
	uint16_t sp = avr->data[0x5D] | (avr->data[0x5E] << 8); // SPL | SPH
	if (sp < 0x100 || sp > avr->ramend) {
		printf("MEMORY CORRUPTION: Stack pointer 0x%04X out of bounds (0x100-0x%04X)\n", sp, avr->ramend);
		corruption_count++;
	}
	
	// Check PC bounds
	if (avr->pc > avr->flashend) {
		printf("MEMORY CORRUPTION: PC 0x%08X exceeds flash end 0x%08X\n", avr->pc, avr->flashend);
		corruption_count++;
	}
	
	// Check for stack underflow (SP too high)
	if (sp > (avr->ramend - 32)) {
		printf("STACK UNDERFLOW WARNING: SP=0x%04X very close to RAMEND=0x%04X\n", sp, avr->ramend);
	}
	
	// Check for stack overflow (SP too low)  
	if (sp < 0x200) {
		printf("STACK OVERFLOW WARNING: SP=0x%04X dangerously low\n", sp);
		corruption_count++;
	}
	
	// Check critical AVR registers
	uint8_t sreg = avr->data[0x5F]; // SREG
	if (sreg == 0x00 && avr->cycle > 1000) {
		printf("SUSPICIOUS: SREG is 0x00 after %llu cycles\n", (unsigned long long)avr->cycle);
	}
	
	return corruption_count;
}


void em_avr_fix_stack_pointer(avr_t *avr)
{
	if (!avr) return;
	
	// Arduino bootloader sometimes leaves SP uninitialized
	uint16_t sp = avr->data[0x5D] | (avr->data[0x5E] << 8);
	
	// If SP is invalid, initialize it properly
	if (sp == 0x0000 || sp > avr->ramend || sp < 0x100) {
		// Set SP to top of RAM (standard Arduino initialization)
		uint16_t new_sp = avr->ramend;
		avr->data[0x5D] = new_sp & 0xFF;        // SPL
		avr->data[0x5E] = (new_sp >> 8) & 0xFF; // SPH
	}
}


void em_avr_print_debug_state(avr_t *avr)
{
	if (!avr) return;
	
	uint16_t sp = avr->data[0x5D] | (avr->data[0x5E] << 8);
	uint8_t sreg = avr->data[0x5F];
	
	printf("=== AVR DEBUG STATE ===\n");
	printf("PC: 0x%08X (flash end: 0x%08X)\n", avr->pc, avr->flashend);
	printf("SP: 0x%04X (ram: 0x100-0x%04X)\n", sp, avr->ramend);
	printf("SREG: 0x%02X [%c%c%c%c%c%c%c%c]\n", sreg,
		(sreg & 0x80) ? 'I' : '-',  // Global interrupt
		(sreg & 0x40) ? 'T' : '-',  // Bit copy storage
		(sreg & 0x20) ? 'H' : '-',  // Half carry
		(sreg & 0x10) ? 'S' : '-',  // Sign bit
		(sreg & 0x08) ? 'V' : '-',  // Overflow
		(sreg & 0x04) ? 'N' : '-',  // Negative
		(sreg & 0x02) ? 'Z' : '-',  // Zero
		(sreg & 0x01) ? 'C' : '-'   // Carry
	);
	printf("Cycle: %llu\n", (unsigned long long)avr->cycle);
	printf("State: %d\n", avr->state);
	printf("========================\n");
}


void em_avr_set_bootrst_fuse(avr_t *avr, int boot_from_app)
{
	if (!avr) return;
	
	// For ATmega328p, BOOTRST fuse bit 0 controls boot vector:
	// 0 = boot from bootloader (0x8000)
	// 1 = boot from application (0x0000)
	
	// Set the fuse bit to boot from application
	if (boot_from_app) {
		// Set BOOTRST bit to 1 (boot from application)
		avr->fuse[0] |= 0x01;  // Set bit 0
		printf("BOOTRST fuse set: boot from application (0x0000)\n");
	} else {
		// Clear BOOTRST bit to 0 (boot from bootloader)
		avr->fuse[0] &= ~0x01; // Clear bit 0
		printf("BOOTRST fuse set: boot from bootloader (0x8000)\n");
	}
}


void em_avr_set_stack_pointer_safe(avr_t *avr)
{
	if (!avr) return;
	
	// Set stack pointer to a safer value - leave some headroom
	// ATmega328p has RAM from 0x100 to 0x08FF (2048 bytes)
	// Set SP to 0x08E0 to leave 32 bytes headroom
	uint16_t safe_sp = avr->ramend - 32; // Leave 32 bytes headroom
	
	avr->data[0x5D] = safe_sp & 0xFF;        // SPL
	avr->data[0x5E] = (safe_sp >> 8) & 0xFF; // SPH
	
	printf("Stack pointer set to safe value: 0x%04X (RAMEND: 0x%04X)\n", safe_sp, avr->ramend);
}


int em_avr_load_hex_string(avr_t *avr, const char *hex_string)
{
	if (!avr || !hex_string) return -1;
	
	// Temporarily enable TTY output to see debug messages
	
	// Clear flash memory first (Arduino does this)
	memset(avr->flash, 0xFF, avr->flashend + 1);
	
	// Debug flash memory bounds
	printf("FLASH BOUNDS: flashend=0x%04X (%d bytes)\n", avr->flashend, avr->flashend + 1);
	
	// Parse HEX string line by line
	const char *line_start = hex_string;
	uint32_t segment = 0;
	int lines_processed = 0;
	int bytes_loaded = 0;
	uint32_t min_addr = 0xFFFFFFFF;
	uint32_t max_addr = 0;
	
	while (*line_start) {
		// Find start of line (skip whitespace)
		while (*line_start && (*line_start == '\r' || *line_start == '\n' || *line_start == ' ')) {
			line_start++;
		}
		if (!*line_start) break;
		
		// Find end of line
		const char *line_end = line_start;
		while (*line_end && *line_end != '\r' && *line_end != '\n') {
			line_end++;
		}
		
		// Process this line if it starts with ':' and has minimum length
		if (*line_start == ':' && (line_end - line_start) >= 11) {
			// Debug: Print the exact line being processed
			printf("Processing HEX line: %.*s\n", (int)(line_end - line_start), line_start);
			
			// Create a null-terminated copy of just the hex data (without colon)
			char hex_data[256];
			int hex_len = line_end - line_start - 1; // -1 to skip the colon
			if (hex_len >= sizeof(hex_data)) hex_len = sizeof(hex_data) - 1;
			memcpy(hex_data, line_start + 1, hex_len);
			hex_data[hex_len] = '\0'; // Null terminate
			
			uint8_t bline[272];
			int len = read_hex_string(hex_data, bline, sizeof(bline));
			
			if (len >= 5) { // Changed from > 4 to >= 5 for proper minimum record length
				// Verify checksum
				uint8_t chk = 0;
				for (int i = 0; i < len - 1; i++) {
					chk += bline[i];
				}
				chk = 0x100 - chk;
				
				if (chk == bline[len-1]) {
					uint32_t addr = 0;
					
					switch (bline[3]) {
						case 0: // normal data
							addr = segment | (bline[1] << 8) | bline[2];
							if (bline[0] > 0 && addr <= avr->flashend) {
								// Load data into flash
								for (int i = 0; i < bline[0]; i++) {
									if (addr + i <= avr->flashend) {
										avr->flash[addr + i] = bline[4 + i];
										bytes_loaded++;
										if (addr + i < min_addr) min_addr = addr + i;
										if (addr + i > max_addr) max_addr = addr + i;
									}
								}
								// Debug first few successful loads
								if (bytes_loaded < 50) {
									printf("Loaded %d bytes at 0x%04X (total: %d)\n", bline[0], addr, bytes_loaded);
								}
							} else {
								printf("REJECT: len=%d, addr=0x%04X, flashend=0x%04X\n", bline[0], addr, avr->flashend);
							}
							break;
						case 1: // End of file
							segment = 0;
							break;
						case 2: // Extended address 2 bytes
							segment = ((bline[4] << 8) | bline[5]) << 4;
							break;
						case 4: // Extended address 4 bytes
							segment = ((bline[4] << 8) | bline[5]) << 16;
							break;
					}
				} else {
					printf("CHECKSUM FAIL: Line %d, calc=0x%02X, exp=0x%02X\n", lines_processed, chk, bline[len-1]);
				}
			}
			lines_processed++;
		}
		
		line_start = line_end;
	}
	
	// TTY restore disabled for debugging
	
	// Return success if any bytes were loaded, otherwise return the bytes count for debugging
	if (bytes_loaded > 0) {
		return 0; // Success
	} else {
		// Return negative value indicating how many lines were processed for debugging
		return -(lines_processed + 1); // -1 means no lines, -2 means 1 line but no bytes, etc.
	}
}

/* Get a pointer to a core IRQ. */

avr_irq_t *
avr_get_core_irq(
		avr_t * avr,
		int     irq_no)
{
	return avr->irq + irq_no;
}

/* Get a pointer to a memory IRQ. */

avr_irq_t *avr_get_memory_irq(avr_t * avr, uint16_t addr, int is16)
{
	avr_irq_t  *irq;
	int			width;
	char        name[32];
	const char *names[1] = {name};

	if (addr <= 31 || addr > avr->ramend) {
		AVR_LOG(avr, LOG_ERROR,
				"Address %#04x out of range for SRAM trace.\n", addr);
		return NULL;
	}
	if (avr->sram_tracepoint_count >= ARRAY_SIZE(avr->sram_tracepoint)) {
		AVR_LOG(avr, LOG_ERROR, "Too many SRAM traces (limit = %d)\n",
				ARRAY_SIZE(avr->sram_tracepoint));
		return NULL;
	}

	width = is16 ? 16 : 8;
	sprintf(name, ">%dSRAM_tracepoint_%d", width, avr->sram_tracepoint_count);
	irq = avr_alloc_irq(&avr->irq_pool, 0, 1, names);
	if (!irq)
		return NULL;
	avr->sram_tracepoint[avr->sram_tracepoint_count].irq = irq;
	avr->sram_tracepoint[avr->sram_tracepoint_count].width = width;
	avr->sram_tracepoint[avr->sram_tracepoint_count++].addr = addr;
	return irq;
}

void
avr_set_command_register(
		avr_t * avr,
		avr_io_addr_t addr)
{
	avr_cmd_set_register(avr, addr);
}

static void
_avr_io_console_write(
		struct avr_t * avr,
		avr_io_addr_t addr,
		uint8_t v,
		void * param)
{
	if (v == '\r' && avr->io_console_buffer.buf) {
		avr->io_console_buffer.buf[avr->io_console_buffer.len] = 0;
		AVR_LOG(avr, LOG_OUTPUT, "O:" "%s" "" "\n",
			avr->io_console_buffer.buf);
		avr->io_console_buffer.len = 0;
		return;
	}
	if (avr->io_console_buffer.len + 1 >= avr->io_console_buffer.size) {
		avr->io_console_buffer.size += 128;
		avr->io_console_buffer.buf = (char*)realloc(
			avr->io_console_buffer.buf,
			avr->io_console_buffer.size);
	}
	if (v >= ' ')
		avr->io_console_buffer.buf[avr->io_console_buffer.len++] = v;
}

void
avr_set_console_register(
		avr_t * avr,
		avr_io_addr_t addr)
{
	if (addr)
		avr_register_io_write(avr, addr, _avr_io_console_write, NULL);
}

void
avr_loadcode(
		avr_t * avr,
		uint8_t * code,
		uint32_t size,
		avr_flashaddr_t address)
{
	if ((address + size) > avr->flashend+1) {
		AVR_LOG(avr, LOG_ERROR, "avr_loadcode(): Attempted to load code of size %d but flash size is only %d.\n",
			size, avr->flashend + 1);
		abort();
	}
	memcpy(avr->flash + address, code, size);
}

/**
 * Accumulates sleep requests (and returns a sleep time of 0) until
 * a minimum count of requested sleep microseconds are reached
 * (low amounts cannot be handled accurately).
 */
uint32_t
avr_pending_sleep_usec(
		avr_t * avr,
		avr_cycle_count_t howLong)
{
	avr->sleep_usec += avr_cycles_to_usec(avr, howLong);
	uint32_t usec = avr->sleep_usec;
	if (usec > 200) {
		avr->sleep_usec = 0;
		return usec;
	}
	return 0;
}

void
avr_callback_sleep_gdb(
		avr_t * avr,
		avr_cycle_count_t howLong)
{
	uint32_t usec = avr_pending_sleep_usec(avr, howLong);
	while (avr_gdb_processor(avr, usec))
		;
}

void
avr_callback_run_gdb(
		avr_t * avr)
{
	avr_gdb_processor(avr, avr->state == cpu_Stopped ? 50000 : 0);

	if (avr->state == cpu_Stopped)
		return ;

	// if we are stepping one instruction, we "run" for one..
	int step = avr->state == cpu_Step;
	if (step)
		avr->state = cpu_Running;

	avr_flashaddr_t new_pc = avr->pc;

	if (avr->state == cpu_Running) {
		new_pc = avr_run_one(avr);
#if CONFIG_SIMAVR_TRACE
		avr_dump_state(avr);
#endif
	}

	// run the cycle timers, get the suggested sleep time
	// until the next timer is due
	avr_cycle_count_t sleep = avr_cycle_timer_process(avr);

	avr->pc = new_pc;

	if (avr->state == cpu_Sleeping) {
		if (!avr->sreg[S_I]) {
			if (avr->log)
				AVR_LOG(avr, LOG_TRACE, "simavr: sleeping with interrupts off, quitting gracefully\n");
			avr->state = cpu_Done;
			return;
		}
		/*
		 * try to sleep for as long as we can (?)
		 */
		avr->sleep(avr, sleep);
		avr->cycle += 1 + sleep;
	}
	// Interrupt servicing might change the PC too, during 'sleep'
	if (avr->state == cpu_Running || avr->state == cpu_Sleeping)
		avr_service_interrupts(avr);

	// if we were stepping, use this state to inform remote gdb
	if (step)
		avr->state = cpu_StepDone;
}

/*
To avoid simulated time and wall clock time to diverge over time
this function tries to keep them in sync (roughly) by sleeping
for the time required to match the expected sleep deadline
in wall clock time.
*/
void
avr_callback_sleep_raw(
		avr_t *avr,
		avr_cycle_count_t how_long)
{
	/* figure out how long we should wait to match the sleep deadline */
	uint64_t deadline_ns = avr_cycles_to_nsec(avr, avr->cycle + how_long);
	uint64_t runtime_ns = avr_get_time_stamp(avr);
	if (runtime_ns >= deadline_ns)
		return;
	uint64_t sleep_us = (deadline_ns - runtime_ns) / 1000;
	usleep(sleep_us);
	return;
}

void
avr_callback_run_raw(
		avr_t * avr)
{
	avr_flashaddr_t new_pc = avr->pc;

	if (avr->state == cpu_Running) {
		new_pc = avr_run_one(avr);
#if CONFIG_SIMAVR_TRACE
		avr_dump_state(avr);
#endif
	}

	// run the cycle timers, get the suggested sleep time
	// until the next timer is due
	avr_cycle_count_t sleep = avr_cycle_timer_process(avr);

	avr->pc = new_pc;

	if (avr->state == cpu_Sleeping) {
		if (!avr->sreg[S_I]) {
			if (avr->log)
				AVR_LOG(avr, LOG_TRACE, "simavr: sleeping with interrupts off, quitting gracefully\n");
			avr->state = cpu_Done;
			return;
		}
		/*
		 * try to sleep for as long as we can (?)
		 */
		avr->sleep(avr, sleep);
		avr->cycle += 1 + sleep;
	}
	// Interrupt servicing might change the PC too, during 'sleep'
	if (avr->state == cpu_Running || avr->state == cpu_Sleeping) {
		/* Note: checking interrupt_state here is completely superfluous, however
			as interrupt_state tells us all we really need to know, here
			a simple check here may be cheaper than a call not needed. */
		if (avr->interrupt_state)
			avr_service_interrupts(avr);
	}
}


int
avr_run(
		avr_t * avr)
{
	avr->run(avr);
	return avr->state;
}

avr_t *
avr_core_allocate(
		const avr_t * core,
		uint32_t coreLen)
{
	uint8_t * b = malloc(coreLen);
	memcpy(b, core, coreLen);
	return (avr_t *)b;
}

avr_t *
avr_make_mcu_by_name(
		const char *name)
{
	avr_kind_t * maker = NULL;
	for (int i = 0; avr_kind[i] && !maker; i++) {
		for (int j = 0; j < 4 && avr_kind[i]->names[j]; j++)
			if (!strcmp(avr_kind[i]->names[j], name)) {
				maker = avr_kind[i];
				break;
			}
	}
	if (!maker) {
		AVR_LOG(((avr_t*)0), LOG_ERROR, "%s: AVR '%s' not known\n", __FUNCTION__, name);
		return NULL;
	}

	avr_t * avr = maker->make();
	AVR_LOG(avr, LOG_TRACE, "Starting %s - flashend %04x ramend %04x e2end %04x\n",
			avr->mmcu, avr->flashend, avr->ramend, avr->e2end);
	return avr;
}

static void
std_logger(
		avr_t * avr,
		const int level,
		const char * format,
		va_list ap)
{
	if (!avr || avr->log >= level) {
		vfprintf((level < LOG_ERROR) ?  stdout : stderr, format, ap);
	}
}


uint16_t em_avr_read_flash_word(avr_t *avr, uint32_t address) {
    if (!avr || address >= avr->flashend) {
        printf("Invalid address 0x%04X for flash read\n", address);
        return 0xFFFF;
    }
    
    // Read 16-bit word from flash memory
    uint16_t word = avr->flash[address] | (avr->flash[address + 1] << 8);
    printf("Read flash word at 0x%04X: 0x%04X\n", address, word);
    return word;
}



