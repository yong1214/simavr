/*
	run_avr.c

	Copyright 2008, 2010 Michel Pollet <buserror@gmail.com>

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

#include <stdlib.h>
#include <stdio.h>
#include <libgen.h>
#include <string.h>
#include <signal.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
#include "sim_avr.h"
#include "sim_elf.h"
#include "sim_core.h"
#include "sim_gdb.h"
#include "sim_hex.h"
#include "sim_vcd_file.h"
#include "sim_time.h"
#include "avr_twi.h"
#include "avr_ioport.h"
#include "avr_uart.h"
#include "uart_dma.h"
#include "i2c_eeprom.h"
#include "ds1338_virt.h"
#include "ssd1306_virt.h"
#include "tmp105_virt.h"
#include "avr_gpio_monitor.h"
#include "avr_gpio_inject.h"
#include "avr_serial_monitor.h"
#include "avr_pwm_monitor.h"
#include "../include/sim_log.h"

#include "sim_core_decl.h"

/* ── UART output echo (sends each UART-TX byte to stderr) ──────────── */
static char uart_line_buf[256];
static int  uart_line_pos = 0;

static void
uart_output_notify(struct avr_irq_t *irq, uint32_t value, void *param)
{
	(void)irq; (void)param;
	char c = (char)(value & 0xFF);
	if (c == '\n' || uart_line_pos >= (int)sizeof(uart_line_buf) - 1) {
		uart_line_buf[uart_line_pos] = '\0';
		fprintf(stderr, "📡 UART TX: %s\n", uart_line_buf);
		fflush(stderr);
		uart_line_pos = 0;
	} else if (c >= ' ') {  /* printable characters */
		uart_line_buf[uart_line_pos++] = c;
	}
	/* silently skip \r and other control characters */
}

// Global virtual devices
static i2c_eeprom_t virtual_eeprom;
static ds1338_virt_t virtual_rtc;
static ssd1306_virt_t virtual_ssd1306;
static tmp105_virt_t virtual_tmp105;

static void
display_usage(
	const char * app)
{
	printf("Usage: %s [...] <firmware>\n", app);
	printf(
	 "       [--help|-h|-?]      Display this usage message and exit\n"
	 "       [--list-cores]      List all supported AVR cores and exit\n"
	 "       [-v]                Raise verbosity level\n"
	 "                           (can be passed more than once)\n"
	 "       [--freq|-f <freq>]  Sets the frequency (in Hz) for an .hex firmware\n"
	 "       [--mcu|-m <device>] Sets the MCU type for an .hex firmware\n"
	 "       [--gdb|-g [<port>]] Listen for gdb connection on <port> "
	 "(default 1234)\n"
#ifdef CONFIG_SIMAVR_TRACE
	 "       [--trace, -t]       Run full scale decoder trace\n"
#else
	 "       [--trace, -t]       Run full scale decoder trace (Disabled)\n"
#endif //CONFIG_SIMAVR_TRACE
	 "       [-ti <vector>]      Add traces for IRQ vector <vector>\n"
	 "       [--input|-i <file>] A VCD file to use as input signals\n"
	 "       [--output|-o <file>] A VCD file to save the traced signals\n"
	 "       [--add-trace|-at    <name=[portpin|irq|trace]@addr/mask>] or \n"
	 "                           <name=[sram8|sram16]@addr>] or \n"
	 "                           <name=ioirq@XXXX/N\n"
	 "                           Add signal to be included in VCD output\n"
	 "       [-ff <.hex file>]   Load next .hex file as flash\n"
	 "       [-ee <.hex file>]   Load next .hex file as eeprom\n"
	 "       <firmware>          A .hex or an ELF file. ELF files are\n"
	 "                           preferred, and can include "
	 "debugging syms\n");
	exit(1);
}

static void
list_cores()
{
	printf( "Supported AVR cores:\n");
	for (int i = 0; avr_kind[i]; i++) {
		printf("       ");
		for (int ti = 0; ti < 4 && avr_kind[i]->names[ti]; ti++)
			printf("%s ", avr_kind[i]->names[ti]);
		printf("\n");
	}
	exit(1);
}

static avr_t * avr = NULL;

static void
sig_int(
		int sign)
{
	printf("signal caught, simavr terminating\n");
	if (avr)
		avr_terminate(avr);
	exit(0);
}

/**
 * Virtual Device Response Injection Functions
 * These allow external virtual devices (in Node.js backend) to inject
 * ACK/NACK/data responses back into SimAVR's I2C peripheral.
 */

void inject_i2c_ack(avr_t *avr, uint8_t addr) {
	if (!avr) return;
	
	// Get TWI peripheral IRQ
	avr_irq_t *twi_input = avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
	
	if (twi_input) {
		// Create ACK message using SimAVR's IRQ format
		uint32_t msg = avr_twi_irq_msg(TWI_COND_ACK, addr, 1);
		
		// Raise IRQ to inject ACK into TWI peripheral
		avr_raise_irq(twi_input, msg);
		
		fprintf(stderr, "✅ I2C_ACK:0x%02X injected\n", addr);
		fflush(stderr);
	} else {
		fprintf(stderr, "❌ Failed to get TWI IRQ for ACK injection\n");
	}
}

void inject_i2c_nack(avr_t *avr) {
	if (!avr) return;
	
	// Get TWI peripheral IRQ
	avr_irq_t *twi_input = avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
	
	if (twi_input) {
		// Create NACK message (ACK bit = 0)
		uint32_t msg = avr_twi_irq_msg(TWI_COND_ACK, 0, 0);
		
		// Raise IRQ to inject NACK
		avr_raise_irq(twi_input, msg);
		
		fprintf(stderr, "❌ I2C_NACK injected\n");
		fflush(stderr);
	}
}

void inject_i2c_data(avr_t *avr, uint8_t data) {
	if (!avr) return;
	
	// Get TWI peripheral IRQ
	avr_irq_t *twi_input = avr_io_getirq(avr, AVR_IOCTL_TWI_GETIRQ(0), TWI_IRQ_INPUT);
	
	if (twi_input) {
		// Create DATA message for I2C read operations
		uint32_t msg = avr_twi_irq_msg(TWI_COND_READ, 0, data);
		
		// Raise IRQ to inject data
		avr_raise_irq(twi_input, msg);
		
		fprintf(stderr, "📥 I2C_DATA:0x%02X injected\n", data);
		fflush(stderr);
	}
}

/**
 * Process a single stdin command
 */
static void process_stdin_command(avr_t *avr, const char *cmd) {
	// Parse and execute command
	if (strncmp(cmd, "ACK:", 4) == 0) {
		uint8_t addr;
		if (sscanf(cmd + 4, "0x%02hhx", &addr) == 1) {
			inject_i2c_ack(avr, addr);
		}
	}
	else if (strncmp(cmd, "DATA:", 5) == 0) {
		uint8_t data;
		if (sscanf(cmd + 5, "0x%02hhx", &data) == 1) {
			inject_i2c_data(avr, data);
		}
	}
	else if (strncmp(cmd, "NACK", 4) == 0) {
		inject_i2c_nack(avr);
	}
}

/**
 * Check stdin for virtual device response commands (non-blocking)
 * Called from main loop - processes all available commands
 * Format: ACK:0xXX, DATA:0xXX, NACK
 */
void check_virtual_device_commands(avr_t *avr) {
	if (!avr) return;
	
	fd_set rfds;
	struct timeval tv = {0, 0}; // Non-blocking
	
	FD_ZERO(&rfds);
	FD_SET(STDIN_FILENO, &rfds);
	
	// Check if data available on stdin
	int ready = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
	
	if (ready > 0) {
		char cmd[256];
		if (fgets(cmd, sizeof(cmd), stdin)) {
			// Remove newline
			cmd[strcspn(cmd, "\n")] = 0;
			process_stdin_command(avr, cmd);
		}
	}
}

/**
 * Synchronous stdin check - busy-wait for immediate response
 * Called immediately after I2C transaction to get ACK/NACK
 * This ensures timing is correct for I2C protocol
 */
void check_virtual_device_commands_sync(avr_t *avr) {
	if (!avr) return;
	
	// Busy-wait loop - check stdin repeatedly for immediate response
	// Virtual devices should respond immediately (within microseconds)
	// We'll check up to 1000 times before giving up
	for (int i = 0; i < 1000; i++) {
		fd_set rfds;
		struct timeval tv = {0, 0}; // Non-blocking
		
		FD_ZERO(&rfds);
		FD_SET(STDIN_FILENO, &rfds);
		
		int ready = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
		
		if (ready > 0) {
			// Process ALL available commands (might be multiple: ACK + DATA)
			while (1) {
				fd_set check_rfds;
				struct timeval check_tv = {0, 0}; // Non-blocking
				FD_ZERO(&check_rfds);
				FD_SET(STDIN_FILENO, &check_rfds);
				
				if (select(STDIN_FILENO + 1, &check_rfds, NULL, NULL, &check_tv) > 0) {
					char cmd[256];
					if (fgets(cmd, sizeof(cmd), stdin)) {
						cmd[strcspn(cmd, "\n")] = 0;
						process_stdin_command(avr, cmd);
					} else {
						return; // No more data
					}
				} else {
					return; // No more data
				}
			}
		}
		
		// Small delay to avoid burning CPU (1 millisecond)
		// This gives the backend time to respond
		usleep(1000);
	}
}

int
main(
		int argc,
		char *argv[])
{
#ifdef CONFIG_SIMAVR_TRACE
	int trace = 0;
#endif //CONFIG_SIMAVR_TRACE
	elf_firmware_t f = {{0}};
	uint32_t f_cpu = 0;
	int gdb = 0;
	int log = LOG_ERROR;
	int port = 1234;
	char name[24] = "";
	uint32_t loadBase = AVR_SEGMENT_OFFSET_FLASH;
	int trace_vectors[8] = {0};
	int trace_vectors_count = 0;
	const char *vcd_input = NULL;

	// SIGPIPE must be ignored so that writing to a pipe whose reader has
	// closed returns EPIPE instead of killing the process.  This is the
	// standard practice for any server/daemon that uses named pipes or
	// sockets.  Without this, any momentary reader disconnect on the UART
	// DMA pipe, GPIO monitor pipe, or serial monitor pipe would crash
	// the entire simulator.
	signal(SIGPIPE, SIG_IGN);

	if (argc == 1)
		display_usage(basename(argv[0]));

	for (int pi = 1; pi < argc; pi++) {
		if (!strcmp(argv[pi], "--list-cores")) {
			list_cores();
		} else if (!strcmp(argv[pi], "-h") || !strcmp(argv[pi], "--help")) {
			display_usage(basename(argv[0]));
		} else if (!strcmp(argv[pi], "-m") || !strcmp(argv[pi], "--mcu")) {
			if (pi < argc-1) {
				snprintf(name, sizeof(name), "%s", argv[++pi]);
				strcpy(f.mmcu, name);
			} else {
				display_usage(basename(argv[0]));
			}
		} else if (!strcmp(argv[pi], "-f") || !strcmp(argv[pi], "--freq")) {
			if (pi < argc-1) {
				f_cpu = atoi(argv[++pi]);
				f.frequency = f_cpu;
			} else {
				display_usage(basename(argv[0]));
			}
		} else if (!strcmp(argv[pi], "-i") || !strcmp(argv[pi], "--input")) {
			if (pi < argc-1)
				vcd_input = argv[++pi];
			else
				display_usage(basename(argv[0]));
		} else if (!strcmp(argv[pi], "-o") ||
				   !strcmp(argv[pi], "--output")) {
			if (pi + 1 >= argc) {
				fprintf(stderr, "%s: missing mandatory argument for %s.\n", argv[0], argv[pi]);
				exit(1);
			}
			snprintf(f.tracename, sizeof(f.tracename), "%s", argv[++pi]);
		} else if (!strcmp(argv[pi], "-t") ||
				   !strcmp(argv[pi], "--trace")) {
#ifdef CONFIG_SIMAVR_TRACE
			trace++;
#else
			fprintf(stderr,
					"%s: tracing option '%s' requires "
					"compilation option CONFIG_SIMAVR_TRACE.\n",
					argv[0], argv[pi]);
#endif //CONFIG_SIMAVR_TRACE
		} else if (!strcmp(argv[pi], "-at") ||
				   !strcmp(argv[pi], "--add-trace")) {
			struct {
				char     kind[64];
				uint8_t  mask;
				uint32_t addr;
				char     name[64];
			}    trace;
			char ioctl[4];
			int  n_args, index, ok = 0;

			if (pi + 1 >= argc) {
				fprintf(stderr, "%s: missing mandatory argument for %s.\n",
						argv[0], argv[pi]);
				exit(1);
			}
			++pi;
			n_args = sscanf(argv[pi], "%63[^=]=%63[^@]@0x%x/0x%hhx",
						   trace.name, trace.kind, &trace.addr, &trace.mask);

			switch (n_args) {
			case 4:
				if (strcmp(trace.kind, "ioirq"))
					ok = 1;
				break;
			case 3:
				if (!strcmp(trace.kind, "sram8") ||
					!strcmp(trace.kind, "sram16")) {
					ok = 1;
				}
				break;
			case 2:
				if (!strcmp(trace.kind, "ioirq") &&
					sscanf(argv[pi], "%63[^=]=%63[^@]@%4c/%d",
						   trace.name, trace.kind, ioctl, &index) == 4) {
					trace.addr =
						AVR_IOCTL_DEF(ioctl[0], ioctl[1], ioctl[2], ioctl[3]);
					trace.mask = index;
					ok = 1;
				}
				break;
			default:
				break;
			}
			if (!ok) {
				--pi;
				fprintf(stderr,
						"%s: format for %s is name=kind@addr</mask>.\n",
						argv[0], argv[pi]);
				exit(1);
			}

			if (!strcmp(trace.kind, "ioirq")) {
				f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_IO_IRQ;
			} else if (!strcmp(trace.kind, "portpin")) {
				f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_PORTPIN;
			} else if (!strcmp(trace.kind, "irq")) {
				f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_IRQ;
			} else if (!strcmp(trace.kind, "trace")) {
				f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_TRACE;
			} else if (!strcmp(trace.kind, "sram8")) {
				f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_SRAM_8;
			} else if (!strcmp(trace.kind, "sram16")) {
				f.trace[f.tracecount].kind = AVR_MMCU_TAG_VCD_SRAM_16;
			} else {
				fprintf(
					stderr,
					"%s: unknown trace kind '%s', not one of 'portpin', 'irq', or 'trace'.\n",
					argv[0],
					trace.kind
				);
				exit(1);
			}
			f.trace[f.tracecount].mask = trace.mask;
			f.trace[f.tracecount].addr = trace.addr;
			strncpy(f.trace[f.tracecount].name, trace.name, sizeof(f.trace[f.tracecount].name));

			printf(
				"Adding %s trace on address 0x%04x, mask 0x%02x ('%s')\n",
				f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_IO_IRQ ? "ioirq"
				: f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_PORTPIN ? "portpin"
				: f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_IRQ     ? "irq"
				: f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_TRACE   ? "trace"
				: f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_SRAM_8  ? "sram8"
				: f.trace[f.tracecount].kind == AVR_MMCU_TAG_VCD_SRAM_16 ? "sram16"
				: "unknown",
				f.trace[f.tracecount].addr,
				f.trace[f.tracecount].mask,
				f.trace[f.tracecount].name
			);

			++f.tracecount;
		} else if (!strcmp(argv[pi], "-ti")) {
			if (pi < argc-1)
				trace_vectors[trace_vectors_count++] = atoi(argv[++pi]);
		} else if (!strcmp(argv[pi], "-g") ||
				   !strcmp(argv[pi], "--gdb")) {
			gdb++;
			if (pi < (argc-2) && argv[pi+1][0] != '-' )
				port = atoi(argv[++pi]);
		} else if (!strcmp(argv[pi], "-v")) {
			log++;
		} else if (!strcmp(argv[pi], "-ee")) {
			loadBase = AVR_SEGMENT_OFFSET_EEPROM;
		} else if (!strcmp(argv[pi], "-ff")) {
			loadBase = AVR_SEGMENT_OFFSET_FLASH;
		} else if (argv[pi][0] != '-') {
			sim_setup_firmware(argv[pi], loadBase, &f, argv[0]);
		}
	}

	// Frequency and MCU type were set early so they can be checked when
	// loading a hex file. Set them again because they can also be set
 	// in an ELF firmware file.

	if (strlen(name))
		strcpy(f.mmcu, name);
	if (f_cpu)
		f.frequency = f_cpu;

	fprintf(stderr, "🔧 RUN_AVR: Creating AVR instance for MCU: %s\n", f.mmcu);
	fflush(stderr);
	avr = avr_make_mcu_by_name(f.mmcu);
	if (!avr) {
		fprintf(stderr, "%s: AVR '%s' not known\n", argv[0], f.mmcu);
		exit(1);
	}
	fprintf(stderr, "🔧 RUN_AVR: Initializing AVR\n");
	fflush(stderr);
	avr_init(avr);
	fprintf(stderr, "✅ RUN_AVR: AVR initialized\n");
	fflush(stderr);
	
	// Initialize logging system
	sim_log_init();
	SIM_LOG_INFO(LOG_CAT_SYSTEM, "Logging system initialized");
	
	// Initialize GPIO/Serial monitors and pipe threads
	const char *gpio_pipe = getenv("SIMAVR_GPIO_PIPE");
	if (!gpio_pipe) gpio_pipe = "/tmp/simavr_gpio.pipe";
	avr_gpio_monitor_init(avr, gpio_pipe);
	avr_gpio_monitor_reset();
	
	const char *gpio_inject_pipe = getenv("SIMAVR_GPIO_INJECT_PIPE");
	if (!gpio_inject_pipe) gpio_inject_pipe = "/tmp/simavr_gpio-inject.pipe";
	avr_gpio_inject_init(avr, gpio_inject_pipe);
	avr_pwm_monitor_init(avr);
	avr_gpio_inject_reset();
	
	const char *serial_pipe = getenv("SIMAVR_SERIAL_PIPE");
	if (!serial_pipe) serial_pipe = "/tmp/serial.pipe";
	avr_serial_monitor_init(avr, serial_pipe);
	fflush(stderr);
	
	// Start Serial monitor pipe thread (will block on fopen("w") until reader opens)
	fprintf(stderr, "🔧 Starting Serial monitor pipe thread...\n");
	fflush(stderr);
	avr_serial_monitor_reset();
	fflush(stderr);
	
	avr->log = (log > LOG_TRACE ? LOG_TRACE : log);
#ifdef CONFIG_SIMAVR_TRACE
	avr->trace = trace;
#endif //CONFIG_SIMAVR_TRACE

	fprintf(stderr, "🔧 RUN_AVR: Loading firmware into AVR\n");
	fflush(stderr);
	avr_load_firmware(avr, &f);
	fprintf(stderr, "✅ RUN_AVR: Firmware loaded\n");
	fflush(stderr);
	if (f.flashbase) {
		fprintf(stderr, "Attempted to load a bootloader at %04x\n", f.flashbase);
		avr->pc = f.flashbase;
	}
	for (int ti = 0; ti < trace_vectors_count; ti++) {
		for (int vi = 0; vi < avr->interrupts.vector_count; vi++)
			if (avr->interrupts.vector[vi]->vector == trace_vectors[ti])
				avr->interrupts.vector[vi]->trace = 1;
	}
	if (vcd_input) {
		static avr_vcd_t input;
		if (avr_vcd_init_input(avr, vcd_input, &input)) {
			fprintf(stderr, "%s: Warning: VCD input file %s failed\n", argv[0], vcd_input);
		}
	}

	// Register UART output IRQ handler to echo UART data to stderr (fallback mode only)
	// In DMA mode, UART_IRQ_OUTPUT is not raised (output goes through DMA pipe).
	// This callback only fires in non-DMA fallback mode.
	avr_irq_t *uart_output_irq = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT);
	if (uart_output_irq) {
		avr_irq_register_notify(uart_output_irq, uart_output_notify, NULL);
	}

	// even if not setup at startup, activate gdb if crashing
	avr->gdb_port = port;
	if (gdb) {
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}

	signal(SIGINT, sig_int);
	signal(SIGTERM, sig_int);

	// Initialize virtual I2C devices using official SimAVR approach
	fprintf(stderr, "🔌 Initializing virtual I2C devices...\n");
	fflush(stderr);
	
	// Virtual I2C EEPROM at address 0x50 (24C256 compatible, 32KB)
	// addr_base = 0xa0 (0x50 << 1, with R/W bit)
	// addr_mask = 0x01 (match both read and write)
	i2c_eeprom_init(avr, &virtual_eeprom, 0xa0, 0x01, NULL, 32768);
	i2c_eeprom_attach(avr, &virtual_eeprom, AVR_IOCTL_TWI_GETIRQ(0));
	virtual_eeprom.verbose = 0; // Disable verbose logging
	fprintf(stderr, "✅ Virtual I2C EEPROM registered at 0x50 (32KB)\n");
	
	// Virtual I2C RTC (DS1338/DS1307) at address 0x68
	// addr = 0xD0 (0x68 << 1, with R/W bit)
	ds1338_virt_init(avr, &virtual_rtc);
	ds1338_virt_attach_twi(&virtual_rtc, AVR_IOCTL_TWI_GETIRQ(0));
	virtual_rtc.verbose = 0; // Disable verbose logging
	fprintf(stderr, "✅ Virtual I2C RTC (DS1338) registered at 0x68\n");
	
	// Virtual I2C SSD1306 OLED Display at address 0x3C (7-bit)
	// 8-bit write address = 0x78 (0x3C << 1)
	ssd1306_virt_init(avr, &virtual_ssd1306);
	ssd1306_virt_attach_twi(&virtual_ssd1306, AVR_IOCTL_TWI_GETIRQ(0));
	virtual_ssd1306.verbose = 0; // Disable verbose logging
	fprintf(stderr, "✅ Virtual I2C SSD1306 OLED registered at 0x3C\n");
	
	// Virtual I2C TMP105 Temperature Sensor at address 0x48 (7-bit)
	// 8-bit write address = 0x90 (0x48 << 1)
	tmp105_virt_init(avr, &virtual_tmp105);
	tmp105_virt_attach_twi(&virtual_tmp105, AVR_IOCTL_TWI_GETIRQ(0));
	virtual_tmp105.verbose = 0; // Disable verbose logging
	fprintf(stderr, "✅ Virtual I2C TMP105 Temperature Sensor registered at 0x48\n");
	
	fprintf(stderr, "✅ Total virtual devices: 4 (EEPROM, RTC, SSD1306, TMP105)\n");
	fflush(stderr);
	
	// Enable real-time mode for better timing behavior
	fprintf(stderr, "⏱️  Enabling real-time execution mode...\n");
	// Set a reasonable cycle limit for real-time chunks (1ms worth of cycles)
	avr->run_cycle_limit = avr_usec_to_cycles(avr, 1000);
	fprintf(stderr, "✅ Real-time mode enabled (cycle limit: %llu cycles/ms)\n", 
	        (unsigned long long)avr->run_cycle_limit);
	fflush(stderr);

	for (;;) {
		int state = avr_run(avr);
		if (state == cpu_Done || state == cpu_Crashed)
			break;
		
		// Check for virtual device response commands from backend
		check_virtual_device_commands(avr);
	}

	// Cleanup DMA threads before terminating AVR
	fprintf(stderr, "🛑 Cleaning up UART DMA threads...\n");
	// Find UART through AVR's IO system
	for (avr_io_t * io = avr->io_port; io; io = io->next) {
		if (strcmp(io->kind, "uart") == 0) {
			avr_uart_t *uart = (avr_uart_t *)io;
			if (uart->dma_thread) {
				uart_dma_thread_stop(uart->dma_thread);
				uart->dma_thread = NULL;
			}
		}
	}

	// Cleanup Serial monitor
	avr_serial_monitor_cleanup();
	
	// Cleanup GPIO monitor
	avr_gpio_monitor_cleanup();
	
	// Cleanup GPIO injector
	avr_gpio_inject_cleanup();
	
	avr_terminate(avr);
}
