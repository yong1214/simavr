#!/bin/bash

# SimAVR WebAssembly Compilation Script
# This script compiles SimAVR to WebAssembly with Emscripten
# and exports all necessary functions for JavaScript interoperability

set -e  # Exit on any error

echo "🔧 Compiling SimAVR to WebAssembly with Emscripten..."

# Check if Emscripten is available
if ! command -v emcc &> /dev/null; then
    echo "❌ Error: Emscripten (emcc) not found!"
    echo "Please install Emscripten: https://emscripten.org/docs/getting_started/downloads.html"
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "avr_emscripten_wrappers.c" ]; then
    echo "❌ Error: avr_emscripten_wrappers.c not found!"
    echo "Please run this script from the simavr/simavr directory"
    exit 1
fi

# Create output directory
mkdir -p ../../frontend/public

# Compile with Emscripten
echo "📦 Compiling SimAVR source files..."

emcc -O2 \
    -s WASM=1 \
    -s EXPORTED_FUNCTIONS='[
        "_avr_make_mcu_by_name",
        "_avr_init", 
        "_avr_reset",
        "_avr_run",
        "_avr_register_io_write",
        "_avr_register_io_read",
        "_em_avr_get_pc",
        "_em_avr_set_pc",
        "_em_avr_get_sp",
        "_em_avr_get_cycle",
        "_em_avr_get_state",
        "_em_avr_get_frequency",
        "_em_avr_read_data_byte",
        "_em_avr_write_data_byte",
        "_em_avr_read_flash_byte",
        "_em_avr_write_flash_byte",
        "_em_avr_get_current_instruction",
        "_em_get_next_uart_string",
        "_em_get_all_uart_output",
        "_em_clear_uart_buffer",
        "_em_get_uart_buffer_stats",
        "_malloc",
        "_free"
    ]' \
    -s EXPORTED_RUNTIME_METHODS='[
        "addFunction",
        "removeFunction", 
        "ccall",
        "cwrap",
        "getValue",
        "setValue",
        "stringToUTF8",
        "UTF8ToString"
    ]' \
    -s ALLOW_TABLE_GROWTH=1 \
    -s RESERVED_FUNCTION_POINTERS=10 \
    -s MODULARIZE=1 \
    -s EXPORT_NAME="SimAVR" \
    -s ENVIRONMENT="web" \
    -s NO_EXIT_RUNTIME=1 \
    -s ASSERTIONS=1 \
    -s STACK_OVERFLOW_CHECK=1 \
    -s INITIAL_MEMORY=16777216 \
    -s MAXIMUM_MEMORY=268435456 \
    -s ALLOW_MEMORY_GROWTH=1 \
    -I./cores \
    -I./sim \
    -I. \
    ./avr_kind_emscripten.c \
    ./avr_emscripten_wrappers.c \
    ./sim/sim_avr.c \
    ./sim/sim_core.c \
    ./sim/sim_io.c \
    ./sim/sim_irq.c \
    ./sim/sim_cycle_timers.c \
    ./sim/sim_gdb.c \
    ./sim/sim_hex.c \
    ./sim/sim_interrupts.c \
    ./sim/sim_cmds.c \
    ./sim/sim_vcd_file.c \
    ./sim/sim_utils.c \
    ./sim/avr_eeprom.c \
    ./sim/avr_flash.c \
    ./sim/avr_watchdog.c \
    ./sim/avr_extint.c \
    ./sim/avr_ioport.c \
    ./sim/avr_uart.c \
    ./sim/avr_acomp.c \
    ./sim/avr_adc.c \
    ./sim/avr_timer.c \
    ./sim/avr_spi.c \
    ./sim/avr_twi.c \
    ./cores/sim_mega328.c \
    ./cores/sim_megax8.c \
    -o ../../frontend/public/simavr.js

echo "✅ SimAVR WebAssembly compilation completed!"

# Copy generated files to frontend
echo "📁 Copying generated files to frontend..."
cp ../../frontend/public/simavr.js ../../frontend/public/simavr.js
cp ../../frontend/public/simavr.wasm ../../frontend/public/simavr.wasm

echo "🎉 SimAVR WebAssembly module ready!"
echo "📁 Files generated:"
echo "   - ../../frontend/public/simavr.js"
echo "   - ../../frontend/public/simavr.wasm"
echo ""
echo "🔧 Exported functions:"
echo "   - _em_avr_get_pc, _em_avr_set_pc"
echo "   - _em_avr_get_sp, _em_avr_set_sp" 
echo "   - _em_avr_get_cycle, _em_avr_get_state"
echo "   - _em_avr_get_current_instruction"
echo "   - _em_avr_read_data_byte, _em_avr_write_data_byte"
echo "   - _em_avr_read_flash_byte, _em_avr_write_flash_byte"
echo "   - And many more..."
echo ""
echo "🚀 Ready to test the hardware simulator!"
