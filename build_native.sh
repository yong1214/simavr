#!/bin/bash

# Native SimAVR Build Script
# Builds SimAVR as native library and executable for Node.js backend service

set -e  # Exit on any error

echo "🔧 Building Native SimAVR..."

# Check if we're in the right directory
if [ ! -f "Makefile.native" ]; then
    echo "❌ Error: Makefile.native not found!"
    echo "Please run this script from the simavr-native directory"
    exit 1
fi

# Check if gcc is available
if ! command -v gcc &> /dev/null; then
    echo "❌ Error: gcc not found!"
    echo "Please install gcc: brew install gcc (macOS) or apt-get install gcc (Ubuntu)"
    exit 1
fi

# Clean previous build
echo "🧹 Cleaning previous build..."
make -f Makefile.native clean

# Build native SimAVR
echo "📦 Building native SimAVR library and executable..."
make -f Makefile.native all

# Install headers
echo "📁 Installing headers..."
make -f Makefile.native install-headers

# Test the build
echo "🧪 Testing build..."
make -f Makefile.native test

echo "✅ Native SimAVR build completed!"
echo ""
echo "📁 Generated files:"
echo "   - lib/libsimavr.a (static library)"
echo "   - lib/libsimavr.so (shared library)"
echo "   - bin/simavr (native executable)"
echo "   - include/ (header files)"
echo ""
echo "🚀 Ready for Node.js backend integration!"



