#!/bin/bash

# Build script for WASM
echo "Building WASM module..."

# Clean previous build
rm -rf pkg/

# Build with wasm-pack
wasm-pack build --target web --out-dir pkg

echo "WASM build complete!"
echo "Files generated in pkg/ directory"
