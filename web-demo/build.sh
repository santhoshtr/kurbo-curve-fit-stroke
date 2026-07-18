#!/bin/bash

# Build script for WASM
echo "Building WASM module..."

# Clean previous build
rm -rf pkg/

 # Check if wasm-pack is installed, if not install it
if ! command -v wasm-pack &> /dev/null; then
    echo "wasm-pack not found. Installing..."
    curl https://wasm-bindgen.github.io/wasm-pack/installer/init.sh -sSf | sh
    # Source the updated PATH if using rustup
    source "$HOME/.cargo/env"
fi

# Build with wasm-pack
wasm-pack build --target web --out-dir pkg

echo "WASM build complete!"
echo "Files generated in pkg/ directory"
