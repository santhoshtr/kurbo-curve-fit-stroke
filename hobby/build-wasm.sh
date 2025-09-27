#!/bin/bash
set -e

echo "Building hobby WASM package..."

# Clean previous builds
rm -rf pkg/

# Build with wasm-pack
wasm-pack build --target web --features wasm

echo "WASM build completed successfully!"
echo "Output directory: pkg/"
echo ""
echo "To use in a web page:"
echo "  import init, { hobby_curve, WebPoint } from './pkg/hobby.js';"
echo "  await init();"
