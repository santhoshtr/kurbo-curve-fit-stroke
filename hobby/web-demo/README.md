# Hobby Curve Interactive Demo

A vanilla JavaScript web interface for the Hobby curve library compiled to WebAssembly.

## Features

- **Interactive Point Management**: Click to add points, drag to move them, right-click to remove
- **Real-time Curve Generation**: See smooth curves update as you manipulate points
- **Curve Options**: 
  - Toggle between open and closed curves
  - Adjust global tension
  - Set individual point entry/exit angles
- **Visual Controls**: 
  - Show/hide control points and construction lines
  - Grid overlay for precision
  - Point selection and highlighting
- **Export Options**: 
  - Generate SVG output
  - Download SVG files
  - Copy SVG to clipboard
- **Presets**: Quick-load common curve shapes (S-curve, triangle, square)

## Setup

1. **Build the WASM module first:**
   ```bash
   cd ../  # Go to hobby directory
   chmod +x build-wasm.sh
   ./build-wasm.sh
   ```

2. **Serve the demo:**
   ```bash
   # From the web-demo directory
   python3 -m http.server 8000
   # Or use any other local server
   ```

3. **Open in browser:**
   Navigate to `http://localhost:8000`

## Usage

### Basic Operations
- **Add Point**: Click anywhere on the canvas
- **Move Point**: Drag existing points
- **Remove Point**: Right-click on a point
- **Select Point**: Click on a point to select it for angle adjustments

### Curve Controls
- **Closed Curve**: Check the "Closed curve" option to connect the last point back to the first
- **Tension**: Use the slider to adjust curve tightness (0.1 = loose, 3.0 = tight)
- **Point Angles**: Select a point and set specific entry/exit angles in degrees

### Visual Options
- **Control Points**: Toggle visibility of Bézier control points and construction lines
- **Grid**: Show/hide alignment grid

### Export
- **SVG Output**: Live-generated SVG code appears in the text area
- **Download**: Save SVG as a file
- **Copy**: Copy SVG code to clipboard

## Technical Details

- Built with vanilla JavaScript (no frameworks)
- Uses Canvas 2D API for real-time rendering
- WASM module provides the curve generation engine
- Responsive design works on desktop and mobile
- High-DPI display support

## Browser Compatibility

- Modern browsers with WebAssembly support
- Chrome 57+, Firefox 52+, Safari 11+, Edge 16+
- Mobile browsers supported via touch events

## Files

- `index.html` - Main HTML structure
- `styles.css` - All styling and responsive design
- `demo.js` - Main JavaScript application logic
- `README.md` - This documentation
