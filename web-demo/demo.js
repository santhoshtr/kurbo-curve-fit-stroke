// Import WASM module
import init, {
  fit_curve,
  curve_to_svg_path,
  WebPoint,
  WebPointType,
  CurveFitterOptions,
  version,
} from "./pkg/chi_web_demo.js";

class CurveFitterDemo {
  constructor() {
    this.canvas = null;
    this.ctx = null;
    this.points = [];
    this.selectedPointIndex = -1;
    this.dragging = false;
    this.dragOffset = { x: 0, y: 0 };

    // Settings
    this.settings = {
      cyclic: false,
      showControlPoints: true,
    };

    // Point types (0 = Smooth, 1 = Corner)
    this.pointTypes = [];

    // Canvas state
    this.canvasRect = null;
  }

  // Convert screen coordinates to Cartesian coordinates
  screenToCartesian(screenX, screenY) {
    // const canvasHeight = this.canvas.getBoundingClientRect().height;
    return {
      x: screenX,
      // y: canvasHeight - screenY, // Flip Y coordinate
      y: screenY, // Flip Y coordinate
    };
  }

  // Convert Cartesian coordinates to screen coordinates
  cartesianToScreen(cartX, cartY) {
    // const canvasHeight = this.canvas.getBoundingClientRect().height;
    return {
      x: cartX,
      // y: canvasHeight - cartY, // Flip Y coordinate
      y: cartY, // Flip Y coordinate
    };
  }

  async init() {
    try {
      // Initialize WASM module
      await init();
      console.log(`Curve Fitter WASM module loaded, version: ${version()}`);

      this.setupCanvas();
      this.setupControls();
      this.setupEventListeners();
      this.loadPreset("s-curve");
      this.updateDisplay();
    } catch (error) {
      console.error("Failed to initialize WASM module:", error);
      this.showError(
        "Failed to load the curve engine. Please refresh the page.",
      );
    }
  }

  setupCanvas() {
    this.canvas = document.getElementById("curveCanvas");
    this.ctx = this.canvas.getContext("2d");

    // Set up high DPI support
    const devicePixelRatio = window.devicePixelRatio || 1;
    const rect = this.canvas.getBoundingClientRect();

    this.canvas.width = rect.width * devicePixelRatio;
    this.canvas.height = rect.height * devicePixelRatio;
    this.canvas.style.width = rect.width + "px";
    this.canvas.style.height = rect.height + "px";

    this.ctx.scale(devicePixelRatio, devicePixelRatio);

    // Update canvas rect for coordinate conversion
    this.updateCanvasRect();
  }

  setupControls() {
    // Checkboxes
    document
      .getElementById("cyclicCheckbox")
      .addEventListener("change", (e) => {
        this.settings.cyclic = e.target.checked;
        this.updateDisplay();
      });

    document
      .getElementById("showControlPoints")
      .addEventListener("change", (e) => {
        this.settings.showControlPoints = e.target.checked;
        this.updateDisplay();
      });

    // Buttons
    document.getElementById("clearPoints").addEventListener("click", () => {
      this.clearPoints();
    });

    document.getElementById("downloadSvg").addEventListener("click", () => {
      this.downloadSvg();
    });

    document.getElementById("copySvg").addEventListener("click", () => {
      this.copySvgToClipboard();
    });

    // Preset buttons
    document.getElementById("loadPreset1").addEventListener("click", () => {
      this.loadPreset("s-curve");
    });

    document.getElementById("loadPreset2").addEventListener("click", () => {
      this.loadPreset("triangle");
    });

    document.getElementById("loadPreset3").addEventListener("click", () => {
      this.loadPreset("square");
    });
  }

  setupEventListeners() {
    // Mouse events
    this.canvas.addEventListener("mousedown", (e) => this.onMouseDown(e));
    this.canvas.addEventListener("mousemove", (e) => this.onMouseMove(e));
    this.canvas.addEventListener("mouseup", (e) => this.onMouseUp(e));
    this.canvas.addEventListener("contextmenu", (e) => this.onContextMenu(e));
    this.canvas.addEventListener("dblclick", (e) => this.onDoubleClick(e));

    // Touch events for mobile
    this.canvas.addEventListener("touchstart", (e) => this.onTouchStart(e));
    this.canvas.addEventListener("touchmove", (e) => this.onTouchMove(e));
    this.canvas.addEventListener("touchend", (e) => this.onTouchEnd(e));

    // Keyboard events
    window.addEventListener("keydown", (e) => this.onKeyDown(e));

    // Window resize
    window.addEventListener("resize", () => {
      this.updateCanvasRect();
      this.updateDisplay();
    });
  }

  updateCanvasRect() {
    this.canvasRect = this.canvas.getBoundingClientRect();
  }

  getMousePos(e) {
    const rect = this.canvasRect;
    const screenPos = {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
    };
    return this.screenToCartesian(screenPos.x, screenPos.y);
  }

  getTouchPos(e) {
    const rect = this.canvasRect;
    const touch = e.touches[0];
    const screenPos = {
      x: touch.clientX - rect.left,
      y: touch.clientY - rect.top,
    };
    return this.screenToCartesian(screenPos.x, screenPos.y);
  }

  onMouseDown(e) {
    const pos = this.getMousePos(e);
    const pointIndex = this.findPointAt(pos.x, pos.y);

    if (pointIndex !== -1) {
      // Select and start dragging existing point
      this.selectedPointIndex = pointIndex;
      this.dragging = true;
      this.dragOffset = {
        x: pos.x - this.points[pointIndex].x,
        y: pos.y - this.points[pointIndex].y,
      };
      this.canvas.style.cursor = "grabbing";
    } else {
      // Deselect when clicking empty space
      this.selectedPointIndex = -1;
    }

    this.updateDisplay();
  }

  onDoubleClick(e) {
    const pos = this.getMousePos(e);
    const pointIndex = this.findPointAt(pos.x, pos.y);

    if (pointIndex !== -1) {
      this.togglePointType(pointIndex);
      this.updateDisplay();
    }
  }

  onKeyDown(e) {
    // Handle Delete/Backspace keys
    if (
      (e.key === "Delete" || e.key === "Backspace") &&
      this.selectedPointIndex !== -1
    ) {
      this.removePoint(this.selectedPointIndex);
      e.preventDefault(); // Prevent browser default behavior
      return;
    }
  }

  onMouseMove(e) {
    const pos = this.getMousePos(e);

    if (this.dragging && this.selectedPointIndex !== -1) {
      // Move the selected point
      this.points[this.selectedPointIndex] = {
        x: pos.x - this.dragOffset.x,
        y: pos.y - this.dragOffset.y,
      };
      this.updateDisplay();
      this.showPointInfo(
        e.clientX - this.canvasRect.left,
        e.clientY - this.canvasRect.top,
        this.selectedPointIndex,
      );
    } else {
      // Update cursor based on hover
      const pointIndex = this.findPointAt(pos.x, pos.y);
      this.canvas.style.cursor = pointIndex !== -1 ? "grab" : "crosshair";

      // Show point info on hover
      if (pointIndex !== -1) {
        this.showPointInfo(
          e.clientX - this.canvasRect.left,
          e.clientY - this.canvasRect.top,
          pointIndex,
        );
      } else {
        this.hidePointInfo();
      }
    }
  }

  onMouseUp(e) {
    this.dragging = false;
    this.canvas.style.cursor = "crosshair";
    this.hidePointInfo();
  }

  onContextMenu(e) {
    e.preventDefault();
    const pos = this.getMousePos(e);
    const pointIndex = this.findPointAt(pos.x, pos.y);

    if (pointIndex === -1) {
      // Right-click on empty space adds a new point
      this.addPointAtPosition(pos.x, pos.y);
    }
  }

  // Touch event handlers
  onTouchStart(e) {
    e.preventDefault();
    const pos = this.getTouchPos(e);
    // Convert back to screen coordinates for the event simulation
    const screenPos = this.cartesianToScreen(pos.x, pos.y);
    this.onMouseDown({
      clientX: screenPos.x + this.canvasRect.left,
      clientY: screenPos.y + this.canvasRect.top,
    });
  }

  onTouchMove(e) {
    e.preventDefault();
    const pos = this.getTouchPos(e);
    // Convert back to screen coordinates for the event simulation
    const screenPos = this.cartesianToScreen(pos.x, pos.y);
    this.onMouseMove({
      clientX: screenPos.x + this.canvasRect.left,
      clientY: screenPos.y + this.canvasRect.top,
    });
  }

  onTouchEnd(e) {
    e.preventDefault();
    this.onMouseUp(e);
  }

  findPointAt(x, y) {
    const threshold = 15;
    for (let i = 0; i < this.points.length; i++) {
      const dx = x - this.points[i].x;
      const dy = y - this.points[i].y;
      if (dx * dx + dy * dy < threshold * threshold) {
        return i;
      }
    }
    return -1;
  }

  addPointAtPosition(x, y) {
    this.points.push({ x, y });
    this.pointTypes.push(0); // Default to Smooth
    this.selectedPointIndex = this.points.length - 1; // Select the new point
    this.updateDisplay();
  }

  removePoint(index) {
    this.points.splice(index, 1);
    this.pointTypes.splice(index, 1);

    if (this.selectedPointIndex === index) {
      this.selectedPointIndex = -1;
    } else if (this.selectedPointIndex > index) {
      this.selectedPointIndex--;
    }

    this.updateDisplay();
  }

  togglePointType(index) {
    if (index >= 0 && index < this.pointTypes.length) {
      // Toggle between Smooth (0) and Corner (1)
      this.pointTypes[index] = this.pointTypes[index] === 0 ? 1 : 0;
    }
  }

  clearPoints() {
    this.points = [];
    this.pointTypes = [];
    this.selectedPointIndex = -1;
    this.updateDisplay();
  }

  loadPreset(name) {
    const presets = {
      "s-curve": {
        points: [
          { x: 100, y: 200 },
          { x: 200, y: 300 },
          { x: 300, y: 100 },
          { x: 400, y: 200 },
        ],
        types: [0, 0, 0, 0], // All smooth
      },
      triangle: {
        points: [
          { x: 100, y: 100 },
          { x: 300, y: 300 },
          { x: 500, y: 100 },
        ],
        types: [1, 1, 1], // All corners
      },
      square: {
        points: [
          { x: 200, y: 200 },
          { x: 400, y: 200 },
          { x: 400, y: 400 },
          { x: 200, y: 400 },
        ],
        types: [1, 1, 1, 1], // All corners
      },
    };

    if (presets[name]) {
      this.points = [...presets[name].points];
      this.pointTypes = [...presets[name].types];
      this.selectedPointIndex = -1;
      this.updateDisplay();
    }
  }

  generateCurve() {
    if (this.points.length < 2) {
      return [];
    }

    try {
      // Convert points to WebPoint objects with their types
      const webPoints = this.points.map((p, i) => {
        const pointType =
          this.pointTypes[i] === 1 ? WebPointType.Corner : WebPointType.Smooth;
        return WebPoint.new_with_type(p.x, p.y, pointType);
      });

      // Create options
      const options = new CurveFitterOptions();
      options.set_cyclic(this.settings.cyclic);

      // Generate curve
      return fit_curve(webPoints, options);
    } catch (error) {
      console.error("Error generating curve:", error);
      return [];
    }
  }

  updateDisplay() {
    this.clearCanvas();

    if (this.points.length >= 2) {
      const segments = this.generateCurve();
      this.drawCurve(segments);

      if (this.settings.showControlPoints) {
        this.drawControlPoints(segments);
      }
    }

    this.drawPoints();
    this.updateSvgOutput();
  }

  clearCanvas() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }

  drawCurve(segments) {
    if (segments.length === 0) return;

    const ctx = this.ctx;

    ctx.strokeStyle = "orangered";
    ctx.lineWidth = 3;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    ctx.beginPath();

    // Convert first point to screen coordinates
    const startScreen = this.cartesianToScreen(
      segments[0].start_x,
      segments[0].start_y,
    );
    ctx.moveTo(startScreen.x, startScreen.y);

    // Draw all curve segments with coordinate conversion
    for (const segment of segments) {
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);
      const endScreen = this.cartesianToScreen(segment.end_x, segment.end_y);

      ctx.bezierCurveTo(
        cp1Screen.x,
        cp1Screen.y,
        cp2Screen.x,
        cp2Screen.y,
        endScreen.x,
        endScreen.y,
      );
    }

    ctx.stroke();
  }

  drawControlPoints(segments) {
    const ctx = this.ctx;

    // Draw control point lines
    ctx.strokeStyle = "#cba6f7";
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);

    for (const segment of segments) {
      // Convert all points to screen coordinates
      const startScreen = this.cartesianToScreen(
        segment.start_x,
        segment.start_y,
      );
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);
      const endScreen = this.cartesianToScreen(segment.end_x, segment.end_y);

      // Line from start to first control point
      ctx.beginPath();
      ctx.moveTo(startScreen.x, startScreen.y);
      ctx.lineTo(cp1Screen.x, cp1Screen.y);
      ctx.stroke();

      // Line from second control point to end
      ctx.beginPath();
      ctx.moveTo(cp2Screen.x, cp2Screen.y);
      ctx.lineTo(endScreen.x, endScreen.y);
      ctx.stroke();
    }

    ctx.setLineDash([]);

    // Draw control points
    ctx.fillStyle = "#cba6f7";

    for (const segment of segments) {
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);

      // First control point
      ctx.beginPath();
      ctx.arc(cp1Screen.x, cp1Screen.y, 4, 0, Math.PI * 2);
      ctx.fill();

      // Second control point
      ctx.beginPath();
      ctx.arc(cp2Screen.x, cp2Screen.y, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  drawPoints() {
    const ctx = this.ctx;

    for (let i = 0; i < this.points.length; i++) {
      const point = this.points[i];
      const pointType = this.pointTypes[i];
      const isSelected = i === this.selectedPointIndex;
      const isCorner = pointType === 1; // 1 = Corner

      // Convert to screen coordinates for drawing
      const screenPos = this.cartesianToScreen(point.x, point.y);

      // Point shape (circle for smooth, rectangle for corner)
      ctx.fillStyle = isSelected ? "#f38ba8" : "#11111b";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;

      ctx.beginPath();
      if (isCorner) {
        // Draw rectangle for corner points
        const size = isSelected ? 8 : 6;
        ctx.rect(screenPos.x - size, screenPos.y - size, size * 2, size * 2);
      } else {
        // Draw circle for smooth points
        const radius = isSelected ? 8 : 6;
        ctx.arc(screenPos.x, screenPos.y, radius, 0, Math.PI * 2);
      }
      ctx.fill();
      ctx.stroke();

      // Point label
      ctx.fillStyle = "Highlight";
      ctx.font = "12px monospace";
      ctx.textAlign = "center";
      ctx.fillText(`${i}`, screenPos.x, screenPos.y - 12);
    }
  }

  showPointInfo(screenX, screenY, pointIndex) {
    const pointInfo = document.getElementById("pointInfo");
    const point = this.points[pointIndex];
    const pointType = this.pointTypes[pointIndex];
    const typeText = pointType === 1 ? "Corner" : "Smooth";

    pointInfo.innerHTML = `Point ${pointIndex}: (${Math.round(point.x)}, ${Math.round(point.y)}) - ${typeText}`;
    pointInfo.style.display = "block";
    pointInfo.style.left = screenX + 10 + "px";
    pointInfo.style.top = screenY - 30 + "px";
  }

  hidePointInfo() {
    const pointInfo = document.getElementById("pointInfo");
    pointInfo.style.display = "none";
  }

  updateSvgOutput() {
    if (this.points.length < 2) {
      document.getElementById("svgOutput").value = "";
      return;
    }

    try {
      const webPoints = this.points.map((p, i) => {
        const pointType =
          this.pointTypes[i] === 1 ? WebPointType.Corner : WebPointType.Smooth;
        return WebPoint.new_with_type(p.x, p.y, pointType);
      });
      const options = new CurveFitterOptions();
      options.set_cyclic(this.settings.cyclic);
      const pathData = curve_to_svg_path(webPoints, options);
      const canvasHeight = this.canvas.getBoundingClientRect().height;

      const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${this.canvas.width} ${canvasHeight}">
  <path d="${pathData}" fill="none" stroke="#667eea" stroke-width="3" stroke-linecap="round"/>
</svg>`;

      document.getElementById("svgOutput").value = svg;
    } catch (error) {
      console.error("Error generating SVG:", error);
    }
  }

  downloadSvg() {
    const svgContent = document.getElementById("svgOutput").value;
    if (!svgContent) return;

    const blob = new Blob([svgContent], { type: "image/svg+xml" });
    const url = URL.createObjectURL(blob);

    const a = document.createElement("a");
    a.href = url;
    a.download = "hobby-curve.svg";
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);

    URL.revokeObjectURL(url);
  }

  async copySvgToClipboard() {
    const svgContent = document.getElementById("svgOutput").value;
    if (!svgContent) return;

    try {
      await navigator.clipboard.writeText(svgContent);

      // Visual feedback
      const btn = document.getElementById("copySvg");
      const originalText = btn.textContent;
      btn.textContent = "Copied!";
      btn.style.background = "#28a745";

      setTimeout(() => {
        btn.textContent = originalText;
        btn.style.background = "";
      }, 2000);
    } catch (error) {
      console.error("Failed to copy to clipboard:", error);
    }
  }

  showError(message) {
    const container = document.querySelector(".container");
    const errorDiv = document.createElement("div");
    errorDiv.style.cssText = `
            color: ActiveText;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 20px;
            text-align: center;
        `;
    errorDiv.textContent = message;
    container.insertBefore(errorDiv, container.firstChild);
  }
}

// Initialize the demo
const demo = new CurveFitterDemo();
window.demo = demo; // Make it global for button onclick handlers

document.addEventListener("DOMContentLoaded", () => {
  demo.init();
});
