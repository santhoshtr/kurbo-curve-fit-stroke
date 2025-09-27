// Import WASM module
import init, {
  hobby_curve,
  hobby_to_svg_path,
  WebPoint,
  HobbyOptions,
  version,
} from "../pkg/hobby.js";

class HobbyCurveDemo {
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
      tension: 1.0,
      showControlPoints: true,
    };

    // Point angles
    this.pointAngles = new Map(); // pointIndex -> {entry?: number, exit?: number}

    // Canvas state
    this.canvasRect = null;
  }

  async init() {
    try {
      // Initialize WASM module
      await init();
      console.log(`Hobby WASM module loaded, version: ${version()}`);

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
    // Tension slider
    const tensionSlider = document.getElementById("tensionSlider");
    const tensionValue = document.getElementById("tensionValue");

    tensionSlider.addEventListener("input", (e) => {
      this.settings.tension = parseFloat(e.target.value);
      tensionValue.textContent = this.settings.tension.toFixed(1);
      this.updateDisplay();
    });

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

    // Touch events for mobile
    this.canvas.addEventListener("touchstart", (e) => this.onTouchStart(e));
    this.canvas.addEventListener("touchmove", (e) => this.onTouchMove(e));
    this.canvas.addEventListener("touchend", (e) => this.onTouchEnd(e));

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
    return {
      x: e.clientX - rect.left,
      y: e.clientY - rect.top,
    };
  }

  getTouchPos(e) {
    const rect = this.canvasRect;
    const touch = e.touches[0];
    return {
      x: touch.clientX - rect.left,
      y: touch.clientY - rect.top,
    };
  }

  onMouseDown(e) {
    const pos = this.getMousePos(e);
    const pointIndex = this.findPointAt(pos.x, pos.y);

    if (pointIndex !== -1) {
      // Start dragging existing point
      this.selectedPointIndex = pointIndex;
      this.dragging = true;
      this.dragOffset = {
        x: pos.x - this.points[pointIndex].x,
        y: pos.y - this.points[pointIndex].y,
      };
      this.canvas.style.cursor = "grabbing";
      this.updateAngleControls();
    } else {
      // Add new point
      this.addPoint(pos.x, pos.y);
      this.selectedPointIndex = this.points.length - 1;
      this.updateAngleControls();
    }

    this.updateDisplay();
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
      this.showPointInfo(pos.x, pos.y, this.selectedPointIndex);
    } else {
      // Update cursor based on hover
      const pointIndex = this.findPointAt(pos.x, pos.y);
      this.canvas.style.cursor = pointIndex !== -1 ? "grab" : "crosshair";

      // Show point info on hover
      if (pointIndex !== -1) {
        this.showPointInfo(pos.x, pos.y, pointIndex);
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

    if (pointIndex !== -1) {
      this.removePoint(pointIndex);
    }
  }

  // Touch event handlers
  onTouchStart(e) {
    e.preventDefault();
    const pos = this.getTouchPos(e);
    this.onMouseDown({
      clientX: pos.x + this.canvasRect.left,
      clientY: pos.y + this.canvasRect.top,
    });
  }

  onTouchMove(e) {
    e.preventDefault();
    const pos = this.getTouchPos(e);
    this.onMouseMove({
      clientX: pos.x + this.canvasRect.left,
      clientY: pos.y + this.canvasRect.top,
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

  addPoint(x, y) {
    this.points.push({ x, y });
    this.updateDisplay();
  }

  removePoint(index) {
    this.points.splice(index, 1);
    this.pointAngles.delete(index);

    // Reindex remaining angles
    const newAngles = new Map();
    for (const [pointIndex, angles] of this.pointAngles.entries()) {
      if (pointIndex > index) {
        newAngles.set(pointIndex - 1, angles);
      } else if (pointIndex < index) {
        newAngles.set(pointIndex, angles);
      }
    }
    this.pointAngles = newAngles;

    if (this.selectedPointIndex === index) {
      this.selectedPointIndex = -1;
    } else if (this.selectedPointIndex > index) {
      this.selectedPointIndex--;
    }

    this.updateAngleControls();
    this.updateDisplay();
  }

  clearPoints() {
    this.points = [];
    this.pointAngles.clear();
    this.selectedPointIndex = -1;
    this.updateAngleControls();
    this.updateDisplay();
  }

  loadPreset(name) {
    const presets = {
      "s-curve": [
        { x: 100, y: 300 },
        { x: 200, y: 200 },
        { x: 300, y: 400 },
        { x: 400, y: 300 },
      ],
      triangle: [
        { x: 0, y: 0 },
        { x: 100, y: 100 },
        { x: 200, y: 0 },
      ],
      square: [
        { x: 200, y: 200 },
        { x: 400, y: 200 },
        { x: 400, y: 400 },
        { x: 200, y: 400 },
      ],
    };

    if (presets[name]) {
      this.points = [...presets[name]];
      this.pointAngles.clear();
      this.selectedPointIndex = -1;
      this.updateAngleControls();
      this.updateDisplay();
    }
  }

  generateCurve() {
    if (this.points.length < 2) {
      return [];
    }

    try {
      // Convert points to WebPoint objects
      const webPoints = this.points.map((p) => new WebPoint(p.x, p.y));

      // Create options
      const options = new HobbyOptions();
      options.set_cyclic(this.settings.cyclic);

      // Set tensions if different from default
      if (this.settings.tension !== 1.0) {
        const tensions = new Array(this.points.length).fill(
          this.settings.tension,
        );
        options.set_tensions(tensions);
      }

      // Set angle constraints
      for (const [pointIndex, angles] of this.pointAngles.entries()) {
        if (angles.entry !== undefined) {
          options.add_entry_angle(pointIndex, angles.entry);
        }
        if (angles.exit !== undefined) {
          options.add_exit_angle(pointIndex, angles.exit);
        }
      }
      console.dir({ webPoints });
      console.dir({ options });
      // Generate curve
      return hobby_curve(webPoints, options);
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

    // Move to start of first segment
    ctx.moveTo(segments[0].start_x, segments[0].start_y);

    // Draw all curve segments
    for (const segment of segments) {
      ctx.bezierCurveTo(
        segment.cp1_x,
        segment.cp1_y,
        segment.cp2_x,
        segment.cp2_y,
        segment.end_x,
        segment.end_y,
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
      // Line from start to first control point
      ctx.beginPath();
      ctx.moveTo(segment.start_x, segment.start_y);
      ctx.lineTo(segment.cp1_x, segment.cp1_y);
      ctx.stroke();

      // Line from second control point to end
      ctx.beginPath();
      ctx.moveTo(segment.cp2_x, segment.cp2_y);
      ctx.lineTo(segment.end_x, segment.end_y);
      ctx.stroke();
    }

    ctx.setLineDash([]);

    // Draw control points
    ctx.fillStyle = "#cba6f7";

    for (const segment of segments) {
      // First control point
      ctx.beginPath();
      ctx.arc(segment.cp1_x, segment.cp1_y, 4, 0, Math.PI * 2);
      ctx.fill();

      // Second control point
      ctx.beginPath();
      ctx.arc(segment.cp2_x, segment.cp2_y, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  drawPoints() {
    const ctx = this.ctx;

    for (let i = 0; i < this.points.length; i++) {
      const point = this.points[i];
      const isSelected = i === this.selectedPointIndex;

      // Point circle
      ctx.fillStyle = isSelected ? "#f38ba8" : "#11111b";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 2;

      ctx.beginPath();
      ctx.arc(point.x, point.y, isSelected ? 8 : 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // Point label
      ctx.fillStyle = "Highlight";
      ctx.font = "12px monospace";
      ctx.textAlign = "center";
      ctx.fillText(`${i}`, point.x, point.y - 12);
    }
  }

  showPointInfo(x, y, pointIndex) {
    const pointInfo = document.getElementById("pointInfo");
    const point = this.points[pointIndex];

    pointInfo.innerHTML = `Point ${pointIndex}: (${Math.round(point.x)}, ${Math.round(point.y)})`;
    pointInfo.style.display = "block";
    pointInfo.style.left = x + 10 + "px";
    pointInfo.style.top = y - 30 + "px";
  }

  hidePointInfo() {
    const pointInfo = document.getElementById("pointInfo");
    pointInfo.style.display = "none";
  }

  updateAngleControls() {
    const container = document.getElementById("angleControls");

    if (this.selectedPointIndex === -1 || this.points.length === 0) {
      container.innerHTML =
        '<p class="help-text">Select a point to set entry/exit angles</p>';
      return;
    }

    const pointIndex = this.selectedPointIndex;
    const angles = this.pointAngles.get(pointIndex) || {};

    container.innerHTML = `
            <p><strong>Point ${pointIndex} Angles</strong></p>
            <div class="angle-control">
                <label>Entry:</label>
                <input type="number" id="entryAngle" value="${angles.entry || 0.0}" 
                       placeholder="auto" min="-360" max="360" step="1">
            </div>
            <div class="angle-control">
                <label>Exit:</label>
                <input type="number" id="exitAngle" value="${angles.exit || 0.0}" 
                       placeholder="auto" min="-360" max="360" step="1">
            </div>
        `;

    // Add event listeners
    document.getElementById("entryAngle").addEventListener("input", (e) => {
      this.setEntryAngle(parseFloat(e.target.value) || undefined);
    });

    document.getElementById("exitAngle").addEventListener("input", (e) => {
      this.setExitAngle(parseFloat(e.target.value) || undefined);
    });
  }

  setEntryAngle(angle) {
    if (this.selectedPointIndex === -1) return;

    const pointIndex = this.selectedPointIndex;
    const angles = this.pointAngles.get(pointIndex) || {};

    if (angle !== undefined && !isNaN(angle)) {
      angles.entry = angle;
    } else {
      delete angles.entry;
    }

    this.pointAngles.set(pointIndex, angles);
    this.updateDisplay();
  }

  setExitAngle(angle) {
    if (this.selectedPointIndex === -1) return;

    const pointIndex = this.selectedPointIndex;
    const angles = this.pointAngles.get(pointIndex) || {};

    if (angle !== undefined && !isNaN(angle)) {
      angles.exit = angle;
    } else {
      delete angles.exit;
    }

    this.pointAngles.set(pointIndex, angles);
    this.updateDisplay();
  }

  clearEntryAngle() {
    this.setEntryAngle(undefined);
    this.updateAngleControls();
  }

  clearExitAngle() {
    this.setExitAngle(undefined);
    this.updateAngleControls();
  }

  updateSvgOutput() {
    if (this.points.length < 2) {
      document.getElementById("svgOutput").value = "";
      return;
    }

    try {
      const webPoints = this.points.map((p) => new WebPoint(p.x, p.y));
      const options = new HobbyOptions();
      options.set_cyclic(this.settings.cyclic);

      if (this.settings.tension !== 1.0) {
        const tensions = new Array(this.points.length).fill(
          this.settings.tension,
        );
        options.set_tensions(tensions);
      }

      for (const [pointIndex, angles] of this.pointAngles.entries()) {
        if (angles.entry !== undefined) {
          options.add_entry_angle(pointIndex, angles.entry);
        }
        if (angles.exit !== undefined) {
          options.add_exit_angle(pointIndex, angles.exit);
        }
      }

      const pathData = hobby_to_svg_path(webPoints, options);

      const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${this.canvas.width} ${this.canvas.height}">
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
const demo = new HobbyCurveDemo();
window.demo = demo; // Make it global for button onclick handlers

document.addEventListener("DOMContentLoaded", () => {
  demo.init();
});
