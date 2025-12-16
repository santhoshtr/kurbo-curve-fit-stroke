import init, {
  fit_curve,
  fit_curve_with_stroke,
  curve_to_svg_path_with_stroke,
  WebPoint,
  WebPointType,
  CurveFitterOptions,
  StrokeOptions,
  version,
} from "./pkg/chi_web_demo.js";

// Import Tweakpane
import { Pane } from "https://cdn.jsdelivr.net/npm/tweakpane@4.0.3/dist/tweakpane.min.js";

class CurveFitterDemo {
  constructor() {
    this.canvas = null;
    this.ctx = null;
    this.points = [];
    this.selectedPointIndex = -1;
    this.dragging = false;
    this.dragOffset = { x: 0, y: 0 };

    // Settings object for Tweakpane
    this.params = {
      cyclic: false,
      showControlPoints: true,
      strokeEnabled: true,
      strokeWidth: 20,
      strokeCap: "round",
      strokeJoin: "round",
      pointOffsets: [],
    };

    // Point types (0 = Smooth, 1 = Corner)
    this.pointTypes = [];
    // Canvas state
    this.canvasRect = null;

    // Tweakpane instance
    this.pane = null;
  }

  screenToCartesian(screenX, screenY) {
    return {
      x: screenX,
      y: screenY,
    };
  }

  cartesianToScreen(cartX, cartY) {
    return {
      x: cartX,
      y: cartY,
    };
  }

  async init() {
    try {
      // Initialize WASM module
      await init();
      console.log(`Curve Fitter WASM module loaded, version: ${version()}`);

      this.setupCanvas();
      this.setupTweakpane();
      this.setupEventListeners();
      this.loadPreset("s-curve");
      this.updateDisplay();
    } catch (error) {
      console.error("Failed to initialize WASM module:", error);
      alert("Failed to load the curve engine. Please refresh the page.");
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

    this.updateCanvasRect();
  }

  setupTweakpane() {
    this.pane = new Pane({
      title: "Curve Fitter",
      container: document.getElementById("controls"),
    });

    // Curve options folder
    const curveFolder = this.pane.addFolder({
      title: "Curve Options",
      expanded: true,
    });

    curveFolder
      .addBinding(this.params, "cyclic", {
        label: "Closed curve",
      })
      .on("change", () => this.updateDisplay());

    curveFolder
      .addBinding(this.params, "showControlPoints", {
        label: "Show controls",
      })
      .on("change", () => this.updateDisplay());

    // Stroke options folder
    const strokeFolder = this.pane.addFolder({
      title: "Stroke Options",
      expanded: true,
    });

    strokeFolder
      .addBinding(this.params, "strokeEnabled", {
        label: "Enable stroke",
      })
      .on("change", () => this.updateDisplay());

    strokeFolder
      .addBinding(this.params, "strokeWidth", {
        label: "Width",
        min: 1,
        max: 50,
        step: 0.5,
      })
      .on("change", () => this.updateDisplay());

    strokeFolder
      .addBinding(this.params, "strokeCap", {
        label: "Cap",
        options: {
          Round: "round",
          Butt: "butt",
          Square: "square",
        },
      })
      .on("change", () => this.updateDisplay());

    strokeFolder
      .addBinding(this.params, "strokeJoin", {
        label: "Join",
        options: {
          Round: "round",
          Bevel: "bevel",
          Miter: "miter",
        },
      })
      .on("change", () => this.updateDisplay());

    // Presets folder
    const presetsFolder = this.pane.addFolder({
      title: "Presets",
      expanded: false,
    });

    presetsFolder.addButton({ title: "S-Curve" }).on("click", () => {
      this.loadPreset("s-curve");
    });

    presetsFolder.addButton({ title: "Triangle" }).on("click", () => {
      this.loadPreset("triangle");
    });

    presetsFolder.addButton({ title: "Square" }).on("click", () => {
      this.loadPreset("square");
    });
    presetsFolder.addBlade({
      view: "separator",
    });
    presetsFolder.addButton({ title: "Clear All" }).on("click", () => {
      this.clearPoints();
    });

    // Export folder
    const exportFolder = this.pane.addFolder({
      title: "Export",
      expanded: false,
    });

    exportFolder.addButton({ title: "Copy SVG" }).on("click", () => {
      this.copySvgToClipboard();
    });

    exportFolder.addButton({ title: "Download SVG" }).on("click", () => {
      this.downloadSvg();
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
      this.selectedPointIndex = pointIndex;
      this.dragging = true;
      this.dragOffset = {
        x: pos.x - this.points[pointIndex].x,
        y: pos.y - this.points[pointIndex].y,
      };
      this.canvas.style.cursor = "grabbing";
    } else {
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

  ensurePointOffsetsInitialized() {
    while (this.params.pointOffsets.length < this.points.length) {
      this.params.pointOffsets.push(0);
    }
    if (this.params.pointOffsets.length > this.points.length) {
      this.params.pointOffsets = this.params.pointOffsets.slice(
        0,
        this.points.length,
      );
    }
  }

  onKeyDown(e) {
    if (
      (e.key === "Delete" || e.key === "Backspace") &&
      this.selectedPointIndex !== -1
    ) {
      this.removePoint(this.selectedPointIndex);
      e.preventDefault();
      return;
    }

    if (this.selectedPointIndex !== -1 && (e.key === "+" || e.key === "=")) {
      this.ensurePointOffsetsInitialized();
      this.params.pointOffsets[this.selectedPointIndex] += 1;
      this.updateDisplay();
      e.preventDefault();
      return;
    }

    if (this.selectedPointIndex !== -1 && e.key === "0") {
      this.ensurePointOffsetsInitialized();
      this.params.pointOffsets[this.selectedPointIndex] = 0;
      this.updateDisplay();
      e.preventDefault();
      return;
    }

    if (this.selectedPointIndex !== -1 && (e.key === "-" || e.key === "_")) {
      this.ensurePointOffsetsInitialized();
      this.params.pointOffsets[this.selectedPointIndex] = Math.max(
        1,
        this.params.pointOffsets[this.selectedPointIndex] - 1,
      );
      this.updateDisplay();
      e.preventDefault();
      return;
    }

    if (this.selectedPointIndex !== -1) {
      const step = e.shiftKey ? 10 : 1;
      let changed = false;

      if (e.key === "ArrowLeft") {
        this.points[this.selectedPointIndex].x -= step;
        changed = true;
      } else if (e.key === "ArrowRight") {
        this.points[this.selectedPointIndex].x += step;
        changed = true;
      } else if (e.key === "ArrowUp") {
        this.points[this.selectedPointIndex].y -= step;
        changed = true;
      } else if (e.key === "ArrowDown") {
        this.points[this.selectedPointIndex].y += step;
        changed = true;
      }

      if (changed) {
        this.updateDisplay();
        e.preventDefault();
        return;
      }
    }
  }

  onMouseMove(e) {
    const pos = this.getMousePos(e);

    if (this.dragging && this.selectedPointIndex !== -1) {
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
      const pointIndex = this.findPointAt(pos.x, pos.y);
      this.canvas.style.cursor = pointIndex !== -1 ? "grab" : "crosshair";

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
      this.addPointAtPosition(pos.x, pos.y);
    }
  }

  onTouchStart(e) {
    e.preventDefault();
    const pos = this.getTouchPos(e);
    const screenPos = this.cartesianToScreen(pos.x, pos.y);
    this.onMouseDown({
      clientX: screenPos.x + this.canvasRect.left,
      clientY: screenPos.y + this.canvasRect.top,
    });
  }

  onTouchMove(e) {
    e.preventDefault();
    const pos = this.getTouchPos(e);
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
    this.pointTypes.push(0);
    this.params.pointOffsets.push(this.params.strokeWidth);
    this.selectedPointIndex = this.points.length - 1;
    this.updateDisplay();
  }

  removePoint(index) {
    this.points.splice(index, 1);
    this.pointTypes.splice(index, 1);
    this.params.pointOffsets.splice(index, 1);

    if (this.selectedPointIndex === index) {
      this.selectedPointIndex = -1;
    } else if (this.selectedPointIndex > index) {
      this.selectedPointIndex--;
    }

    this.updateDisplay();
  }

  togglePointType(index) {
    if (index >= 0 && index < this.pointTypes.length) {
      this.pointTypes[index] = this.pointTypes[index] === 0 ? 1 : 0;
    }
  }

  clearPoints() {
    this.points = [];
    this.pointTypes = [];
    this.params.pointOffsets = [];
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
        types: [0, 0, 0, 0],
      },
      triangle: {
        points: [
          { x: 100, y: 100 },
          { x: 300, y: 300 },
          { x: 500, y: 100 },
        ],
        types: [1, 1, 1],
      },
      square: {
        points: [
          { x: 200, y: 200 },
          { x: 400, y: 200 },
          { x: 400, y: 400 },
          { x: 200, y: 400 },
        ],
        types: [1, 1, 1, 1],
      },
    };

    if (presets[name]) {
      this.points = [...presets[name].points];
      this.pointTypes = [...presets[name].types];
      this.params.pointOffsets = new Array(this.points.length).fill(0);
      this.selectedPointIndex = -1;
      this.updateDisplay();
    }
  }

  generateCurve() {
    if (this.points.length < 2) {
      return [];
    }

    try {
      const webPoints = this.points.map((p, i) => {
        const pointType =
          this.pointTypes[i] === 1 ? WebPointType.Corner : WebPointType.Smooth;
        return WebPoint.new_with_type(p.x, p.y, pointType);
      });

      const options = new CurveFitterOptions();
      options.set_cyclic(this.params.cyclic);

      return fit_curve(webPoints, options);
    } catch (error) {
      console.error("Error generating curve:", error);
      return [];
    }
  }

  generateStrokeCurve() {
    if (this.points.length < 2) {
      return [];
    }

    try {
      const webPoints = this.points.map((p, i) => {
        const pointType =
          this.pointTypes[i] === 1 ? WebPointType.Corner : WebPointType.Smooth;
        return WebPoint.new_with_type(p.x, p.y, pointType);
      });

      const options = new CurveFitterOptions();
      options.set_cyclic(this.params.cyclic);

      const strokeOptions = new StrokeOptions();
      this.ensurePointOffsetsInitialized();
      const strokeWidths = this.params.pointOffsets.map(
        (v) => v + this.params.strokeWidth,
      );
      strokeOptions.set_widths(strokeWidths);
      if (this.params.strokeCap === "butt") strokeOptions.set_cap_butt();
      else if (this.params.strokeCap === "square")
        strokeOptions.set_cap_square();
      else strokeOptions.set_cap_round();

      if (this.params.strokeJoin === "bevel") strokeOptions.set_join_bevel();
      else if (this.params.strokeJoin === "miter")
        strokeOptions.set_join_miter();
      else strokeOptions.set_join_round();

      return fit_curve_with_stroke(webPoints, options, strokeOptions);
    } catch (error) {
      console.error("Error generating stroke curve:", error);
      return [];
    }
  }

  updateDisplay() {
    this.clearCanvas();

    if (this.points.length >= 2) {
      // If stroke is enabled, draw both stroke outline and the original path
      if (this.params.strokeEnabled) {
        const strokeSegments = this.generateStrokeCurve();
        this.drawStrokeOutline(strokeSegments);

        if (this.params.showControlPoints) {
          this.drawStrokeControlPoints(strokeSegments);
        }
      }

      // Draw the original curve on top
      const curveSegments = this.generateCurve();
      this.drawCurve(curveSegments);

      if (this.params.showControlPoints) {
        this.drawControlPoints(curveSegments);
      }
    }

    this.drawPoints();
  }

  clearCanvas() {
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }

  drawCurve(segments) {
    if (segments.length === 0) return;

    const ctx = this.ctx;

    ctx.strokeStyle = "orangered";
    ctx.lineWidth = 2;
    ctx.lineCap = "round";
    ctx.lineJoin = "round";

    ctx.beginPath();

    const startScreen = this.cartesianToScreen(
      segments[0].start_x,
      segments[0].start_y,
    );
    ctx.moveTo(startScreen.x, startScreen.y);

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

  drawStrokeOutline(segments) {
    if (segments.length === 0) return;

    const ctx = this.ctx;

    ctx.fillStyle = "rgba(255, 69, 0, 0.2)";
    ctx.strokeStyle = "rgba(255, 69, 0, 0.8)";
    ctx.lineWidth = 1;

    ctx.beginPath();

    const startScreen = this.cartesianToScreen(
      segments[0].start_x,
      segments[0].start_y,
    );
    ctx.moveTo(startScreen.x, startScreen.y);

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
    ctx.closePath();
    ctx.fill();
    ctx.stroke();
  }

  drawControlPoints(segments) {
    const ctx = this.ctx;

    ctx.strokeStyle = "#cba6f7";
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);

    for (const segment of segments) {
      const startScreen = this.cartesianToScreen(
        segment.start_x,
        segment.start_y,
      );
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);
      const endScreen = this.cartesianToScreen(segment.end_x, segment.end_y);

      ctx.beginPath();
      ctx.moveTo(startScreen.x, startScreen.y);
      ctx.lineTo(cp1Screen.x, cp1Screen.y);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(cp2Screen.x, cp2Screen.y);
      ctx.lineTo(endScreen.x, endScreen.y);
      ctx.stroke();
    }

    ctx.setLineDash([]);

    ctx.fillStyle = "#cba6f7";

    for (const segment of segments) {
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);

      ctx.beginPath();
      ctx.arc(cp1Screen.x, cp1Screen.y, 4, 0, Math.PI * 2);
      ctx.fill();

      ctx.beginPath();
      ctx.arc(cp2Screen.x, cp2Screen.y, 4, 0, Math.PI * 2);
      ctx.fill();
    }
  }

  drawStrokeControlPoints(segments) {
    const ctx = this.ctx;

    ctx.strokeStyle = "rgba(203, 166, 247, 0.5)";
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);

    for (let i = 0; i < segments.length; i++) {
      const segment = segments[i];
      const startScreen = this.cartesianToScreen(
        segment.start_x,
        segment.start_y,
      );
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);
      const endScreen = this.cartesianToScreen(segment.end_x, segment.end_y);

      ctx.beginPath();
      ctx.moveTo(startScreen.x, startScreen.y);
      ctx.lineTo(cp1Screen.x, cp1Screen.y);
      ctx.stroke();

      ctx.beginPath();
      ctx.moveTo(cp2Screen.x, cp2Screen.y);
      ctx.lineTo(endScreen.x, endScreen.y);
      ctx.stroke();
    }

    ctx.setLineDash([]);

    ctx.fillStyle = "rgba(203, 166, 247, 0.6)";

    for (let i = 0; i < segments.length; i++) {
      const segment = segments[i];
      const cp1Screen = this.cartesianToScreen(segment.cp1_x, segment.cp1_y);
      const cp2Screen = this.cartesianToScreen(segment.cp2_x, segment.cp2_y);

      ctx.beginPath();
      ctx.arc(cp1Screen.x, cp1Screen.y, 3, 0, Math.PI * 2);
      ctx.fill();

      ctx.beginPath();
      ctx.arc(cp2Screen.x, cp2Screen.y, 3, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.fillStyle = "rgba(100, 200, 255, 0.8)";

    for (let i = 0; i < segments.length; i++) {
      const segment = segments[i];
      const startScreen = this.cartesianToScreen(
        segment.start_x,
        segment.start_y,
      );
      const endScreen = this.cartesianToScreen(segment.end_x, segment.end_y);

      ctx.beginPath();
      ctx.arc(startScreen.x, startScreen.y, 5, 0, Math.PI * 2);
      ctx.fill();

      if (i === segments.length - 1) {
        ctx.beginPath();
        ctx.arc(endScreen.x, endScreen.y, 5, 0, Math.PI * 2);
        ctx.fill();
      }
    }

    ctx.fillStyle = "rgba(255, 255, 255, 0.8)";
    ctx.font = "10px monospace";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";

    for (let i = 0; i < segments.length; i++) {
      const segment = segments[i];
      const startScreen = this.cartesianToScreen(
        segment.start_x,
        segment.start_y,
      );
      ctx.fillText(`${i}`, startScreen.x - 10, startScreen.y - 10);
    }
  }

  drawPoints() {
    const ctx = this.ctx;

    for (let i = 0; i < this.points.length; i++) {
      const point = this.points[i];
      const pointType = this.pointTypes[i];
      const isSelected = i === this.selectedPointIndex;
      const isCorner = pointType === 1;

      const screenPos = this.cartesianToScreen(point.x, point.y);

      ctx.fillStyle = isSelected ? "#f38ba8" : "#11111b";
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 1;

      ctx.beginPath();
      if (isCorner) {
        const size = isSelected ? 6 : 4;
        ctx.rect(screenPos.x - size, screenPos.y - size, size * 2, size * 2);
      } else {
        const radius = isSelected ? 6 : 4;
        ctx.arc(screenPos.x, screenPos.y, radius, 0, Math.PI * 2);
      }
      ctx.fill();
      ctx.stroke();

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

  async downloadSvg() {
    const svg = this.generateSvg();
    if (!svg) return;

    const blob = new Blob([svg], { type: "image/svg+xml" });
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
    const svg = this.generateSvg();
    if (!svg) return;

    try {
      await navigator.clipboard.writeText(svg);
      console.log("SVG copied to clipboard!");
    } catch (error) {
      console.error("Failed to copy to clipboard:", error);
    }
  }

  generateSvg() {
    if (this.points.length < 2) {
      return null;
    }

    try {
      const webPoints = this.points.map((p, i) => {
        const pointType =
          this.pointTypes[i] === 1 ? WebPointType.Corner : WebPointType.Smooth;
        return WebPoint.new_with_type(p.x, p.y, pointType);
      });

      const curveOptions = new CurveFitterOptions();
      curveOptions.set_cyclic(this.params.cyclic);

      const strokeOptions = new StrokeOptions();
      this.ensurePointOffsetsInitialized();
      const strokeWidths = this.params.pointOffsets.map(
        (v) => v + this.params.strokeWidth,
      );
      strokeOptions.set_widths(strokeWidths);

      if (this.params.strokeCap === "butt") strokeOptions.set_cap_butt();
      else if (this.params.strokeCap === "square")
        strokeOptions.set_cap_square();
      else strokeOptions.set_cap_round();

      if (this.params.strokeJoin === "bevel") strokeOptions.set_join_bevel();
      else if (this.params.strokeJoin === "miter")
        strokeOptions.set_join_miter();
      else strokeOptions.set_join_round();

      const pathData = curve_to_svg_path_with_stroke(
        webPoints,
        curveOptions,
        strokeOptions,
        this.params.strokeEnabled,
      );

      const canvasHeight = this.canvas.getBoundingClientRect().height;
      const fillStyle = this.params.strokeEnabled
        ? 'fill="#667eea"'
        : 'fill="none" stroke="#667eea" stroke-width="3"';

      return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${this.canvas.width} ${canvasHeight}">
  <path d="${pathData}" ${fillStyle} stroke-linecap="round"/>
</svg>`;
    } catch (error) {
      console.error("Error generating SVG:", error);
      return null;
    }
  }
}

// Initialize the demo
const demo = new CurveFitterDemo();
window.demo = demo;

document.addEventListener("DOMContentLoaded", () => {
  demo.init();
});
