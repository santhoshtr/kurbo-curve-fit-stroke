# Skeleton-Aware Stroke Refitting Workflow

## Overview

The curve-fit library now supports **two distinct strategies** for refitting stroked paths:

1. **Outline-Based Refitting** (`refit_stroke`): Extracts curves purely from the stroke outline geometry
2. **Selective Skeleton Correction** (`refit_stroke_with_skeleton_correction`): **RECOMMENDED** Uses skeleton as error correction tool only

### Quick Comparison

| Aspect | Outline-Based | Selective (⭐) |
|--------|---------------|-----------------|
| **Visual Quality** | ⭐⭐⭐⭐⭐ Smoothest | ⭐⭐⭐⭐⭐ **Best** |
| **Skeleton Fidelity** | ⭐ Low | ⭐⭐⭐⭐ Excellent |
| **Complexity** | Simple | Medium |
| **Recommended Use** | General | Production ⭐ |
| **Function** | `refit_stroke()` | `refit_stroke_with_skeleton_correction()` |

---
## Problem Context

When a path is stroked with variable widths, the resulting outline geometry becomes distorted compared to the skeleton (original curve). This distortion is expected and normal, but it can cause:

- **False corner detection**: Smooth points on the skeleton may appear as corners in the outline
- **Angle contamination**: Angles extracted from the outline may not reflect the original skeleton's design intent
- **Loss of design information**: Without skeleton reference, the algorithm can't distinguish between intentional corners and geometric artifacts

### Example: Variable-Width Stroking

```
Skeleton: 50,100 → 150,50 → 250,150 → 350,100
Widths:   [10.0, 15.0, 40.0, 25.0]

Result: Outline curves have different curvatures than skeleton
Outcome: Point (244, 169) appears as corner (angle diff: 12.24°)
         even though skeleton has it as smooth
```

## Two Refitting Strategies

### Strategy 1: Outline-Based Refitting

```rust
let config = StrokeRefitterConfig::new();  // 15.0° thresholds
let refitted = refit_stroke(&stroke_path, &config)?;
```

**How it works:**
1. Extract on-curve points directly from stroke outline
2. Determine point type (Corner vs Smooth) based on angle difference
3. Apply G1 smoothing where angles are close
4. Fit final curve using Hobby's algorithm

**Strengths:**
- Simple and direct - works with any stroke outline
- No additional information needed beyond the stroke
- Good for most cases

**Limitations:**
- May misclassify smooth skeleton points as corners
- Loses original design intent
- Affected by stroking geometry distortions

**When to use:**
- General-purpose stroke refitting
- When original skeleton is not available
- For simple constant-width strokes

---

### Strategy 2: Selective Skeleton Correction (RECOMMENDED)

```rust
// Step 1: Register original skeleton for preservation
let skeleton_info = register_skeleton_for_preservation(
    &skeleton_curve,
    &input_points,     // Original input points with angles
    &widths,           // Stroke widths at each point
    is_closed,
)?;

// Step 2: Refit stroke using selective skeleton correction
let config = StrokeRefitterConfig::new();
let refitted = refit_stroke_with_skeleton_correction(
    &stroke_path,
    &skeleton_info,
    &config
)?;
```

**How it works:**
1. Extract on-curve points from stroke outline with outline-derived angles
2. Classify point types (Corner/Smooth) based on angle differences
3. Apply G1 smoothing to enforce continuity
4. **Detect which Smooth points STILL fail G1 continuity** (tolerance: 0.5°)
5. For failing points ONLY:
   - Match to skeleton location using offset distance
   - If confident match found: replace angles with skeleton angles
   - If no match: keep outline angles as-is
6. Re-apply G1 smoothing to propagate corrections to neighbors
7. Fit final curve using Hobby's algorithm

**Strengths:**
- **Combines best of both approaches**: outline smoothness + skeleton accuracy
- Preserves natural geometry of outline for well-behaved points
- Uses skeleton as **error correction tool**, not primary source
- Produces smoother curves than pure skeleton replacement
- Fewer false corner detections than pure outline-based
- Excellent balance of quality and fidelity

**Limitations:**
- Requires original skeleton curve and widths
- More complex than outline-based approach
- Points with G1 failures that don't match skeleton are left as-is

**When to use:**
- **Production workflows** - best overall quality
- Variable-width stroking where distortions cause problems
- When you want the smoothest possible result with skeleton awareness
- High-quality font/glyph rendering
- Round-trip workflows: fit → stroke → refit → high fidelity to original

**Visual Quality Comparison:**
```
Pure Outline    | Smooth natural curves, may have false corners
                | Good for simple cases, excellent visual quality
                |
Selective Corr. | Smooth curves with error correction
(RECOMMENDED)   | Fixes specific problems, preserves overall smoothness
                | Best visual + best accuracy
                |
Full Skeleton   | Preserves skeleton intent, may look "forced"
                | Good skeleton fidelity, potentially less smooth
```

---

## Usage Patterns

### Pattern 1: Simple Refitting (Outline-Based)

```json
{
  "operations": ["fit_curve", "stroke", "refit_stroke"],
  "outputs": {
    "fitted_curve": "fitted",
    "stroke_outline": "stroke",
    "refitted_stroke": "refitted"
  }
}
```

### Pattern 2: Production Workflow (Selective Skeleton Correction - Recommended)

```json
{
  "operations": [
    "fit_curve",                       // Fit original curve
    "stroke",                          // Generate stroke outline
    "refit_stroke",                    // Baseline outline-based refitting
    "register_skeleton",               // Register skeleton for correction
    "refit_with_skeleton_correction"   // Apply selective skeleton correction
  ],
  "outputs": {
    "fitted_curve": "fitted",
    "stroke_outline": "stroke",
    "refitted_stroke": "refitted-outline",
    "skeleton_corrected": "refitted-skeleton-corrected"
  }
}
```

This pattern generates both outputs, allowing validation:
- **refitted-outline**: Pure outline-based (baseline for comparison)
- **refitted-skeleton-corrected**: Selective correction (recommended for production) ⭐

---

## Test Cases

Seven comprehensive test cases demonstrate various scenarios:

### 1. **wave-simple** - Variable-Width Stroking
- 4-point smooth wave with widths [10.0, 15.0, 40.0, 25.0]
- Tests both strategies (outline-based and selective correction)
- Shows skeleton preservation with variable widths
- Key test for demonstrating selective correction advantages

### 2. **wave-constant-width** - Constant-Width Stroking
- Same wave with uniform width [5.0, 5.0, 5.0, 5.0]
- Simpler outline geometry
- Both strategies should produce similar results

### 3. **letter-o** - Closed Path
- Circular O-shape with variable widths
- Tests closed path handling
- Outline-based refitting only

### 4. **straight-line** - Simplest Case
- Diagonal line from (0,0) to (100,100)
- Minimal geometry
- Both strategies should be identical

### 5. **metapost-style** - Explicit Tangent Angles
- Curve with explicit incoming/outgoing angles
- Tests constraint preservation
- Both strategies preserve these constraints

### 6. **corner-angle** - Intentional Corners
- Explicit 90° corner with different incoming/outgoing angles
- Tests corner detection and preservation
- Both strategies respect explicit corners

### 7. **skeleton-preservation** - Focused Comparison
- Dedicated test comparing both strategies side-by-side
- Open path with variable widths
- Demonstrates selective correction advantages

---

## Configuration: StrokeRefitterConfig

Both strategies use `StrokeRefitterConfig` to customize behavior:

```rust
pub struct StrokeRefitterConfig {
    pub corner_threshold_degrees: f64,      // Default: 15.0°
    pub g1_smooth_threshold_degrees: f64,   // Default: 15.0°
}
```

### Preset Configurations

**Default (variable-width stroking)**
```rust
let config = StrokeRefitterConfig::new();
// corner_threshold: 15.0°
// g1_smooth_threshold: 15.0°
```

**Constant-width stroking (stricter)**
```rust
let config = StrokeRefitterConfig::constant_width();
// corner_threshold: 10.0°
// g1_smooth_threshold: 10.0°
```

**Aggressive smoothing (fewer corners)**
```rust
let config = StrokeRefitterConfig::aggressive_smooth();
// corner_threshold: 20.0°
// g1_smooth_threshold: 20.0°
```

**Precise corner preservation (intentional corners)**
```rust
let config = StrokeRefitterConfig::precise_corners();
// corner_threshold: 5.0°
// g1_smooth_threshold: 5.0°
```

**Custom thresholds**
```rust
let config = StrokeRefitterConfig::custom(12.0, 12.0);
```

### Understanding Thresholds

- **`corner_threshold_degrees`**: If incoming and outgoing angles differ by more than this, point is classified as a corner
- **`g1_smooth_threshold_degrees`**: If a smooth point has angle difference within this threshold, average the angles for G1 continuity
- **Should match**: For consistency, both thresholds should be the same value

---

## G1 Continuity Validation

Both strategies validate that output curves maintain **G1 continuity** (continuous tangent angles):

```
G1 Continuity: incoming_angle ≈ outgoing_angle (at each smooth point)
```

If validation fails:
- Error message shows exact kink location and angle difference
- Can be detected before output if needed
- Guides threshold tuning

### Example Valid Output
```
[Point 8] (244.04, 169.09) - Smooth
  Incoming:  -161.82°
  Outgoing:  -161.82°
  Difference: 0.00°
  ✓ OK (perfect G1 continuity)
```

---

## API Reference

### Outline-Based Refitting

```rust
pub fn refit_stroke(
    stroke_path: &BezPath,
    config: &StrokeRefitterConfig,
) -> Result<BezPath, String>
```

Refits a stroke outline using only outline geometry.

---

### Skeleton Registration

```rust
pub fn register_skeleton_for_preservation(
    skeleton_path: &BezPath,
    skeleton_angles: &[InputPoint],
    widths: &[f64],
    is_closed: bool,
) -> Result<SkeletonInfo, String>
```

Pre-computes skeleton information for angle preservation. Call this **before** stroking.

**Arguments:**
- `skeleton_path`: The original fitted curve (before stroking)
- `skeleton_angles`: Original InputPoints with explicit angles
- `widths`: Stroke width at each on-curve point
- `is_closed`: Whether the skeleton is a closed path

**Returns:**
- `SkeletonInfo`: Cached geometry for fast matching during refitting

---

### Selective Skeleton Correction Refitting

```rust
pub fn refit_stroke_with_skeleton_correction(
    stroke_path: &BezPath,
    skeleton_info: &SkeletonInfo,
    config: &StrokeRefitterConfig,
) -> Result<BezPath, String>
```

Refits a stroke outline using selective skeleton correction as an error correction tool.

**Arguments:**
- `stroke_path`: The stroked outline to refit
- `skeleton_info`: Pre-registered skeleton from `register_skeleton_for_preservation()`
- `config`: Refitting configuration

**Returns:**
- `BezPath`: Refitted curve with selective skeleton corrections applied

---

## Workflow Example: Production Refitting

```rust
// User provides input points with design intent
let input_points = vec![
    InputPoint { x: 50.0, y: 100.0, point_type: Smooth, ... },
    // ... more points
];

// Step 1: Fit the skeleton curve
let skeleton = fit_curve(input_points.clone(), is_closed)?;

// Step 2: Stroke the skeleton with variable widths
let widths = vec![10.0, 15.0, 20.0, 15.0];
let stroker = VariableStroker::new(0.1);
let stroke = stroker.stroke(&skeleton, &widths, &style);

// Step 3: Register skeleton for preservation
let skeleton_info = register_skeleton_for_preservation(
    &skeleton,
    &input_points,
    &widths,
    is_closed,
)?;

// Step 4: Refit using selective skeleton correction (RECOMMENDED)
let config = StrokeRefitterConfig::new();
let refitted = refit_stroke_with_skeleton_correction(&stroke, &skeleton_info, &config)?;

// Result: refitted ≈ skeleton (faithful to design intent)
//         Better than pure outline refitting (fewer false corners)
//         Smoother than full skeleton replacement
```

---

## Performance Notes

- **Outline-based refitting**: Very fast, single pass through outline
- **Skeleton-aware refitting**: Slightly slower due to matching phase, but still efficient
  - Matching uses ~20 samples per segment (numerical search)
  - Modern CPUs handle this easily for typical curves

---

## Future Enhancements

1. **Closed path skeleton preservation**: Extend skeleton registration to handle closed paths properly
2. **Width tolerance tuning**: Make `SKELETON_MATCH_TOLERANCE` configurable
3. **Multi-resolution skeleton matching**: For complex strokes with many segments
4. **Weighted angle blending**: Smooth transition between outline and skeleton angles based on confidence

---

## Running Tests

```bash
# Run all skeleton workflow tests
cargo run -p curve_fitter --release -- --test-dir curve_fitter/tests/

# Run specific test
cargo run -p curve_fitter --release -- --test curve_fitter/tests/wave-simple.json

# Compare both strategies (shows detailed validation)
cargo run -p curve_fitter --release -- --test curve_fitter/tests/skeleton-preservation.json
```

All 7 tests demonstrate the skeleton-aware workflow and validate G1 continuity.
