# Debug Analysis: False Corner Point in Wave-Simple Test

## Problem Statement
When running `cargo run -- --test tests/wave-simple.json --validate`, point **(244.04, 169.09)** is being marked as a **Corner** even though it should be **Smooth**.

The input curve consists of 4 smooth points with variable-width stroking, so the refitted outline should also be perfectly smooth everywhere.

## Root Cause: IDENTIFIED ✅

### The Issue
**Point (244.04, 169.09) has an angle difference of 12.24°, which exceeds the 10° threshold, causing it to be marked as a Corner.**

### Exact Measurements
From the debug output:

```
Incoming tangent vector:  (-27.465427, -5.869552)  [magnitude: 28.086]
Outgoing tangent vector: (-33.354821, -15.064865)  [magnitude: 36.599]

Incoming angle:  -167.9370°
Outgoing angle: -155.6935°
Angle difference: 12.2435° (threshold: 10.0°)
```

The angle difference **12.24° > 10.0° threshold** → marked as **CORNER**

## Why This Happens

### Geometry Analysis

1. **Skeleton (original curve)**: Smooth at all points
   - Input points: (50,100) → (150,50) → (250,150) → (350,100)
   - All marked as `Smooth` with no explicit tangent angles

2. **Variable-width Stroking**: Widths [10.0, 15.0, **40.0**, 25.0]
   - Point 1: width 10
   - Point 2: width 15
   - Point 3: width **40** ← LARGEST WIDTH
   - Point 4: width 25 ← WIDTH DROPS from 40 to 25

3. **Outline Geometry Distortion**:
   - When stroking with variable width, the offset curves are computed from the skeleton
   - At point 3 (250, 150) with width 40, the outline curves are offset by ±20 units
   - Between points 3→4, width changes from 40→25 (dropping by 15 units)
   - This causes the outline curve geometry to have **different curvatures** than the skeleton
   - The outline "bends" more sharply than the skeleton to accommodate the width transition

4. **Tangent Vector Divergence**:
   - At point (244.04, 169.09), which is on the outline:
     - Incoming tangent (from left outline segment): -167.94°
     - Outgoing tangent (from right outline segment): -155.69°
     - These directions differ by **12.24°**
   
5. **False Positive Classification**:
   - The `is_corner()` function checks: `if angle_diff > 10.0° → PointType::Corner`
   - 12.24° > 10.0° → Marked as Corner
   - But geometrically, point (244, 169) is **NOT** a corner - it's just where the outline distortion is highest

## The Core Issue: Wrong Processing Level

**The problem is that we're detecting "corners" in the STROKE OUTLINE, not in the SKELETON.**

### Processing Pipeline
```
1. Input: 4 smooth points
   ↓
2. fit_curve(): Creates smooth skeleton BezPath
   ↓
3. stroke(): Creates outline BezPath from skeleton with variable widths
   ↓
4. refit_stroke(): ← WE ARE HERE
   - Extracts tangents from OUTLINE geometry
   - Compares incoming vs outgoing angles
   - Point (244, 169) gets >10° angle difference
   - Incorrectly marked as Corner
   ↓
5. spline.solve(): Uses the refitted outline
   - Lines 86-99 in spline.rs also check for corner-ness
   - But the damage is already done
```

## Why is This a Problem?

1. **Semantic Issue**: The original skeleton is smooth everywhere
2. **Algorithmic Issue**: Variable-width stroking creates outline geometry that doesn't preserve tangent continuity from the skeleton
3. **False Positive**: A smooth point in the skeleton appears as a corner just because the outline geometry is distorted
4. **Data Loss**: We're losing the information that this point should be smooth

## Solution Options

### Option A: Increase CORNER_THRESHOLD_DEGREES (Quick Fix)
**File**: `stroke_refitter.rs` line 10
```rust
// Change from:
const CORNER_THRESHOLD_DEGREES: f64 = 10.0;
// To:
const CORNER_THRESHOLD_DEGREES: f64 = 15.0; // or 20.0
```

**Pros**: Simple one-line fix
**Cons**: May allow actual corners to slip through; arbitrary threshold

**Verdict**: Not recommended - treats symptom, not cause

### Option B: Use Skeleton as Authority (Recommended)
Always use the skeleton's point types and angles as the source of truth.

**When to mark as Corner**:
- Only if the SKELETON had a corner
- Never based solely on outline geometry

**Implementation**:
- Modify `refit_stroke()` to accept original input points
- Use their `point_type` field as the definitive source
- Only use outline tangents to fine-tune smooth point angles

**Pros**: Preserves design intent; theoretically sound
**Cons**: Requires architectural change; need to thread input points through

### Option C: Adaptive Threshold Based on Width Change (Smart)
Use a dynamic threshold based on how much width changes:

```rust
fn compute_corner_threshold(width_before: f64, width_after: f64, base_threshold: f64) -> f64 {
    let width_ratio = (width_before - width_after).abs() / width_before.max(width_after);
    // Larger width changes allow larger angle differences
    base_threshold * (1.0 + width_ratio * 5.0)
}
```

**Example**:
- Width change 10→15: ratio 0.333 → threshold ≈ 12.6°
- Width change 40→25: ratio 0.6 → threshold ≈ 19.0°

**Pros**: Contextual, acknowledges geometry
**Cons**: Tuning-heavy; may be brittle

### Option D: G1 Smoothing Alignment
Currently:
- `CORNER_THRESHOLD_DEGREES = 10.0` (stroke_refitter.rs:10)
- `g1_smooth` uses 4.0° threshold (stroke_refitter.rs:531)

Alignment doesn't help here because point (244, 169) is marked Corner BEFORE smoothing, so it's skipped by `g1_smooth`.

## Verdict

**This is NOT a bug in spline.rs** - the issue originates in `stroke_refitter.rs` line 476.

The point is being correctly classified as Corner based on the 12.24° angle difference. The problem is that **variable-width stroking geometrically creates this angle difference in the outline, even when the skeleton is smooth.**

This is a **fundamental limitation of the current approach**: We can't reliably detect smooth vs corner points in a stroked outline without reference to the original skeleton.

## Recommended Next Step

Implement **Option B: Use Skeleton as Authority** by:
1. Modifying `refit_stroke()` signature to optionally accept original input points
2. Using original `point_type` values as the source of truth
3. Using outline tangents only to refine smooth point angles (via g1_smooth)
4. This preserves the designer's intent and avoids false positives

