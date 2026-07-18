use kurbo::{BezPath, CubicBez, PathEl, PathSeg, Point, QuadBez};

use crate::{
    tangents,
    var_offset::SegmentWidth,
    var_stroke::{VariableStroke, VariableStrokeCtx},
};

/// How the stroke width is interpolated between the per-point widths.
///
/// The offset tangent is `(1 - k*d)x' + d'*n`, so wherever the width slope
/// `d'` jumps between segments the outline kinks. `Linear` has that jump at
/// every on-curve point; the other profiles are C1 across points.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WidthProfile {
    /// Piecewise-linear between points (width slope jumps at points)
    Linear,
    /// Per-segment smoothstep easing: zero width slope at every point.
    /// The default: C1 everywhere, so the outline has no joint kinks.
    #[default]
    Smoothstep,
    /// Monotone cubic (Fritsch-Carlson) through the width samples: continuous
    /// slope at intermediate points, zero slope only at width extrema
    MonotoneCubic,
}

/// Number of widths `stroke` expects: one per on-curve point. Degenerate
/// segments contribute no point, and the closing segment of a subpath that
/// returns to its start reuses the start width rather than expecting its own.
fn count_width_points(elements: &[PathEl]) -> usize {
    let mut count = 0;
    let mut start = Point::ZERO;
    let mut pos = Point::ZERO;
    // Whether the last width-consuming segment of the current subpath ended
    // back at the subpath start (its width wraps to the start width)
    let mut wrapped = false;

    for el in elements {
        match *el {
            PathEl::MoveTo(p) => {
                if wrapped {
                    count -= 1;
                }
                wrapped = false;
                count += 1;
                start = p;
                pos = p;
            }
            PathEl::LineTo(p1) => {
                if p1 != pos {
                    count += 1;
                    pos = p1;
                    wrapped = pos == start;
                }
            }
            PathEl::QuadTo(p1, p2) => {
                if p1 != pos || p2 != pos {
                    count += 1;
                    pos = p2;
                    wrapped = pos == start;
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != pos || p2 != pos || p3 != pos {
                    count += 1;
                    pos = p3;
                    wrapped = pos == start;
                }
            }
            PathEl::ClosePath => {
                pos = start;
            }
        }
    }
    if wrapped {
        count -= 1;
    }

    count
}

/// Fritsch-Carlson slopes (per unit chord) at each point of one subpath.
///
/// `w` has one width per point, `chords` one length per segment
/// (`w.len() == chords.len() + 1`). For a cyclic subpath the first and last
/// points coincide and their neighbors wrap around.
fn monotone_point_slopes(w: &[f64], chords: &[f64], cyclic: bool) -> Vec<f64> {
    let m = chords.len();
    let secant = |k: usize| (w[k + 1] - w[k]) / chords[k].max(1e-12);

    (0..=m)
        .map(|k| {
            let (s_in, s_out) = if cyclic {
                let k_eff = k % m;
                let prev = if k_eff == 0 { m - 1 } else { k_eff - 1 };
                (secant(prev), secant(k_eff))
            } else if k == 0 {
                (secant(0), secant(0))
            } else if k == m {
                (secant(m - 1), secant(m - 1))
            } else {
                (secant(k - 1), secant(k))
            };

            if s_in * s_out <= 0.0 {
                // Width extremum (or a flat side): zero slope keeps the
                // interpolant monotone and the extremum kink-free
                0.0
            } else {
                // Harmonic mean; bounded by 2*min(s_in, s_out), which keeps
                // every segment monotone (Fritsch-Carlson condition)
                2.0 / (1.0 / s_in + 1.0 / s_out)
            }
        })
        .collect()
}

/// Per-segment width functions for the whole path, in traversal order.
///
/// Mirrors the traversal rules of `stroke`: one entry per non-degenerate
/// drawing segment, widths consumed cyclically so a closed subpath's seam
/// reuses the first width.
fn plan_segment_widths(
    elements: &[PathEl],
    widths: &[f64],
    profile: WidthProfile,
) -> Vec<SegmentWidth> {
    // Gather per-subpath point widths and segment chords
    struct SubpathPlan {
        widths: Vec<f64>,
        chords: Vec<f64>,
        wrapped: bool,
    }

    let mut subpaths: Vec<SubpathPlan> = Vec::new();
    let mut current: Option<SubpathPlan> = None;
    let mut start = Point::ZERO;
    let mut pos = Point::ZERO;
    let mut widx = 0usize;
    let n = widths.len();

    let push_segment =
        |current: &mut Option<SubpathPlan>, widx: &mut usize, chord: f64, wrapped: bool| {
            if let Some(sp) = current {
                sp.widths.push(widths[*widx % n]);
                *widx += 1;
                sp.chords.push(chord);
                sp.wrapped = wrapped;
            }
        };

    for el in elements {
        match *el {
            PathEl::MoveTo(p) => {
                if let Some(sp) = current.take()
                    && !sp.chords.is_empty()
                {
                    subpaths.push(sp);
                }
                current = Some(SubpathPlan {
                    widths: vec![widths[widx % n]],
                    chords: Vec::new(),
                    wrapped: false,
                });
                widx += 1;
                start = p;
                pos = p;
            }
            PathEl::LineTo(p1) => {
                if p1 != pos {
                    push_segment(&mut current, &mut widx, (p1 - pos).hypot(), p1 == start);
                    pos = p1;
                }
            }
            PathEl::QuadTo(p1, p2) => {
                if p1 != pos || p2 != pos {
                    push_segment(&mut current, &mut widx, (p2 - pos).hypot(), p2 == start);
                    pos = p2;
                }
            }
            PathEl::CurveTo(p1, p2, p3) => {
                if p1 != pos || p2 != pos || p3 != pos {
                    push_segment(&mut current, &mut widx, (p3 - pos).hypot(), p3 == start);
                    pos = p3;
                }
            }
            PathEl::ClosePath => {
                pos = start;
            }
        }
    }
    if let Some(sp) = current.take()
        && !sp.chords.is_empty()
    {
        subpaths.push(sp);
    }

    // Turn each subpath into per-segment width functions
    let mut plan = Vec::new();
    for sp in &subpaths {
        match profile {
            WidthProfile::Linear => {
                for k in 0..sp.chords.len() {
                    plan.push(SegmentWidth::linear(sp.widths[k], sp.widths[k + 1]));
                }
            }
            WidthProfile::Smoothstep => {
                for k in 0..sp.chords.len() {
                    plan.push(SegmentWidth {
                        w0: sp.widths[k],
                        w1: sp.widths[k + 1],
                        m0: 0.0,
                        m1: 0.0,
                    });
                }
            }
            WidthProfile::MonotoneCubic => {
                let slopes = monotone_point_slopes(&sp.widths, &sp.chords, sp.wrapped);
                for k in 0..sp.chords.len() {
                    // Slopes are per unit chord; the segment parameter runs
                    // over one chord length
                    plan.push(SegmentWidth {
                        w0: sp.widths[k],
                        w1: sp.widths[k + 1],
                        m0: slopes[k] * sp.chords[k],
                        m1: slopes[k + 1] * sp.chords[k],
                    });
                }
            }
        }
    }

    plan
}

pub struct VariableStroker {
    pub tolerance: f64,
    pub width_profile: WidthProfile,
}

impl VariableStroker {
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            width_profile: WidthProfile::default(),
        }
    }

    /// Builder method for setting the width interpolation profile
    pub fn with_width_profile(mut self, profile: WidthProfile) -> Self {
        self.width_profile = profile;
        self
    }

    /// Stroke a path with variable widths.
    ///
    /// `widths` must contain exactly one width per on-curve point of the
    /// path; a closed subpath's seam reuses the first width. Returns an
    /// error if the count does not match.
    pub fn stroke(
        &self,
        path: impl IntoIterator<Item = PathEl>,
        widths: &[f64],
        style: &VariableStroke,
    ) -> Result<BezPath, String> {
        let elements: Vec<PathEl> = path.into_iter().collect();

        let expected = count_width_points(&elements);
        if widths.len() != expected {
            return Err(format!(
                "Width count mismatch: path has {} on-curve points, got {} widths",
                expected,
                widths.len()
            ));
        }

        let plan = plan_segment_widths(&elements, widths, self.width_profile);
        let mut seg_idx = 0usize;

        let mut ctx = VariableStrokeCtx::default();

        // Heuristic for join threshold based on average width (safety fallback)
        // Real threshold calc happens per-join now since width changes.
        ctx.join_thresh = 2.0 * self.tolerance;

        for el in elements {
            let p0 = ctx.last_pt;
            let w0 = ctx.last_width;

            match el {
                PathEl::MoveTo(p) => {
                    ctx.finish(style, self.tolerance);
                    let w = plan.get(seg_idx).map(|sw| sw.w0).unwrap_or(1.0);

                    ctx.start_pt = p;
                    ctx.last_pt = p;
                    ctx.start_width = w;
                    ctx.last_width = w;
                }
                PathEl::LineTo(p1) => {
                    if p1 != p0 {
                        let sw = plan[seg_idx];
                        seg_idx += 1;
                        let tangent = p1 - p0;

                        // Join at P0 using W0
                        ctx.do_join(style, tangent, w0, self.tolerance);
                        ctx.last_tan = tangent;

                        if sw.is_linear() {
                            ctx.do_line(tangent, p1, w0, sw.w1);
                        } else {
                            // Non-linear width curves the offsets of a
                            // straight line: use the cubic offset machinery
                            let c = CubicBez::new(
                                p0,
                                p0 + tangent / 3.0,
                                p0 + tangent * (2.0 / 3.0),
                                p1,
                            );
                            ctx.do_cubic(c, sw);
                            ctx.last_pt = p1;
                        }
                        ctx.last_width = sw.w1;
                    }
                }
                PathEl::QuadTo(p1, p2) => {
                    if p1 != p0 || p2 != p0 {
                        let sw = plan[seg_idx];
                        seg_idx += 1;
                        let q = QuadBez::new(p0, p1, p2);
                        let (tan0, tan1) = tangents(&PathSeg::Quad(q));

                        ctx.do_join(style, tan0, w0, self.tolerance);

                        // Convert Quad to Cubic for variable offset
                        ctx.do_cubic(q.raise(), sw);

                        ctx.last_tan = tan1;
                        ctx.last_width = sw.w1;
                    }
                }
                PathEl::CurveTo(p1, p2, p3) => {
                    if p1 != p0 || p2 != p0 || p3 != p0 {
                        let sw = plan[seg_idx];
                        seg_idx += 1;
                        let c = CubicBez::new(p0, p1, p2, p3);
                        let (tan0, tan1) = tangents(&PathSeg::Cubic(c));

                        ctx.do_join(style, tan0, w0, self.tolerance);

                        ctx.do_cubic(c, sw);

                        ctx.last_tan = tan1;
                        ctx.last_width = sw.w1;
                    }
                }
                PathEl::ClosePath => {
                    if p0 != ctx.start_pt {
                        let w_end = ctx.start_width;
                        let tangent = ctx.start_pt - p0;

                        ctx.do_join(style, tangent, w0, self.tolerance);
                        ctx.last_tan = tangent;
                        ctx.do_line(tangent, ctx.start_pt, w0, w_end);
                    }
                    ctx.finish_closed(style, self.tolerance);
                }
            }
        }
        ctx.finish(style, self.tolerance);
        Ok(ctx.output)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_width_count_open_path() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.curve_to((120.0, 20.0), (140.0, 40.0), (160.0, 40.0));

        // 3 on-curve points
        assert_eq!(count_width_points(path.elements()), 3);

        let stroker = VariableStroker::new(0.1);
        let style = VariableStroke::default();
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0], &style).is_ok());
        assert!(stroker.stroke(&path, &[10.0, 12.0], &style).is_err());
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0, 9.0], &style).is_err());
    }

    #[test]
    fn test_width_count_closed_path() {
        // Closed path whose final segment returns to the start: the seam
        // reuses the first width, so 3 distinct points need 3 widths
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((50.0, 80.0));
        path.line_to((0.0, 0.0));
        path.close_path();

        assert_eq!(count_width_points(path.elements()), 3);

        let stroker = VariableStroker::new(0.1);
        let style = VariableStroke::default();
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0], &style).is_ok());
        assert!(stroker.stroke(&path, &[10.0, 12.0, 8.0, 10.0], &style).is_err());
    }

    #[test]
    fn test_width_profiles_are_c1_at_joints() {
        // Open path, three segments with uneven chords and widths
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((150.0, 0.0));
        path.line_to((350.0, 0.0));
        let widths = [10.0, 20.0, 50.0, 25.0];

        for profile in [WidthProfile::Smoothstep, WidthProfile::MonotoneCubic] {
            let plan = plan_segment_widths(path.elements(), &widths, profile);
            assert_eq!(plan.len(), 3);
            // Endpoint widths are interpolated exactly
            for (k, sw) in plan.iter().enumerate() {
                assert_eq!(sw.eval(0.0), widths[k]);
                assert!((sw.eval(1.0) - widths[k + 1]).abs() < 1e-12);
            }
            // Width slope per unit arc length is continuous across joints
            let chords = [100.0, 50.0, 200.0];
            for k in 0..2 {
                let out_slope = plan[k].deriv(1.0) / chords[k];
                let in_slope = plan[k + 1].deriv(0.0) / chords[k + 1];
                assert!(
                    (out_slope - in_slope).abs() < 1e-12,
                    "{profile:?} joint {k}: {out_slope} != {in_slope}"
                );
            }
        }
    }

    #[test]
    fn test_monotone_profile_flat_at_extremum() {
        let mut path = BezPath::new();
        path.move_to((0.0, 0.0));
        path.line_to((100.0, 0.0));
        path.line_to((200.0, 0.0));
        // Width extremum at the middle point
        let plan = plan_segment_widths(
            path.elements(),
            &[10.0, 50.0, 10.0],
            WidthProfile::MonotoneCubic,
        );
        assert_eq!(plan[0].deriv(1.0), 0.0);
        assert_eq!(plan[1].deriv(0.0), 0.0);
        // Monotone: never exceeds the extremum width
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            assert!(plan[0].eval(t) <= 50.0 + 1e-9);
            assert!(plan[1].eval(t) <= 50.0 + 1e-9);
        }
    }
}
