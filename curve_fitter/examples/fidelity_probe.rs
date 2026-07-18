use curve_fitter::var_stroke::VariableStroke;
use curve_fitter::var_stroker::{VariableStroker, WidthProfile};
use curve_fitter::{InputPoint, PointType, StrokeRefitterConfig, fit_curve, refit_stroke};
use kurbo::{ParamCurve, ParamCurveNearest, PathSeg, Point};

fn smooth(x: f64, y: f64) -> InputPoint {
    InputPoint { x, y, point_type: PointType::Smooth, incoming_angle: None, outgoing_angle: None }
}

// Max/avg distance from densely sampled refit outline to the raw outline
fn fidelity(raw: &kurbo::BezPath, refit: &kurbo::BezPath) -> (f64, f64) {
    let raw_segs: Vec<PathSeg> = raw.segments().collect();
    let mut max_d: f64 = 0.0;
    let mut sum = 0.0;
    let mut n = 0;
    for seg in refit.segments() {
        for i in 0..=20 {
            let p: Point = seg.eval(i as f64 / 20.0);
            let d = raw_segs
                .iter()
                .map(|rs| rs.nearest(p, 1e-9).distance_sq)
                .fold(f64::INFINITY, f64::min)
                .sqrt();
            max_d = max_d.max(d);
            sum += d;
            n += 1;
        }
    }
    (max_d, sum / n as f64)
}

fn run(name: &str, points: Vec<InputPoint>, widths: &[f64], closed: bool) {
    let fitted = fit_curve(points, closed).unwrap();
    for profile in [
        WidthProfile::Linear,
        WidthProfile::Smoothstep,
        WidthProfile::MonotoneCubic,
    ] {
        let raw = VariableStroker::new(0.1)
            .with_width_profile(profile)
            .stroke(&fitted, widths, &VariableStroke::default())
            .unwrap();
        let (refit, _) = refit_stroke(&raw, None, &StrokeRefitterConfig::new()).unwrap();
        let (max_d, avg_d) = fidelity(&raw, &refit);
        println!("{name} {profile:?}: refit max deviation {max_d:.3}, avg {avg_d:.3}");
    }
}

fn main() {
    run(
        "wave-simple  ",
        vec![smooth(50.0, 100.0), smooth(150.0, 50.0), smooth(250.0, 150.0), smooth(350.0, 100.0)],
        &[10.0, 15.0, 50.0, 25.0],
        false,
    );
    run(
        "circle-var   ",
        vec![smooth(272.0, 92.0), smooth(472.0, 292.0), smooth(275.0, 489.0), smooth(72.0, 295.0)],
        &[20.0, 60.0, 20.0, 20.0],
        true,
    );
}
