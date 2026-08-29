//! Auto-exposure control law for [`RgbCamera`](super::RgbCamera): the pure
//! decision that maps a metered frame to the next sensor operating point. It
//! holds no hardware, so the convergence behaviour can be reasoned about and
//! unit-tested in isolation from the sensor and transport that drive it.

/// Metering summary of one raw frame, in the 8-bit MSB space. Produced by the
/// RGB pipeline's `meter` and consumed by [`auto_exposure_step`].
pub struct MeterResult {
    /// High-percentile luma (0..=255): how bright the bright parts are.
    pub p_high: f32,
    /// Fraction of samples at or above the near-saturation threshold.
    pub clip: f32,
}

/// The sensor operating point the controller drives.
#[derive(Clone, Copy)]
pub struct AutoExposureState {
    /// Integration time in lines.
    pub exposure: u32,
    /// Analog gain as a linear multiplier (1.0 = unity).
    pub gain: f32,
}

/// Sensor-reported ceilings the controller must respect.
#[derive(Clone, Copy)]
pub struct AutoExposureLimits {
    /// Largest integration time the sensor accepts, in lines.
    pub max_exposure: u32,
    /// Largest usable analog gain as a linear multiplier.
    pub max_gain: f32,
}

/// Outcome of one control step.
pub enum AutoExposureStep {
    /// The frame is within the converged band; hold the current operating point.
    Converged,
    /// Move to this operating point and re-meter.
    Adjust(AutoExposureState),
}

const TARGET_LUMA: f32 = 230.0; // drive the high percentile to just below saturation
const LUMA_LOW: f32 = 205.0; // converged band (lower bound)
const LUMA_HIGH: f32 = 248.0; // converged band (upper bound)
const CLIP_LIMIT: f32 = 0.02; // tolerate up to 2% near-saturated pixels
const HL_SCALE: f32 = 0.7; // forced exposure cut per step while highlights clip

/// One auto-exposure step: hold if the frame is converged, otherwise move to
/// the operating point that scales the metered light toward the target.
pub fn auto_exposure_step(
    state: AutoExposureState,
    limits: AutoExposureLimits,
    meter: &MeterResult,
) -> AutoExposureStep {
    if is_converged(meter) {
        return AutoExposureStep::Converged;
    }
    let light = state.exposure as f32 * state.gain;
    AutoExposureStep::Adjust(distribute(light * light_scale(meter), limits))
}

/// Whether a metered frame sits inside the converged band: bright enough,
/// without too many near-saturated pixels.
fn is_converged(meter: &MeterResult) -> bool {
    meter.clip <= CLIP_LIMIT && meter.p_high >= LUMA_LOW && meter.p_high <= LUMA_HIGH
}

/// Desired multiplicative change in total light. Clipping forces a reduction
/// even if the percentile looks fine (a small, very bright spot in an otherwise
/// dim scene).
fn light_scale(meter: &MeterResult) -> f32 {
    let luma_scale = TARGET_LUMA / meter.p_high.max(1.0);
    if meter.clip > CLIP_LIMIT {
        luma_scale.min(HL_SCALE)
    } else {
        luma_scale
    }
}

/// Split a desired total-light level across exposure and gain within the
/// sensor's ceilings. Exposure is the primary lever, preferred all the way to
/// its ceiling before any gain is added, since gain only amplifies noise.
fn distribute(desired_light: f32, limits: AutoExposureLimits) -> AutoExposureState {
    let max_exp = limits.max_exposure as f32;
    let desired = desired_light.clamp(1.0, max_exp * limits.max_gain);
    let (exposure, gain) = if desired <= max_exp {
        (desired, 1.0)
    } else {
        (max_exp, (desired / max_exp).min(limits.max_gain))
    };

    AutoExposureState {
        exposure: exposure.round() as u32,
        gain,
    }
}
