//! Simplified J2/Kepler orbit-reference model for local portfolio experiments.
//!
//! This file is intentionally **not** an SGP4/SDP4 implementation, does not
//! ingest a live NORAD catalog, and does not establish conjunction-assessment or
//! collision-avoidance authority. It preserves reusable typed orbital-state,
//! Kepler iteration, secular J2-rate, and relative-geometry mechanisms.

use std::f64::consts::PI;

const MU_EARTH: f64 = 398_600.4418; // km^3/s^2
const EARTH_RADIUS_KM: f64 = 6_378.137;
const J2: f64 = 0.001_082_63;
const MINUTES_PER_DAY: f64 = 1_440.0;
const TWO_PI: f64 = 2.0 * PI;
pub const EVIDENCE_STATE: &str =
    "LOCAL_J2_KEPLER_REFERENCE_NOT_SGP4_OR_COLLISION_AVOIDANCE_AUTHORITY";

#[derive(Debug, Clone)]
pub struct OrbitalElements {
    pub object_id: u32,
    pub mean_motion_rev_day: f64,
    pub eccentricity: f64,
    pub inclination_deg: f64,
    pub raan_deg: f64,
    pub arg_perigee_deg: f64,
    pub mean_anomaly_deg: f64,
}

impl OrbitalElements {
    pub fn validate(&self) -> Result<(), &'static str> {
        let finite = [
            self.mean_motion_rev_day,
            self.eccentricity,
            self.inclination_deg,
            self.raan_deg,
            self.arg_perigee_deg,
            self.mean_anomaly_deg,
        ]
        .iter()
        .all(|value| value.is_finite());
        if !finite {
            return Err("orbital elements must be finite");
        }
        if self.mean_motion_rev_day <= 0.0 {
            return Err("mean motion must be positive");
        }
        if !(0.0..1.0).contains(&self.eccentricity) {
            return Err("eccentricity must be in [0,1)");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StateVector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub epoch_minutes: f64,
}

impl StateVector {
    pub fn position_magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn velocity_magnitude(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }

    pub fn altitude_km(&self) -> f64 {
        self.position_magnitude() - EARTH_RADIUS_KM
    }
}

/// Simplified two-body propagation with secular J2 angle rates.
///
/// Historical source called this SGP4. The corrected type name prevents that
/// claim: drag, deep-space terms, full SGP4 initialization, and TLE parsing are
/// not implemented here.
pub struct J2KeplerPropagator {
    n0: f64,
    a0_km: f64,
    e0: f64,
    i0: f64,
    raan0: f64,
    arg_perigee0: f64,
    mean_anomaly0: f64,
}

impl J2KeplerPropagator {
    pub fn new(elements: OrbitalElements) -> Result<Self, &'static str> {
        elements.validate()?;
        let n0 = elements.mean_motion_rev_day * TWO_PI / MINUTES_PER_DAY;
        // n0 is rad/min; convert to rad/s before applying SI-consistent mu.
        let n0_s = n0 / 60.0;
        let a0_km = (MU_EARTH / (n0_s * n0_s)).powf(1.0 / 3.0);
        Ok(Self {
            n0,
            a0_km,
            e0: elements.eccentricity,
            i0: elements.inclination_deg.to_radians(),
            raan0: elements.raan_deg.to_radians(),
            arg_perigee0: elements.arg_perigee_deg.to_radians(),
            mean_anomaly0: elements.mean_anomaly_deg.to_radians(),
        })
    }

    pub fn propagate(&self, t_min: f64) -> Result<StateVector, &'static str> {
        if !t_min.is_finite() {
            return Err("propagation time must be finite");
        }

        let cos_i = self.i0.cos();
        let sin_i = self.i0.sin();
        let p_km = self.a0_km * (1.0 - self.e0 * self.e0);
        if p_km <= 0.0 {
            return Err("invalid semilatus rectum");
        }

        let radius_ratio_sq = (EARTH_RADIUS_KM / p_km).powi(2);
        let raan_rate = -1.5 * J2 * self.n0 * radius_ratio_sq * cos_i;
        let arg_rate = 0.75 * J2 * self.n0 * radius_ratio_sq * (5.0 * cos_i * cos_i - 1.0);
        let mean_rate = self.n0
            * (1.0 + 0.75 * J2 * radius_ratio_sq * (3.0 * cos_i * cos_i - 1.0));

        let mean_anomaly = self.mean_anomaly0 + mean_rate * t_min;
        let raan = self.raan0 + raan_rate * t_min;
        let arg_perigee = self.arg_perigee0 + arg_rate * t_min;

        let mut eccentric_anomaly = mean_anomaly;
        for _ in 0..20 {
            let denominator = 1.0 - self.e0 * eccentric_anomaly.cos();
            if denominator.abs() < 1e-12 {
                return Err("Kepler iteration denominator collapsed");
            }
            let delta = (eccentric_anomaly - self.e0 * eccentric_anomaly.sin() - mean_anomaly)
                / denominator;
            eccentric_anomaly -= delta;
            if delta.abs() < 1e-12 {
                break;
            }
        }

        let sin_half = (eccentric_anomaly / 2.0).sin();
        let cos_half = (eccentric_anomaly / 2.0).cos();
        let true_anomaly = 2.0
            * ((1.0 + self.e0).sqrt() * sin_half)
                .atan2((1.0 - self.e0).sqrt() * cos_half);
        let radius = self.a0_km * (1.0 - self.e0 * eccentric_anomaly.cos());
        if !radius.is_finite() || radius <= 0.0 {
            return Err("propagated radius is invalid");
        }

        let argument_of_latitude = arg_perigee + true_anomaly;
        let cos_u = argument_of_latitude.cos();
        let sin_u = argument_of_latitude.sin();
        let cos_raan = raan.cos();
        let sin_raan = raan.sin();

        let x = radius * (cos_raan * cos_u - sin_raan * sin_u * cos_i);
        let y = radius * (sin_raan * cos_u + cos_raan * sin_u * cos_i);
        let z = radius * sin_u * sin_i;

        // Perifocal velocity rotated into ECI. This remains a simplified
        // osculating-state reference; it is not an operational propagator.
        let h = (MU_EARTH * p_km).sqrt();
        let vx_p = -MU_EARTH / h * true_anomaly.sin();
        let vy_p = MU_EARTH / h * (self.e0 + true_anomaly.cos());
        let cos_w = arg_perigee.cos();
        let sin_w = arg_perigee.sin();
        let r11 = cos_raan * cos_w - sin_raan * sin_w * cos_i;
        let r12 = -cos_raan * sin_w - sin_raan * cos_w * cos_i;
        let r21 = sin_raan * cos_w + cos_raan * sin_w * cos_i;
        let r22 = -sin_raan * sin_w + cos_raan * cos_w * cos_i;
        let r31 = sin_w * sin_i;
        let r32 = cos_w * sin_i;
        let vx = r11 * vx_p + r12 * vy_p;
        let vy = r21 * vx_p + r22 * vy_p;
        let vz = r31 * vx_p + r32 * vy_p;

        Ok(StateVector {
            x,
            y,
            z,
            vx,
            vy,
            vz,
            epoch_minutes: t_min,
        })
    }

    pub fn separation_km(left: &StateVector, right: &StateVector) -> f64 {
        let dx = left.x - right.x;
        let dy = left.y - right.y;
        let dz = left.z - right.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    pub fn relative_velocity_kms(left: &StateVector, right: &StateVector) -> f64 {
        let dvx = left.vx - right.vx;
        let dvy = left.vy - right.vy;
        let dvz = left.vz - right.vz;
        (dvx * dvx + dvy * dvy + dvz * dvz).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_elements() -> OrbitalElements {
        OrbitalElements {
            object_id: 25_544,
            mean_motion_rev_day: 15.49,
            eccentricity: 0.000_670_3,
            inclination_deg: 51.6434,
            raan_deg: 247.4627,
            arg_perigee_deg: 130.536,
            mean_anomaly_deg: 325.0288,
        }
    }

    #[test]
    fn propagated_state_is_finite_and_leo_like() {
        let propagator = J2KeplerPropagator::new(sample_elements()).unwrap();
        let state = propagator.propagate(0.0).unwrap();
        assert!(state.position_magnitude().is_finite());
        assert!(state.velocity_magnitude().is_finite());
        assert!((250.0..600.0).contains(&state.altitude_km()));
    }

    #[test]
    fn invalid_elements_fail_closed() {
        let mut elements = sample_elements();
        elements.eccentricity = 1.0;
        assert!(J2KeplerPropagator::new(elements).is_err());
    }

    #[test]
    fn relative_geometry_is_nonnegative() {
        let propagator = J2KeplerPropagator::new(sample_elements()).unwrap();
        let first = propagator.propagate(0.0).unwrap();
        let second = propagator.propagate(1.0).unwrap();
        assert!(J2KeplerPropagator::separation_km(&first, &second) > 0.0);
        assert!(J2KeplerPropagator::relative_velocity_kms(&first, &second) > 0.0);
    }

    #[test]
    fn evidence_boundary_is_explicit() {
        assert_eq!(
            EVIDENCE_STATE,
            "LOCAL_J2_KEPLER_REFERENCE_NOT_SGP4_OR_COLLISION_AVOIDANCE_AUTHORITY"
        );
    }
}
