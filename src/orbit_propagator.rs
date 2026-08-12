//! Deterministic two-body orbit kernel for local conjunction simulation.
//!
//! This crate deliberately does not claim SGP4/SDP4 fidelity, flight
//! certification, operational spacecraft authority, or SpaceX affiliation.

use std::error::Error;
use std::f64::consts::PI;
use std::fmt::{Display, Formatter};

pub const MU_EARTH_KM3_S2: f64 = 398_600.441_8;
pub const EARTH_RADIUS_KM: f64 = 6_378.137;
const SECONDS_PER_DAY: f64 = 86_400.0;
const TWO_PI: f64 = 2.0 * PI;

#[derive(Debug, Clone, PartialEq)]
pub struct Tle {
    pub norad_id: u32,
    pub mean_motion_rev_day: f64,
    pub eccentricity: f64,
    pub inclination_deg: f64,
    pub raan_deg: f64,
    pub arg_perigee_deg: f64,
    pub mean_anomaly_deg: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
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
    pub fn position_magnitude(self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn velocity_magnitude(self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }

    pub fn altitude_km(self) -> f64 {
        self.position_magnitude() - EARTH_RADIUS_KM
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PropagationError {
    InvalidNoradId,
    InvalidMeanMotion,
    InvalidEccentricity,
    InvalidInclination,
    NonFiniteElement,
    SolverDidNotConverge,
}

impl Display for PropagationError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

impl Error for PropagationError {}

#[derive(Debug, Clone)]
pub struct TwoBodyPropagator {
    tle: Tle,
    mean_motion_rad_s: f64,
    semi_major_axis_km: f64,
}

impl TwoBodyPropagator {
    pub fn new(tle: Tle) -> Result<Self, PropagationError> {
        if tle.norad_id == 0 {
            return Err(PropagationError::InvalidNoradId);
        }
        let fields = [
            tle.mean_motion_rev_day,
            tle.eccentricity,
            tle.inclination_deg,
            tle.raan_deg,
            tle.arg_perigee_deg,
            tle.mean_anomaly_deg,
        ];
        if fields.iter().any(|value| !value.is_finite()) {
            return Err(PropagationError::NonFiniteElement);
        }
        if tle.mean_motion_rev_day <= 0.0 {
            return Err(PropagationError::InvalidMeanMotion);
        }
        if !(0.0..1.0).contains(&tle.eccentricity) {
            return Err(PropagationError::InvalidEccentricity);
        }
        if !(0.0..=180.0).contains(&tle.inclination_deg) {
            return Err(PropagationError::InvalidInclination);
        }

        let mean_motion_rad_s = tle.mean_motion_rev_day * TWO_PI / SECONDS_PER_DAY;
        let semi_major_axis_km =
            (MU_EARTH_KM3_S2 / mean_motion_rad_s.powi(2)).powf(1.0 / 3.0);
        Ok(Self {
            tle,
            mean_motion_rad_s,
            semi_major_axis_km,
        })
    }

    pub fn propagate(&self, minutes_since_epoch: f64) -> Result<StateVector, PropagationError> {
        if !minutes_since_epoch.is_finite() {
            return Err(PropagationError::NonFiniteElement);
        }
        let eccentricity = self.tle.eccentricity;
        let mut mean_anomaly = self.tle.mean_anomaly_deg.to_radians()
            + self.mean_motion_rad_s * minutes_since_epoch * 60.0;
        mean_anomaly = mean_anomaly.rem_euclid(TWO_PI);

        let mut eccentric_anomaly = mean_anomaly;
        let mut converged = false;
        for _ in 0..32 {
            let denominator = 1.0 - eccentricity * eccentric_anomaly.cos();
            let delta = (eccentric_anomaly
                - eccentricity * eccentric_anomaly.sin()
                - mean_anomaly)
                / denominator;
            eccentric_anomaly -= delta;
            if delta.abs() < 1e-13 {
                converged = true;
                break;
            }
        }
        if !converged {
            return Err(PropagationError::SolverDidNotConverge);
        }

        let cos_e = eccentric_anomaly.cos();
        let sin_e = eccentric_anomaly.sin();
        let radius_km = self.semi_major_axis_km * (1.0 - eccentricity * cos_e);
        let denominator = 1.0 - eccentricity * cos_e;
        let cos_nu = (cos_e - eccentricity) / denominator;
        let sin_nu = (1.0 - eccentricity * eccentricity).sqrt() * sin_e / denominator;
        let p = self.semi_major_axis_km * (1.0 - eccentricity * eccentricity);
        let velocity_scale = (MU_EARTH_KM3_S2 / p).sqrt();

        let x_pf = radius_km * cos_nu;
        let y_pf = radius_km * sin_nu;
        let vx_pf = -velocity_scale * sin_nu;
        let vy_pf = velocity_scale * (eccentricity + cos_nu);

        let raan = self.tle.raan_deg.to_radians();
        let inclination = self.tle.inclination_deg.to_radians();
        let arg_perigee = self.tle.arg_perigee_deg.to_radians();
        let (co, so) = (raan.cos(), raan.sin());
        let (ci, si) = (inclination.cos(), inclination.sin());
        let (cw, sw) = (arg_perigee.cos(), arg_perigee.sin());

        let r11 = co * cw - so * sw * ci;
        let r12 = -co * sw - so * cw * ci;
        let r21 = so * cw + co * sw * ci;
        let r22 = -so * sw + co * cw * ci;
        let r31 = sw * si;
        let r32 = cw * si;

        Ok(StateVector {
            x: r11 * x_pf + r12 * y_pf,
            y: r21 * x_pf + r22 * y_pf,
            z: r31 * x_pf + r32 * y_pf,
            vx: r11 * vx_pf + r12 * vy_pf,
            vy: r21 * vx_pf + r22 * vy_pf,
            vz: r31 * vx_pf + r32 * vy_pf,
            epoch_minutes: minutes_since_epoch,
        })
    }

    pub fn miss_distance(a: StateVector, b: StateVector) -> f64 {
        ((a.x - b.x).powi(2) + (a.y - b.y).powi(2) + (a.z - b.z).powi(2)).sqrt()
    }

    pub fn relative_velocity(a: StateVector, b: StateVector) -> f64 {
        ((a.vx - b.vx).powi(2) + (a.vy - b.vy).powi(2) + (a.vz - b.vz).powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iss_like_tle() -> Tle {
        Tle {
            norad_id: 25_544,
            mean_motion_rev_day: 15.49,
            eccentricity: 0.000_670_3,
            inclination_deg: 51.6434,
            raan_deg: 247.4627,
            arg_perigee_deg: 130.5360,
            mean_anomaly_deg: 325.0288,
        }
    }

    #[test]
    fn iss_like_state_has_physical_radius_and_speed() {
        let propagator = TwoBodyPropagator::new(iss_like_tle()).unwrap();
        let state = propagator.propagate(0.0).unwrap();
        assert!((300.0..500.0).contains(&state.altitude_km()));
        assert!((7.0..8.5).contains(&state.velocity_magnitude()));
    }

    #[test]
    fn propagation_changes_position() {
        let propagator = TwoBodyPropagator::new(iss_like_tle()).unwrap();
        let a = propagator.propagate(0.0).unwrap();
        let b = propagator.propagate(1.0).unwrap();
        assert!(TwoBodyPropagator::miss_distance(a, b) > 1.0);
    }

    #[test]
    fn invalid_eccentricity_is_rejected() {
        let mut tle = iss_like_tle();
        tle.eccentricity = 1.1;
        assert_eq!(
            TwoBodyPropagator::new(tle).unwrap_err(),
            PropagationError::InvalidEccentricity
        );
    }
}
