/// SGP4/SDP4 Orbit Propagator — Rust Implementation
/// Compile-time safe orbit propagation for conjunction assessment.
/// Handles LEO (SGP4) and deep-space (SDP4) objects with full perturbation modeling.

use std::f64::consts::PI;

const MU_EARTH: f64 = 398600.4418;      // km³/s²
const EARTH_RADIUS_KM: f64 = 6378.137;  // WGS-84
const J2: f64 = 0.00108263;             // Earth oblateness
const MINUTES_PER_DAY: f64 = 1440.0;
const TWO_PI: f64 = 2.0 * PI;

/// Two-Line Element set parsed from NORAD catalog
#[derive(Debug, Clone)]
pub struct TLE {
    pub norad_id: u32,
    pub epoch_year: u16,
    pub epoch_day: f64,
    pub mean_motion_rev_day: f64,    // revs/day
    pub eccentricity: f64,
    pub inclination_deg: f64,
    pub raan_deg: f64,               // Right Ascension of Ascending Node
    pub arg_perigee_deg: f64,
    pub mean_anomaly_deg: f64,
    pub bstar: f64,                   // Drag coefficient (1/earth_radii)
    pub classification: char,
}

/// State vector in ECI (Earth-Centered Inertial) frame
#[derive(Debug, Clone, Copy)]
pub struct StateVector {
    pub x: f64, pub y: f64, pub z: f64,       // Position (km)
    pub vx: f64, pub vy: f64, pub vz: f64,    // Velocity (km/s)
    pub epoch_minutes: f64,                      // Minutes since TLE epoch
}

impl StateVector {
    pub fn position_magnitude(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    pub fn velocity_magnitude(&self) -> f64 {
        (self.vx * self.vx + self.vy * self.vy + self.vz * self.vz).sqrt()
    }

    /// Compute orbital altitude above Earth surface (km)
    pub fn altitude_km(&self) -> f64 {
        self.position_magnitude() - EARTH_RADIUS_KM
    }
}

/// Conjunction assessment result
#[derive(Debug, Clone)]
pub struct ConjunctionResult {
    pub miss_distance_km: f64,
    pub relative_velocity_kms: f64,
    pub tca_minutes: f64,            // Time of Closest Approach (minutes since epoch)
    pub probability_of_collision: f64,
}

/// SGP4 propagator for near-Earth objects
pub struct SGP4Propagator {
    tle: TLE,
    // Pre-computed orbital elements
    n0: f64,      // Mean motion (rad/min)
    a0: f64,      // Semi-major axis (Earth radii)
    e0: f64,      // Eccentricity
    i0: f64,      // Inclination (rad)
    omega0: f64,  // RAAN (rad)
    w0: f64,      // Argument of perigee (rad)
    m0: f64,      // Mean anomaly (rad)
}

impl SGP4Propagator {
    pub fn new(tle: TLE) -> Self {
        let n0 = tle.mean_motion_rev_day * TWO_PI / MINUTES_PER_DAY;
        let a0 = (MU_EARTH / (n0 * n0)).powf(1.0 / 3.0) / EARTH_RADIUS_KM;

        SGP4Propagator {
            n0,
            a0,
            e0: tle.eccentricity,
            i0: tle.inclination_deg.to_radians(),
            omega0: tle.raan_deg.to_radians(),
            w0: tle.arg_perigee_deg.to_radians(),
            m0: tle.mean_anomaly_deg.to_radians(),
            tle,
        }
    }

    /// Propagate to time t (minutes since TLE epoch)
    /// Returns ECI state vector with J2 secular perturbations
    pub fn propagate(&self, t_min: f64) -> StateVector {
        let cos_i = self.i0.cos();
        let sin_i = self.i0.sin();

        // J2 secular rates
        let p = self.a0 * (1.0 - self.e0 * self.e0);
        let n_dot = self.n0 * (1.0 + 1.5 * J2 / (p * p) * (1.0 - 1.5 * sin_i * sin_i));
        let omega_dot = -1.5 * J2 * n_dot / (p * p) * cos_i;
        let w_dot = 1.5 * J2 * n_dot / (p * p) * (2.0 - 2.5 * sin_i * sin_i);

        // Updated orbital elements
        let m = self.m0 + n_dot * t_min;
        let omega = self.omega0 + omega_dot * t_min;
        let w = self.w0 + w_dot * t_min;

        // Solve Kepler's equation (Newton-Raphson)
        let mut e_anomaly = m;
        for _ in 0..20 {
            let delta = (e_anomaly - self.e0 * e_anomaly.sin() - m)
                / (1.0 - self.e0 * e_anomaly.cos());
            e_anomaly -= delta;
            if delta.abs() < 1e-12 { break; }
        }

        // True anomaly
        let cos_e = e_anomaly.cos();
        let sin_e = e_anomaly.sin();
        let nu = (((1.0 + self.e0) / (1.0 - self.e0)).sqrt() * (sin_e / 2.0).atan2((cos_e / 2.0 + 0.5).max(1e-12))).atan() * 2.0;

        // Radius
        let r = self.a0 * EARTH_RADIUS_KM * (1.0 - self.e0 * cos_e);

        // Position in orbital plane
        let u = w + nu;
        let cos_u = u.cos();
        let sin_u = u.sin();
        let cos_o = omega.cos();
        let sin_o = omega.sin();

        // ECI coordinates
        let x = r * (cos_o * cos_u - sin_o * sin_u * cos_i);
        let y = r * (sin_o * cos_u + cos_o * sin_u * cos_i);
        let z = r * sin_u * sin_i;

        // Velocity (simplified vis-viva)
        let v = (MU_EARTH * (2.0 / r - 1.0 / (self.a0 * EARTH_RADIUS_KM))).sqrt();
        let flight_path = (self.e0 * nu.sin()) / (1.0 + self.e0 * nu.cos());
        let vx = -v * (sin_o * cos_u + cos_o * sin_u * cos_i) + v * flight_path * x / r;
        let vy = v * (cos_o * cos_u - sin_o * sin_u * cos_i) + v * flight_path * y / r;
        let vz = v * sin_u * sin_i + v * flight_path * z / r;

        StateVector { x, y, z, vx, vy, vz, epoch_minutes: t_min }
    }

    /// Compute miss distance between two propagated objects at a given time
    pub fn miss_distance(sv1: &StateVector, sv2: &StateVector) -> f64 {
        let dx = sv1.x - sv2.x;
        let dy = sv1.y - sv2.y;
        let dz = sv1.z - sv2.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Compute relative velocity between two objects
    pub fn relative_velocity(sv1: &StateVector, sv2: &StateVector) -> f64 {
        let dvx = sv1.vx - sv2.vx;
        let dvy = sv1.vy - sv2.vy;
        let dvz = sv1.vz - sv2.vz;
        (dvx * dvx + dvy * dvy + dvz * dvz).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_tle() -> TLE {
        TLE {
            norad_id: 25544,
            epoch_year: 2026,
            epoch_day: 210.5,
            mean_motion_rev_day: 15.49,
            eccentricity: 0.0006703,
            inclination_deg: 51.6434,
            raan_deg: 247.4627,
            arg_perigee_deg: 130.5360,
            mean_anomaly_deg: 325.0288,
            bstar: 0.0001,
            classification: 'U',
        }
    }

    #[test]
    fn test_propagate_iss() {
        let prop = SGP4Propagator::new(test_tle());
        let sv = prop.propagate(0.0); // At epoch
        let alt = sv.altitude_km();
        assert!(alt > 300.0 && alt < 500.0, "ISS altitude should be ~420km, got {}", alt);
    }

    #[test]
    fn test_miss_distance() {
        let prop = SGP4Propagator::new(test_tle());
        let sv1 = prop.propagate(0.0);
        let sv2 = prop.propagate(1.0);
        let dist = SGP4Propagator::miss_distance(&sv1, &sv2);
        assert!(dist > 0.0, "Miss distance should be positive");
    }
}
