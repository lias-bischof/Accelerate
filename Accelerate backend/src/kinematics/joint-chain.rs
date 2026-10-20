use nalgebra::{Isometry3, Matrix6, Vector3, Vector6};
use crate::kinematics::joint::Joint;
use std::f64::consts::TAU;

const ITERATIONS: usize = 200;
const DAMPING: f64 = 0.01;
const POSITION_TOLERANCE: f64 = 1e-5;
const ROTATION_TOLERANCE: f64 = 1e-4 * 0.25;
const INITIAL_STEP: f64 = 1.0;
const STEP_REDUCTION: f64 = 0.5;
const BACKTRACKING: usize = 16;
const STEP_LIMITATION: f64 = 0.25;

pub struct Chain(pub [Joint; 6]);

impl Chain {

    fn reachable(&self, target: &Isometry3<f64>) -> bool {
        let mut length = 0.0;
        for joint in &self.0 {
            length += joint.a.hypot(joint.d);
        }
        target.translation.vector.norm() <= length + POSITION_TOLERANCE
    }

    pub fn pose(&self, angles: &[f64; 6]) -> Isometry3<f64> {
        let mut pose = Isometry3::identity();
        for i in 0..6 {
            pose *= self.0[i].transform(angles[i]);
        }
        return pose
    }

    pub fn positions(&self, angles: &[f64; 6]) -> [Vector3<f64>; 6] {
        let mut pose = Isometry3::identity();
        std::array::from_fn(|i| {
            pose *= self.0[i].transform(angles[i]);
            return pose.translation.vector
        })
    }

    pub fn jacobian(&self, angles: &[f64; 6]) -> Matrix6<f64> {
        let end = self.pose(angles).translation.vector;
        let mut pose = Isometry3::identity();
        let mut jacobian = Matrix6::zeros();
        for i in 0..6 {
            let axis = pose.rotation * Vector3::z();
            let linear = axis.cross(&(end - pose.translation.vector));
            jacobian.fixed_view_mut::<3, 1>(0, i).copy_from(&linear);
            jacobian.fixed_view_mut::<3, 1>(3, i).copy_from(&(axis * 0.25));
            pose *= self.0[i].transform(angles[i]);
        }
        return jacobian
    }

    pub fn inverse(&self, target: &Isometry3<f64>, angles: &[f64; 6]) -> Option<[f64; 6]> {
        let initial = *angles;
        if !target.to_homogeneous().iter().all(|value| value.is_finite())
            || self.0.iter().zip(angles).any(|(joint, angle)| {
                !joint.θmin.is_finite() || !joint.θmax.is_finite() || joint.θmin > joint.θmax
                    || !angle.is_finite() || *angle < joint.θmin || *angle > joint.θmax
            }) {
            return None;
        }
        let mut angles = Vector6::from(*angles);
        if !self.reachable(target) {
            return None;
        }
        for _ in 0..ITERATIONS {
            let pose = self.pose(angles.as_ref());
            let error = Chain::error(&pose, target);
            if error.fixed_rows::<3>(0).norm() < POSITION_TOLERANCE
                && error.fixed_rows::<3>(3).norm() < ROTATION_TOLERANCE {
                return Some(std::array::from_fn(|i| {
                    let joint = &self.0[i];
                    let turns = ((initial[i] - angles[i]) / TAU).round().clamp(
                        ((joint.θmin - angles[i]) / TAU).ceil(),
                        ((joint.θmax - angles[i]) / TAU).floor()
                    );
                    return (angles[i] + turns * TAU).clamp(joint.θmin, joint.θmax)
                }));
            }
            let direction = self.direction(angles.as_ref(), &error)?;
            let cost = error.norm();
            let mut step = INITIAL_STEP;
            let mut next = None;
            for _ in 0..BACKTRACKING {
                let candidate = Vector6::from_fn(|i, _| {
                    (angles[i] + step * direction[i]).clamp(self.0[i].θmin, self.0[i].θmax)
                });
                let pose = self.pose(candidate.as_ref());
                let error = Chain::error(&pose, target);
                if error.norm() < cost {
                    next = Some(candidate);
                    break;
                }
                step *= STEP_REDUCTION;
            }
            angles = next?;
        }
        return None;
    }

    fn direction(&self, angles: &[f64; 6], error: &Vector6<f64>) -> Option<Vector6<f64>> {
        let mut jacobian = self.jacobian(angles);
        let mut direction = Vector6::zeros();
        for _ in 0..=6 {
            let system = jacobian * jacobian.transpose() + Matrix6::identity() * DAMPING.powi(2);
            let correction = system.cholesky()?.solve(error);
            direction = jacobian.transpose() * correction;
            let mut blocked = false;
            for i in 0..6 {
                let joint = &self.0[i];
                if (angles[i] <= joint.θmin && direction[i] < 0.0)
                    || (angles[i] >= joint.θmax && direction[i] > 0.0) {
                    jacobian.column_mut(i).fill(0.0);
                    blocked = true;
                }
            }
            if !blocked { break; }
        }
        let largest = direction.amax();
        if largest > STEP_LIMITATION {
            direction *= STEP_LIMITATION / largest;
        }
        Some(direction)
    }

    fn error(pose: &Isometry3<f64>, target: &Isometry3<f64>) -> Vector6<f64> {
        let position = target.translation.vector - pose.translation.vector;
        let rotation = (target.rotation * pose.rotation.inverse()).scaled_axis();
        Vector6::from([position.x, position.y, position.z,
            rotation.x * 0.25, rotation.y * 0.25, rotation.z * 0.25])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn chain() -> Chain {
        Chain([
            [0.0, 0.15, FRAC_PI_2], [0.30, 0.0, 0.0], [0.25, 0.0, 0.0],
            [0.0, 0.10, FRAC_PI_2], [0.0, 0.10, -FRAC_PI_2], [0.0, 0.08, 0.0],
        ].map(|[a, d, alpha]| Joint { a, d, α: alpha, Δϑ: 0.0, θmin: -PI, θmax: PI }))
    }

    #[test]
    fn solves_with_individual_limits_and_a_fixed_joint() {
        let mut chain = chain();
        let desired = [0.2, 0.4, -0.6, 0.3, -0.2, 0.1];
        for (i, joint) in chain.0.iter_mut().enumerate() {
            joint.θmin = desired[i] - 0.15;
            joint.θmax = desired[i] + 0.15;
        }
        chain.0[0].θmin = desired[0];
        chain.0[0].θmax = desired[0];
        let mut start = desired.map(|angle| angle + 0.05);
        start[0] = desired[0];
        let target = chain.pose(&desired);
        let solution = chain.inverse(&target, &start).unwrap();
        assert!(solution.iter().zip(&chain.0).all(|(angle, joint)| *angle >= joint.θmin && *angle <= joint.θmax));
        let error = Chain::error(&chain.pose(&solution), &target);
        assert!(error.fixed_rows::<3>(0).norm() < POSITION_TOLERANCE);
        assert!(error.fixed_rows::<3>(3).norm() < ROTATION_TOLERANCE);
    }

    #[test]
    fn rejects_a_target_when_limits_prevent_reaching_it() {
        let mut chain = chain();
        let target = chain.pose(&[0.2; 6]);
        for joint in &mut chain.0 { joint.θmin = 0.0; joint.θmax = 0.0; }
        assert!(chain.inverse(&target, &[0.0; 6]).is_none());
    }

    #[test]
    fn rejects_invalid_limits_and_starting_angles() {
        let mut chain = chain();
        let target = chain.pose(&[0.0; 6]);
        assert!(chain.inverse(&target, &[TAU; 6]).is_none());
        chain.0[0].θmin = 1.0;
        chain.0[0].θmax = -1.0;
        assert!(chain.inverse(&target, &[0.0; 6]).is_none());
        chain.0[0].θmin = f64::NAN;
        assert!(chain.inverse(&target, &[0.0; 6]).is_none());
    }

    #[test]
    fn stays_near_current_angles_for_multiple_turn_joints() {
        let mut chain = chain();
        for joint in &mut chain.0 { joint.θmin = -3.0 * TAU; joint.θmax = 3.0 * TAU; }
        let desired = [0.2, 0.4, -0.6, 0.3, -0.2, 0.1];
        let start = desired.map(|angle| angle + TAU + 0.01);
        let solution = chain.inverse(&chain.pose(&desired), &start).unwrap();
        assert!(solution.iter().zip(start).all(|(angle, previous)| (angle - previous).abs() < PI));
    }

    #[test]
    fn repeated_target_needs_no_additional_movement_with_limits() {
        let chain = chain();
        let mut target = chain.pose(&[0.0; 6]);
        target.translation.vector = Vector3::new(0.5, -0.18, 0.05);
        let first = chain.inverse(&target, &[0.0; 6]).unwrap();
        let second = chain.inverse(&target, &first).unwrap();
        assert!(first.iter().zip(second).all(|(a, b)| (a - b).abs() < 1e-10));
    }
}
