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