use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

pub struct Joint {
    pub a: f64,
    pub d: f64,
    pub α: f64,
    pub Δϑ: f64,
    pub θmin: f64,
    pub θmax: f64,
}

impl Joint {

    pub fn transform(&self, angle: f64) -> Isometry3<f64> {
        let ϑ = angle + self.Δϑ;
        let position = Translation3::new(self.a * ϑ.cos(), self.a * ϑ.sin(), self.d);
        let rotation = UnitQuaternion::from_euler_angles(self.α, 0.0, ϑ);
        return Isometry3::from_parts(position, rotation)
    }
}
