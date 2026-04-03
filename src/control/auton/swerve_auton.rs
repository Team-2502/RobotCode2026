use crate::control::fueler::{Target, TargetingMode};
use crate::{Ferris, update_drivetrain_telemetry};
pub struct AutonSwerve {}
use frcrs::drive;
use uom::si::f64::{Angle, Length};

impl AutonSwerve {
    pub fn new() -> AutonSwerve {
        AutonSwerve {}
    }

    pub async fn update(&self, ferris: &Ferris) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.update_pose().await;
            let pose = drivetrain.localization.get_state();
            update_drivetrain_telemetry(&pose).await;
        }
    }

    pub fn control_drivetrain(
        &self,
        ferris: &mut Ferris,
        theta: Angle,
        magnitude: Length,
        rot: Angle,
    ) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.control_drivetrain(theta, magnitude, rot);
        }
    }

    pub fn turn_to(&self, ferris: &mut Ferris, theta: Angle, magnitude: Length, desired: Angle) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.turn_to(theta, magnitude, desired);
        }
    }
}
