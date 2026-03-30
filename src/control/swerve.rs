use crate::constants::config::MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND;
use crate::control::fueler::{Target, TargetingMode};
use crate::{
    Ferris, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND, get_drivetrain_max_speed,
    update_drivetrain_telemetry,
};
use frcrs::{alliance_station, deadzone};
use std::f64::consts::PI;
use uom::si::angle::{radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

pub struct Swerve {}

impl Swerve {
    pub fn new() -> Swerve {
        Swerve {}
    }

    pub async fn update(&self, ferris: &Ferris) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.update_pose().await;
            let pose = drivetrain.localization.get_state();
            update_drivetrain_telemetry(&pose).await;
        }
    }

    pub fn act(&mut self, ferris: &mut Ferris, target: Target, mode: TargetingMode) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            let deadzone_output = 0.0..1.0;
            let deadzone_input = 0.05..1.0;

            // ds coordinate system is +y is downfield, actual is +x for downfield
            // flip x and y
            let deadzoned_x = deadzone(
                -ferris.controllers.left_drive.get_y(),
                &deadzone_input,
                &deadzone_output,
            );
            let deadzoned_y = deadzone(
                -ferris.controllers.left_drive.get_x(),
                &deadzone_input,
                &deadzone_output,
            );
            let deadzoned_z = deadzone(
                ferris.controllers.right_drive.get_z(),
                &deadzone_input,
                &deadzone_output,
            );

            let pose = drivetrain.localization.get_state();
            let input_magnitude = (deadzoned_x * deadzoned_x + deadzoned_y * deadzoned_y).sqrt();

            let theta = Angle::new::<radian>(deadzoned_y.atan2(deadzoned_x));
            let field_theta = if alliance_station().red() {
                theta + Angle::new::<radian>(PI)
            } else {
                theta
            };

            let magnitude = if mode == TargetingMode::Track {
                get_drivetrain_max_speed(&pose, field_theta, target.target_location)
            } else {
                Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND)
            };

            drivetrain.control_drivetrain(
                field_theta,
                magnitude * input_magnitude,
                deadzoned_z * Angle::new::<revolution>(MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND),
            );
        }
    }
}
