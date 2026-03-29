use crate::{Ferris, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND, update_drivetrain_telemetry};
use frcrs::deadzone;
use uom::si::f64::Length;
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
            let (linear_velocity, angular_velocity) = drivetrain.localization.get_velocities();
            update_drivetrain_telemetry(&pose, &linear_velocity, &angular_velocity).await;
        }
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            let max_dt_speed = Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND);

            let deadzone_output_range = 0.0..1.0;
            let deadzone_input_range = 0.05..1.0;
            drivetrain.control_drivetrain(
                deadzone(
                    -ferris.controllers.left_drive.get_x(),
                    &deadzone_input_range,
                    &deadzone_output_range,
                ),
                deadzone(
                    ferris.controllers.left_drive.get_y(),
                    &deadzone_input_range,
                    &deadzone_output_range,
                ),
                deadzone(
                    ferris.controllers.right_drive.get_z(),
                    &deadzone_input_range,
                    &deadzone_output_range,
                ),
                max_dt_speed,
            );
            if ferris.controllers.right_drive.get(1) {
                drivetrain.reset_heading();
            }

            if ferris.controllers.right_drive.get(4) {
                drivetrain.set_gyro_offset();
            }
        }
    }
}
