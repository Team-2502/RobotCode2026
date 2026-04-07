use crate::Ferris;
use crate::constants::config::{
    MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND,
    SLOW_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND, SLOW_DRIVETRAIN_SPEED_METERS_PER_SECOND,
};
use frcrs::{alliance_station, deadzone};
use std::f64::consts::PI;
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;
pub struct Swerve {}

impl Swerve {
    pub fn new() -> Swerve {
        Swerve {}
    }

    pub async fn update(&self, ferris: &mut Ferris) {
        if ferris.controllers.left_drive.get(3) {
            if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
                drivetrain.set_gyro_offset();
            }
        }
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
            // ds coordinate system is +y is downfield, actual is +x for downfield
            // flip x and y
            let deadzone_output = 0.0..1.0;
            let deadzone_input = 0.05..1.0;
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
                ferris.controllers.left_drive.get_z(),
                &deadzone_input,
                &deadzone_output,
            );

            // setup inputs into polar conv, field orient
            let input_magnitude = (deadzoned_x * deadzoned_x + deadzoned_y * deadzoned_y).sqrt();
            let theta = Angle::new::<radian>(deadzoned_y.atan2(deadzoned_x));
            let mut field_theta = theta + drivetrain.get_dt_heading();

            let magnitude = Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND);

            if ferris.controllers.left_drive.get(2) {
                // snap heading to 90 degree increments
                let heading = drivetrain.get_dt_heading().get::<radian>();
                drivetrain.turn_to(
                    field_theta,
                    Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND) * input_magnitude,
                    Angle::new::<radian>((heading / (PI / 2.0)).round() * (PI / 2.0)),
                );
            } else if ferris.controllers.left_drive.get(4) {
                // snap direction to 90 degree inputs
                let rotation_rate = deadzoned_z
                    * Angle::new::<radian>(MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND);

                field_theta = Angle::new::<radian>(
                    (field_theta.get::<radian>() / (PI / 2.0)).round() * (PI / 2.0),
                );

                drivetrain.control_drivetrain(
                    field_theta,
                    Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND) * input_magnitude,
                    rotation_rate,
                );
            } else if ferris.controllers.left_drive.get(1) {
                // slow
                let rotation_rate = deadzoned_z
                    * Angle::new::<radian>(SLOW_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND);
                drivetrain.control_drivetrain(
                    field_theta,
                    Length::new::<meter>(SLOW_DRIVETRAIN_SPEED_METERS_PER_SECOND) * input_magnitude,
                    rotation_rate,
                );
            } else {
                // standard behaviour
                let rotation_rate = deadzoned_z
                    * Angle::new::<radian>(MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND);
                drivetrain.control_drivetrain(
                    field_theta,
                    magnitude * input_magnitude,
                    rotation_rate,
                );
            }
        }
    }
}
