use crate::constants::config::{
    self, BLUE_HUB_X_INCHES, HALF_FIELD_WIDTH_METERS, RED_HUB_X_INCHES,
};
use crate::constants::drivetrain::{
    BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS, BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS,
    FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS, FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS,
    GYRO_OFFSET_UPDATE_RATIO, SWERVE_DRIVE_RATIO, SWERVE_TURN_RATIO,
};
use crate::constants::robotmap::drivetrain_map::{
    BL_DRIVE_ID, BL_ENCODER_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, BR_TURN_ID,
    DRIVETRAIN_CANBUS, FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID,
    FR_TURN_ID, GYRO_ID,
};
use crate::subsystems::localization::Localization;
use crate::subsystems::swerve::kinematics::Kinematics;
use crate::subsystems::vision::Vision;
use frcrs::alliance_station;
use frcrs::ctre::{CanCoder, ControlMode, Pigeon, Talon};
use nalgebra::{Rotation2, Vector2, vector};
use std::f64::consts::PI;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;
use tokio::time::timeout;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{inch, meter};

/// Drivetrain struct.
/// kinematics field interfaces with inverse kinematics functions.
/// motor_encoder_offsets are the absolute positions of the CANCoders on startup. These allow us to start the robot without physically zeroing the wheels.
pub struct Drivetrain {
    pub(in crate::subsystems::swerve) gyro: Pigeon,
    gyro_offset: Angle,
    gyro_set: bool,
    pub limelight_side: Vision,
    pub limelight_front: Vision,

    pub(in crate::subsystems::swerve) kinematics: Kinematics,
    pub localization: Localization,
    last_modules: Vec<(Angle, Angle)>,

    //pub(crate::subsystems::swerve) makes this pub to everything in crate::subsystems::swerve
    fl_encoder: CanCoder,
    pub(in crate::subsystems::swerve) fl_drive: Talon,
    pub(in crate::subsystems::swerve) fl_turn: Talon,

    bl_encoder: CanCoder,
    pub(in crate::subsystems::swerve) bl_drive: Talon,
    pub(in crate::subsystems::swerve) bl_turn: Talon,

    br_encoder: CanCoder,
    pub(in crate::subsystems::swerve) br_drive: Talon,
    pub(in crate::subsystems::swerve) br_turn: Talon,

    fr_encoder: CanCoder,
    pub(in crate::subsystems::swerve) fr_drive: Talon,
    pub(in crate::subsystems::swerve) fr_turn: Talon,
}

pub enum FieldZone {
    BlueTop,
    BlueBottom,
    MiddleTop,
    MiddleBottom,
    RedTop,
    RedBottom,
}

impl Drivetrain {
    /// Returns a new Drivetrain. CAN IDs and CanBus set in constants::robotmap::drivetrain_map
    pub fn new(/*starting_pose: Vector2<Length>, starting_yaw: Angle*/) -> Drivetrain {
        // make the encoders before rest of robot - we need them to get CANCoder offsets
        let fl_encoder = CanCoder::new(FL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let bl_encoder = CanCoder::new(BL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let br_encoder = CanCoder::new(BR_ENCODER_ID, DRIVETRAIN_CANBUS);
        let fr_encoder = CanCoder::new(FR_ENCODER_ID, DRIVETRAIN_CANBUS);

        let limelight_front = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 200)),
            5807,
        ));

        let limelight_side = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        Drivetrain {
            gyro: Pigeon::new(GYRO_ID, Some("can0".to_string())),
            gyro_offset: Angle::new::<degree>(0.0),
            gyro_set: false,
            limelight_front,
            limelight_side,

            kinematics: Kinematics::new(),
            localization: Localization::new(),
            last_modules: Vec::new(),

            fl_encoder,
            fl_drive: Talon::new(FL_DRIVE_ID, DRIVETRAIN_CANBUS),
            fl_turn: Talon::new(FL_TURN_ID, DRIVETRAIN_CANBUS),

            bl_encoder,
            bl_drive: Talon::new(BL_DRIVE_ID, DRIVETRAIN_CANBUS),
            bl_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

            br_encoder,
            br_drive: Talon::new(BR_DRIVE_ID, DRIVETRAIN_CANBUS),
            br_turn: Talon::new(BR_TURN_ID, DRIVETRAIN_CANBUS),

            fr_encoder,
            fr_drive: Talon::new(FR_DRIVE_ID, DRIVETRAIN_CANBUS),
            fr_turn: Talon::new(FR_TURN_ID, DRIVETRAIN_CANBUS),
        }
    }

    /// Stops the drivetrain.
    pub fn stop(&self) {
        self.fl_drive.stop();
        self.fl_turn.stop();

        self.bl_drive.stop();
        self.bl_turn.stop();

        self.br_drive.stop();
        self.br_turn.stop();

        self.fr_drive.stop();
        self.fr_turn.stop();
    }

    fn update_odo(&mut self) {
        let (difs, measured_angles) = self.get_odo_inputs();
        let (pose_vec, yaw_change, translation_error, yaw_error) =
            self.kinematics.odometry(difs, measured_angles);

        self.localization.translation_from_odometry(
            pose_vec,
            yaw_change,
            translation_error,
            yaw_error,
        );
    }
    // TODO: gyro things
    /// updates the limelight values and passes in drivetrain data for fom
    pub async fn update_pose(&mut self) {
        let gyro_angle = Angle::new::<radian>(self.gyro.get_rotation().z);
        let gyro_yaw_error = Angle::new::<degree>(0.2);

        if self.gyro_set {
            self.localization.update_yaw(
                gyro_angle + self.gyro_offset,
                gyro_yaw_error.get::<radian>(),
            );
        }

        //let (pose, _, _, _) = self.localization.get_state();
        //println!("update_pose: current localized botpose: {:?}", pose);

        self.update_odo();
        let front_update_handle =
            timeout(Duration::from_millis(100), self.limelight_front.update());
        let side_update_handle = timeout(Duration::from_millis(100), self.limelight_side.update());

        if front_update_handle.await.is_ok() {
            //println!("entered front update");
            // println!("(unused in code) ll imu yaw: {}", self.limelight_front.status.finalYaw);
            // println!("(unused in code) ll field yaw: {}", self.limelight_front.get_field_yaw().get::<degree>());
            if self.limelight_front.get_botpose().is_some() {
                self.localization.update_pose_from_limelight(
                    self.limelight_front.get_botpose().unwrap(),
                    self.limelight_front.get_field_yaw(),
                    // self.limelight_front.get_location_error(),
                    // self.limelight_front.get_yaw_error(),
                    Vector2::new(Length::new::<meter>(0.0001), Length::new::<meter>(0.0001)),
                    Angle::new::<degree>(0.002),
                );
                let current_offset = self.limelight_front.get_field_yaw() - gyro_angle;
                if self.gyro_set {
                    self.gyro_offset +=
                        GYRO_OFFSET_UPDATE_RATIO * get_angle_difs(self.gyro_offset, current_offset)
                } else {
                    self.gyro_offset = current_offset;
                    self.gyro_set = true;
                }
            }

            // before: self.limelight_front.get_field_yaw()
        }

        if side_update_handle.await.is_ok() {
            //println!("entered side update");
            if self.limelight_side.get_botpose().is_some() {
                self.localization.update_pose_from_limelight(
                    self.limelight_side.get_botpose().unwrap(),
                    self.limelight_side.get_field_yaw(),
                    /*self.limelight_side.get_location_error(),*/
                    Vector2::new(Length::new::<meter>(0.0001), Length::new::<meter>(0.0001)),
                    self.limelight_side.get_yaw_error(),
                );
                let current_offset = self.limelight_side.get_field_yaw() - gyro_angle;
                if self.gyro_set {
                    self.gyro_offset +=
                        GYRO_OFFSET_UPDATE_RATIO * get_angle_difs(self.gyro_offset, current_offset)
                } else {
                    self.gyro_offset = current_offset;
                    self.gyro_set = true;
                }
            }
        }

        // if side_update_handle.await.is_ok() {
        //     println!("entered side update");
        //     println!(
        //         "pigeon gyro: {:?} + 90",
        //         self.gyro.get_rotation().z * PI / 180.0
        //     );
        //     if self.limelight_side.get_botpose_orb().is_some() {
        //         self.localization.update_pose_from_limelight(
        //             self.limelight_side.get_botpose_orb().unwrap(),
        //             Angle::new::<degree>(self.gyro.get_rotation()[(0, 0)] + 90.0),
        //             self.limelight_side.get_location_error(),
        //             self.limelight_side.get_yaw_error(),
        //         );
        //     }
        //
        //     // TODO: implement this
        //     // // 0.002 is how much to trust gyro; estimated noise
        //     let ll_side_yaw = self.limelight_side.get_yaw();
        //     if ll_side_yaw.is_some() {
        //         self.localization.update_yaw(ll_side_yaw.unwrap(), 0.002);
        //     }
        // }
    }

    pub fn set_gyro_offset(&mut self) {
        self.gyro_offset = Angle::new::<radian>(self.gyro.get_rotation().z);
    }

    /// Resets the gyro.
    pub fn reset_heading(&mut self) {
        // self.offset = self.limelight_side.get_field_yaw() - self.limelight_side.get_yaw();
    }

    /// Field-orientate input from the driverstation.
    /// target_transformation is the x and y input from the driverstation put into a vector.
    /// This function rotates the driver's field orientated input to be robot oriented but the same direction.
    pub fn field_orientate(&mut self, target_transformation: Vector2<f64>) -> Vector2<f64> {
        let (_, yaw, _, _) = self.localization.get_state();
        //
        if alliance_station().red() {
            Rotation2::new(-yaw.get::<radian>()) * target_transformation
        } else {
            Rotation2::new(-yaw.get::<radian>() + PI) * target_transformation
        }
        // let yaw = self.get_offset_gyro_yaw();
        // println!("field orient input yaw: {:?}", yaw);
        //
        // if alliance_station().red() {
        //     Rotation2::new(-yaw.get::<radian>() + PI) * target_transformation
        // } else {
        //     Rotation2::new(-yaw.get::<radian>()) * target_transformation
        // }
    }

    /// ## Sets drivetrain motor speeds.
    pub fn set_speeds(&mut self, targets: Vec<(f64, Angle)>) {
        let mut update_turn = false;
        for tuple in &targets {
            if tuple.0.abs() > 0.05 {
                update_turn = true;
            }
        }

        let relative_target_modifers = vec![
            optimize_difs(abs_offset(
                Angle::new::<revolution>(FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.fl_encoder.get_absolute()),
                targets[0].1,
            )),
            optimize_difs(abs_offset(
                Angle::new::<revolution>(BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.bl_encoder.get_absolute()),
                targets[1].1,
            )),
            optimize_difs(abs_offset(
                Angle::new::<revolution>(BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.br_encoder.get_absolute()),
                targets[2].1,
            )),
            optimize_difs(abs_offset(
                Angle::new::<revolution>(FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.fr_encoder.get_absolute()),
                targets[3].1,
            )),
        ];

        self.fl_drive.set(
            ControlMode::Velocity,
            (targets[0].0 * relative_target_modifers[0].1) * SWERVE_DRIVE_RATIO,
        );
        self.bl_drive.set(
            ControlMode::Velocity,
            (targets[1].0 * relative_target_modifers[1].1) * SWERVE_DRIVE_RATIO,
        );
        self.br_drive.set(
            ControlMode::Velocity,
            (targets[2].0 * relative_target_modifers[2].1) * SWERVE_DRIVE_RATIO,
        );
        self.fr_drive.set(
            ControlMode::Velocity,
            (targets[3].0 * relative_target_modifers[3].1) * SWERVE_DRIVE_RATIO,
        );

        if update_turn {
            self.fl_turn.set(
                ControlMode::Position,
                self.fl_turn.get_position()
                    + relative_target_modifers[0].0.get::<revolution>() * SWERVE_TURN_RATIO,
            );
            self.bl_turn.set(
                ControlMode::Position,
                self.bl_turn.get_position()
                    + relative_target_modifers[1].0.get::<revolution>() * SWERVE_TURN_RATIO,
            );
            self.br_turn.set(
                ControlMode::Position,
                self.br_turn.get_position()
                    + relative_target_modifers[2].0.get::<revolution>() * SWERVE_TURN_RATIO,
            );
            self.fr_turn.set(
                ControlMode::Position,
                self.fr_turn.get_position()
                    + relative_target_modifers[3].0.get::<revolution>() * SWERVE_TURN_RATIO,
            );
        }
    }

    /// Control the drivetrain.
    /// x, y, and rotation are driverstation inputs.
    pub fn control_drivetrain(&mut self, x: f64, y: f64, rotation: f64) {
        let target_transformation = match config::FIELD_ORIENTED {
            true => self.field_orientate(vector![x, y]),
            false => vector![x, y],
        };

        //println!("target_transformation: {:?}", target_transformation);

        let targets = self.kinematics.get_targets(target_transformation, rotation);

        self.set_speeds(targets);
    }

    /// Drive revolution differences, turn angle diff
    fn get_odo_inputs(&mut self) -> (Vec<(Angle, Angle)>, Vec<Angle>) {
        if self.last_modules.is_empty() {
            self.last_modules = vec![
                (
                    Angle::new::<revolution>(self.fl_drive.get_position() / SWERVE_DRIVE_RATIO),
                    Angle::new::<revolution>(self.fl_turn.get_position()) / SWERVE_TURN_RATIO,
                ),
                (
                    Angle::new::<revolution>(self.bl_drive.get_position() / SWERVE_DRIVE_RATIO),
                    Angle::new::<revolution>(self.bl_turn.get_position()) / SWERVE_TURN_RATIO,
                ),
                (
                    Angle::new::<revolution>(self.br_drive.get_position() / SWERVE_DRIVE_RATIO),
                    Angle::new::<revolution>(self.br_turn.get_position()) / SWERVE_TURN_RATIO,
                ),
                (
                    Angle::new::<revolution>(self.fr_drive.get_position() / SWERVE_DRIVE_RATIO),
                    Angle::new::<revolution>(self.fr_turn.get_position()) / SWERVE_TURN_RATIO,
                ),
            ];
        }

        let measured = vec![
            (
                Angle::new::<revolution>(self.fl_drive.get_position() / SWERVE_DRIVE_RATIO),
                Angle::new::<revolution>(self.fl_turn.get_position()) / SWERVE_TURN_RATIO,
            ),
            (
                Angle::new::<revolution>(self.bl_drive.get_position() / SWERVE_DRIVE_RATIO),
                Angle::new::<revolution>(self.bl_turn.get_position()) / SWERVE_TURN_RATIO,
            ),
            (
                Angle::new::<revolution>(self.br_drive.get_position() / SWERVE_DRIVE_RATIO),
                Angle::new::<revolution>(self.br_turn.get_position()) / SWERVE_TURN_RATIO,
            ),
            (
                Angle::new::<revolution>(self.fr_drive.get_position() / SWERVE_DRIVE_RATIO),
                Angle::new::<revolution>(self.fr_turn.get_position()) / SWERVE_TURN_RATIO,
            ),
        ];
        let differences = vec![
            (
                get_angle_difs(self.last_modules[0].0, measured[0].0),
                get_angle_difs(self.last_modules[0].1, measured[0].1),
            ),
            (
                get_angle_difs(self.last_modules[1].0, measured[1].0),
                get_angle_difs(self.last_modules[1].1, measured[1].1),
            ),
            (
                get_angle_difs(self.last_modules[2].0, measured[2].0),
                get_angle_difs(self.last_modules[2].1, measured[2].1),
            ),
            (
                get_angle_difs(self.last_modules[3].0, measured[3].0),
                get_angle_difs(self.last_modules[3].1, measured[3].1),
            ),
        ];

        let measured_angles = vec![
            -abs_offset(
                Angle::new::<revolution>(FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.fl_encoder.get_absolute()),
                Angle::new::<degree>(0.0),
            ),
            -abs_offset(
                Angle::new::<revolution>(BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.bl_encoder.get_absolute()),
                Angle::new::<degree>(0.0),
            ),
            -abs_offset(
                Angle::new::<revolution>(BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.br_encoder.get_absolute()),
                Angle::new::<degree>(0.0),
            ),
            -abs_offset(
                Angle::new::<revolution>(FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS),
                Angle::new::<revolution>(self.fr_encoder.get_absolute()),
                Angle::new::<degree>(0.0),
            ),
        ];
        self.last_modules = measured;
        (differences, measured_angles)
    }

    pub fn get_zone(&self) -> FieldZone {
        let (pose, _, _, _) = self.localization.get_state();
        if pose.y.get::<meter>() < HALF_FIELD_WIDTH_METERS {
            if pose.x.get::<inch>() < BLUE_HUB_X_INCHES {
                FieldZone::BlueBottom
            } else if pose.x.get::<inch>() < RED_HUB_X_INCHES {
                FieldZone::MiddleBottom
            } else {
                FieldZone::RedBottom
            }
        } else {
            if pose.x.get::<inch>() < BLUE_HUB_X_INCHES {
                FieldZone::BlueTop
            } else if pose.x.get::<inch>() < RED_HUB_X_INCHES {
                FieldZone::MiddleTop
            } else {
                FieldZone::RedTop
            }
        }
    }
}

pub fn get_angle_difs(from: Angle, to: Angle) -> Angle {
    Angle::new::<radian>(f64::atan2(
        f64::sin(to.get::<radian>() - from.get::<radian>()),
        f64::cos(to.get::<radian>() - from.get::<radian>()),
    ))
}

fn wrap_angle(angle: Angle) -> Angle {
    Angle::new::<radian>(f64::atan2(
        f64::sin(angle.get::<radian>()),
        f64::cos(angle.get::<radian>()),
    ))
}

/// Angle is difference, f64 is -1 or 1 for if module is reversed
fn optimize_difs(dif: Angle) -> (Angle, f64) {
    let dif_f64 = dif.get::<degree>();

    if dif_f64 > 90.0 {
        (Angle::new::<degree>(-180.0 + dif_f64), -1.0)
    } else if dif_f64 < -90.0 {
        (Angle::new::<degree>(180.0 + dif_f64), -1.0)
    } else {
        (Angle::new::<degree>(dif_f64), 1.0)
    }
}

fn abs_offset(abs_zero_position: Angle, abs_reading: Angle, target: Angle) -> Angle {
    // current_angle + get_angle_difs(get_angle_difs(abs_zero_position, abs_reading), target)
    wrap_angle(target - (abs_reading - abs_zero_position))
}

#[cfg(test)]
mod drivetrain_tests {
    use super::*;
    use float_cmp::assert_approx_eq;
    use std::panic;

    // Measured is 0-360
    // Target is -180 to 180
    #[test]
    fn get_difs_test() {
        // (measured, target)
        let inputs = vec![
            (Angle::new::<degree>(0.0), Angle::new::<degree>(180.0)), // 1
            (Angle::new::<degree>(90.0), Angle::new::<degree>(180.0)), // 2
            (Angle::new::<degree>(180.0), Angle::new::<degree>(180.0)), // 3
            (Angle::new::<degree>(270.0), Angle::new::<degree>(180.0)), // 4
            (Angle::new::<degree>(360.0), Angle::new::<degree>(180.0)), // 5
            (Angle::new::<degree>(0.0), Angle::new::<degree>(-180.0)), // 6
            (Angle::new::<degree>(90.0), Angle::new::<degree>(-180.0)), // 7
            (Angle::new::<degree>(180.0), Angle::new::<degree>(-180.0)), // 8
            (Angle::new::<degree>(270.0), Angle::new::<degree>(-180.0)), // 9
            (Angle::new::<degree>(360.0), Angle::new::<degree>(-180.0)), // 10
            (Angle::new::<degree>(350.0), Angle::new::<degree>(10.0)), // 11
            (Angle::new::<degree>(10.0), Angle::new::<degree>(-10.0)), // 12
        ];

        let expected = vec![
            Angle::new::<degree>(180.0),  // 1
            Angle::new::<degree>(90.0),   // 2
            Angle::new::<degree>(0.0),    // 3
            Angle::new::<degree>(-90.0),  // 4
            Angle::new::<degree>(-180.0), // 5
            Angle::new::<degree>(-180.0), // 6
            Angle::new::<degree>(90.0),   // 7
            Angle::new::<degree>(0.0),    // 8
            Angle::new::<degree>(-90.0),  // 9
            Angle::new::<degree>(-180.0), // 10
            Angle::new::<degree>(20.0),   // 11
            Angle::new::<degree>(-20.0),  // 12
        ];

        let mut results = Vec::new();

        for tuple in inputs {
            results.push(get_angle_difs(tuple.0, tuple.1));
        }

        let mut i = 1.0;
        for result in results.clone() {
            println!("result {}: {}", i, result.get::<degree>());
            i += 1.0;
        }

        for tuple in expected.iter().zip(results.iter()) {
            assert_approx_eq!(
                f64,
                tuple.0.get::<degree>(),
                tuple.1.get::<degree>(),
                epsilon = 0.001
            );
        }
    }

    #[test]
    fn optimize_difs_test() {
        let inputs = vec![
            Angle::new::<degree>(0.0),    // 1
            Angle::new::<degree>(45.0),   // 2
            Angle::new::<degree>(90.0),   // 3
            Angle::new::<degree>(135.0),  // 4
            Angle::new::<degree>(180.0),  // 5
            Angle::new::<degree>(-45.0),  // 6
            Angle::new::<degree>(-90.0),  // 7
            Angle::new::<degree>(-135.0), // 8
            Angle::new::<degree>(-180.0), // 9
        ];

        let expected = vec![
            (Angle::new::<degree>(0.0), 1.0),    // 1
            (Angle::new::<degree>(45.0), 1.0),   // 2
            (Angle::new::<degree>(90.0), 1.0),   // 3
            (Angle::new::<degree>(-45.0), -1.0), // 4
            (Angle::new::<degree>(0.0), -1.0),   // 5
            (Angle::new::<degree>(-45.0), 1.0),  // 6
            (Angle::new::<degree>(-90.0), 1.0),  // 7
            (Angle::new::<degree>(45.0), -1.0),  // 8
            (Angle::new::<degree>(0.0), -1.0),   // 9
        ];

        let mut results = Vec::new();

        for input in inputs {
            results.push(optimize_difs(input));
        }

        let mut i = 1.0;
        for result in results.clone() {
            println!("result {}: ({}, {})", i, result.0.get::<degree>(), result.1);
            i += 1.0;
        }

        for tuple in expected.iter().zip(results.iter()) {
            assert_approx_eq!(f64, tuple.0.1, tuple.1.1);
            assert_approx_eq!(f64, tuple.0.0.get::<degree>(), tuple.1.0.get::<degree>());
        }
    }
}
