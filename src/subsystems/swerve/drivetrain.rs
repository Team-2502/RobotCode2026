use crate::constants::config;
use crate::constants::drivetrain::{
    BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS, BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS,
    FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS, FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS, SWERVE_TURN_RATIO,
};
use crate::constants::robotmap::drivetrain_map::{
    BL_DRIVE_ID, BL_ENCODER_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, BR_TURN_ID,
    DRIVETRAIN_CANBUS, FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID,
    FR_TURN_ID, GYRO_ID,
};
use crate::subsystems::swerve::kinematics::Kinematics;
use crate::subsystems::swerve::odometry::{Odometry, RobotPoseEstimate};
use crate::subsystems::vision::Vision;
use frcrs::Robot;
use frcrs::ctre::{CanCoder, ControlMode, Pigeon, Talon};
use frcrs::telemetry::Telemetry;
use nalgebra::{Rotation2, Vector2, vector};
use std::f64::consts::PI;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::path::Ancestors;
use std::time::Duration;
use tokio::time::timeout;
use uom::si::angle::{degree, radian, revolution};
use uom::si::angular_velocity::radian_per_second;
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::length::meter;
use uom::si::quantities::AngularVelocity;

/// Drivetrain struct.
/// kinematics field interfaces with inverse kinematics functions.
/// motor_encoder_offsets are the absolute positions of the CANCoders on startup. These allow us to start the robot without physically zeroing the wheels.
pub struct Drivetrain {
    kinematics: Kinematics,
    pub(in crate::subsystems::swerve) odometry: Odometry,
    //pub(in crate::subsystems::swerve) gyro: Pigeon,
    pub limelight: Vision,
    pub velocity: Vector2<f64>,
    pub angular_velocity: f64,

    pub yaw: Angle,
    pub offset: Angle,

    motor_encoder_offsets: [Angle; 4],

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

impl Drivetrain {
    /// Returns a new Drivetrain. CAN IDs and CanBus set in constants::robotmap::drivetrain_map
    pub fn new(starting_pose: RobotPoseEstimate) -> Drivetrain {
        // make the encoders before rest of robot - we need them to get CANCoder offsets
        let fl_encoder = CanCoder::new(FL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let bl_encoder = CanCoder::new(BL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let br_encoder = CanCoder::new(BR_ENCODER_ID, DRIVETRAIN_CANBUS);
        let fr_encoder = CanCoder::new(FR_ENCODER_ID, DRIVETRAIN_CANBUS);

        let limelight = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        let velocity = Vector2::new(0.0, 0.0);
        let angular_velocity = 0.0;

        // .get_absolute returns the CANCoder's rotation from -1 to 1

        let motor_encoder_offsets = [
            Angle::new::<revolution>(fl_encoder.get_absolute()),
            Angle::new::<revolution>(bl_encoder.get_absolute()),
            Angle::new::<revolution>(br_encoder.get_absolute()),
            Angle::new::<revolution>(fr_encoder.get_absolute()),
        ];

        Drivetrain {
            kinematics: Kinematics::new(),
            odometry: Odometry::new(starting_pose),
            //gyro: Pigeon::new(GYRO_ID, DRIVETRAIN_CANBUS),
            limelight,

            velocity,
            angular_velocity,

            yaw: Angle::new::<degree>(0.0),
            offset: Angle::new::<degree>(0.0),

            motor_encoder_offsets,

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

    /// updates the limelight values and passes in drivetrain data for fom
    pub async fn update_limelight(&mut self) {
        let pose = self.get_pose_estimate();
        let _ = timeout(
            Duration::from_millis(10),
            self.limelight
                .update(pose.angle, Vector2::new(pose.x, pose.y)),
        )
        .await;
    }

    /// ## Gets the pose estimate.
    /// Note: FOM only applies to x and y. <br>
    /// Note: ONLY CALL THIS AFTER CONTROL_DRIVETRAIN HAS BEEN CALLED. If you call this before, the values will not be accurate.
    pub fn get_pose_estimate(&self) -> RobotPoseEstimate {
        self.odometry.pose_estimate.clone()
    }

    /// ## Set the robot's pose.
    pub fn set_pose_estimate(&mut self, pose_estimate: RobotPoseEstimate) {
        self.odometry.pose_estimate = pose_estimate;
    }

    /// Resets the gyro.
    pub fn reset_heading(&mut self) {
        self.offset = self.limelight.get_yaw() + Angle::new::<degree>(180.);
    }

    /// Field-orientate input from the driverstation.
    /// target_transformation is the x and y input from the driverstation put into a vector.
    /// This function rotates the driver's field orientated input to be robot oriented but the same direction.
    fn field_orientate(&self, target_transformation: Vector2<f64>) -> Vector2<f64> {
        // println!(
        //     "[DEBUG]: field_orient: yaw: {:?}",
        //     self.limelight.get_yaw()
        // );
        let oriented = Rotation2::new(
            self.limelight.get_yaw().get::<radian>() + self.offset.get::<radian>() + 180.,
        ) * target_transformation;
        // println!("{}", oriented);
        oriented
    }
    /// Optimizes the setpoints.
    /// For example, instead of turning to 135 degrees from 0 degrees, turn to -45 degrees and invert speed.
    /// Targets need to be -180 to 180 degs
    pub fn optimize_targets(&self, targets: Vec<(f64, Angle)>) -> Vec<(f64, Angle)> {
        let mut measured = vec![
            Angle::new::<revolution>(self.fl_turn.get_position() / SWERVE_TURN_RATIO),
            Angle::new::<revolution>(self.bl_turn.get_position() / SWERVE_TURN_RATIO),
            Angle::new::<revolution>(self.br_turn.get_position() / SWERVE_TURN_RATIO),
            Angle::new::<revolution>(self.fr_turn.get_position() / SWERVE_TURN_RATIO),
        ];

        let mut difs = vec![
            get_angle_difs(measured[0].clone(), targets.clone()[0].1),
            get_angle_difs(measured[1].clone(), targets.clone()[1].1),
            get_angle_difs(measured[2].clone(), targets.clone()[2].1),
            get_angle_difs(measured[3].clone(), targets.clone()[3].1),
        ];

        let mut optimized_difs = vec![
            optimize_difs(difs[0]),
            optimize_difs(difs[1]),
            optimize_difs(difs[2]),
            optimize_difs(difs[3]),
        ];

        vec![
            add_difs_to_setpoints(optimized_difs[0], measured[0], targets[0]),
            add_difs_to_setpoints(optimized_difs[1], measured[1], targets[1]),
            add_difs_to_setpoints(optimized_difs[2], measured[2], targets[2]),
            add_difs_to_setpoints(optimized_difs[3], measured[3], targets[3]),
        ]
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
            ControlMode::Percent,
            targets[0].0 * relative_target_modifers[0].1,
        );
        self.bl_drive.set(
            ControlMode::Percent,
            targets[1].0 * relative_target_modifers[1].1,
        );
        self.br_drive.set(
            ControlMode::Percent,
            targets[2].0 * relative_target_modifers[2].1,
        );
        self.fr_drive.set(
            ControlMode::Percent,
            targets[3].0 * relative_target_modifers[3].1,
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

        // println!("[DEBUG]: set_speeds: input: {:?}", targets);
        // set drive motor speeds based on targets
        // println!("[DEBUG]: set_speeds: setting fl_drive to: {}", targets[0].0);
        // println!("[DEBUG]: set_speeds: setting bl_drive to: {}", targets[1].0);
        // println!("[DEBUG]: set_speeds: setting br_drive to: {}", targets[2].0);
        // println!("[DEBUG]: set_speeds: setting fr_drive to: {}", targets[3].0);

        // set turn motors based on targets
        // println!(
        //     "[DEBUG]: set_speeds: setting fl_turn to: {}",
        //     (targets[0].1.get::<revolution>() + self.motor_encoder_offsets[0]) * SWERVE_TURN_RATIO
        // );
        // println!(
        //     "[DEBUG]: set_speeds: setting bl_turn to: {}",
        //     (targets[1].1.get::<revolution>() + self.motor_encoder_offsets[1]) * SWERVE_TURN_RATIO
        // );
        // println!(
        //     "[DEBUG]: set_speeds: setting br_turn to: {}",
        //     (targets[2].1.get::<revolution>() + self.motor_encoder_offsets[2]) * SWERVE_TURN_RATIO
        // );
        // println!(
        //     "[DEBUG]: set_speeds: setting fr_turn to: {}",
        //     (targets[3].1.get::<revolution>() + self.motor_encoder_offsets[3]) * SWERVE_TURN_RATIO
        // );

        // println!("constant offsets: {:?}", self.motor_encoder_offsets);
        // println!(
        //     "current frame abs encoder: fl: {}",
        //     self.fl_encoder.get_absolute()
        // );
        // println!(
        //     "current frame abs encoder: bl: {}",
        //     self.bl_encoder.get_absolute()
        // );
        // println!(
        //     "current frame abs encoder: br: {}",
        //     self.br_encoder.get_absolute()
        // );
        // println!(
        //     "current frame abs encoder: fr: {}",
        //     self.fr_encoder.get_absolute()
        // );
    }

    /// Control the drivetrain.
    /// x, y, and rotation are driverstation inputs.
    pub fn control_drivetrain(&mut self, x: f64, y: f64, rotation: f64) {
        // NOTE: REMOVE THIS WHEN IMPLEMENTING FUSED
        self.update_pose();

        println!("pose: x: {}", self.odometry.pose_estimate.x.get::<meter>());
        println!("pose: y: {}", self.odometry.pose_estimate.y.get::<meter>());
        println!(
            "pose: angle: {}",
            self.odometry.pose_estimate.angle.get::<degree>()
        );

        let target_transformation = match config::FIELD_ORIENTED {
            true => self.field_orientate(vector![x, y]),
            false => vector![x, y],
        };

        self.update_pose();

        let targets = self.kinematics.get_targets(target_transformation, rotation);

        let optimized_targets = self.optimize_targets(targets);

        self.set_speeds(optimized_targets);
    }

    /// #updates the localized cords using odo and vision
    /// returns a RobotPoseEstimate and sets the odo pose to the best cords
    pub async fn update_localization(&mut self) -> RobotPoseEstimate {
        // Updates drivetrain.odometry.pose_estimate.
        self.update_pose();

        // set fused pose to current odo as a base
        let mut fused_pose = self.get_pose_estimate();

        // attempt to get vision pose
        if let Some(vision_xy) = self.limelight.get_botpose_orb() {
            // get both of the figures of merit; higher is better
            let vision_fom = self.limelight.get_vision_fom();
            let odo_fom = fused_pose.fom;

            // get the total fom should not be >1
            let total_weight = odo_fom + vision_fom;

            // fuse the x cords based on fom
            let fused_x = (fused_pose.x.get::<meter>() * odo_fom
                + vision_xy.x.get::<meter>() * vision_fom)
                / total_weight;

            // fuse the y cords based on fom
            let fused_y = (fused_pose.y.get::<meter>() * odo_fom
                + vision_xy.y.get::<meter>() * vision_fom)
                / total_weight;

            // fuse the yaws to wrap 0-360
            let odo_yaw = fused_pose.angle.get::<radian>();
            let vis_yaw = self.limelight.get_yaw().get::<radian>();

            let sin_sum = odo_yaw.sin() * odo_fom + vis_yaw.sin() * vision_fom;
            let cos_sum = odo_yaw.cos() * odo_fom + vis_yaw.cos() * vision_fom;

            let mut fused_yaw = sin_sum.atan2(cos_sum);

            if fused_yaw < 0. || fused_yaw >= std::f64::consts::PI * 2. {
                println!(
                    "[ERROR] fused_yaw was negative after a function that should only ever return [0,2pi). Computed fused_yaw: {}",
                    fused_yaw
                );
                fused_yaw = fused_yaw.rem_euclid(std::f64::consts::PI * 2.);
            }

            //
            fused_pose.x = Length::new::<meter>(fused_x);
            fused_pose.y = Length::new::<meter>(fused_y);
            fused_pose.angle = Angle::new::<radian>(fused_yaw);
        }

        // set odo to fused fom
        self.set_pose_estimate(fused_pose.clone());
        self.set_next_frame_module_odometry();
        RobotPoseEstimate::new(fused_pose.fom, fused_pose.x, fused_pose.y, fused_pose.angle)
    }

    pub async fn post_odo(&self) {
        Telemetry::put_number("odo_x", self.get_pose_estimate().x.get::<meter>()).await;
        Telemetry::put_number("odo_y", self.get_pose_estimate().y.get::<meter>()).await;
        Telemetry::put_number(
            "odo_heading",
            self.get_pose_estimate().angle.get::<radian>(),
        )
        .await;
        Telemetry::put_number("odo_fom", self.get_pose_estimate().fom).await;
    }

    // Needs to be updated each frame
    pub fn update_velocity(&mut self) {
        // meters/second
        let magnitude = self.limelight.get_linear_velocity();
        let heading = self.yaw;

        let frame_velocity: Vector2<f64> = Vector2::new(
            magnitude * (heading.get::<radian>().cos()),
            magnitude * (heading.get::<radian>().sin()),
        );

        self.velocity = frame_velocity;
        self.angular_velocity = self.limelight.get_angular_velocity();
    }

    pub fn get_yaw(&self) -> Angle {
        self.limelight.get_yaw() + self.offset
    }
}

fn get_angle_difs(from: Angle, to: Angle) -> Angle {
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

fn add_difs_to_setpoints(dif: (Angle, f64), measured: Angle, target: (f64, Angle)) -> (f64, Angle) {
    ((dif.1 * target.0), measured + dif.0)
}

fn abs_offset(abs_zero_position: Angle, abs_reading: Angle, target: Angle) -> Angle {
    // current_angle + get_angle_difs(get_angle_difs(abs_zero_position, abs_reading), target)
    wrap_angle(target - (abs_reading - abs_zero_position))
}

#[cfg(test)]
mod drivetrain_tests {
    use super::*;
    use float_cmp::assert_approx_eq;
    use std::{f64::NAN, panic};

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

    #[test]
    fn add_difs_to_setpoints_test() {
        // dif: (Angle, f64), measured: Angle, target: (f64, Angle)
        let inputs = vec![
            (
                (Angle::new::<degree>(0.0), 1.0), // 1
                Angle::new::<degree>(0.0),        // 1
                (1.0, Angle::new::<degree>(NAN)), // 1
            ),
            (
                (Angle::new::<degree>(45.0), 1.0), // 2
                Angle::new::<degree>(0.0),         // 2
                (1.0, Angle::new::<degree>(NAN)),  // 2
            ),
            (
                (Angle::new::<degree>(90.0), 1.0), // 3
                Angle::new::<degree>(0.0),         // 3
                (1.0, Angle::new::<degree>(NAN)),  // 3
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 4
                Angle::new::<degree>(0.0),          // 4
                (1.0, Angle::new::<degree>(NAN)),   // 4
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 5
                Angle::new::<degree>(0.0),          // 5
                (1.0, Angle::new::<degree>(NAN)),   // 5
            ),
            (
                (Angle::new::<degree>(0.0), 1.0), // 6
                Angle::new::<degree>(90.0),       // 6
                (1.0, Angle::new::<degree>(NAN)), // 6
            ),
            (
                (Angle::new::<degree>(45.0), 1.0), // 7
                Angle::new::<degree>(90.0),        // 7
                (1.0, Angle::new::<degree>(NAN)),  // 7
            ),
            (
                (Angle::new::<degree>(90.0), 1.0), // 8
                Angle::new::<degree>(90.0),        // 8
                (1.0, Angle::new::<degree>(NAN)),  // 8
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 9
                Angle::new::<degree>(90.0),         // 9
                (1.0, Angle::new::<degree>(NAN)),   // 9
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 10
                Angle::new::<degree>(90.0),         // 10
                (1.0, Angle::new::<degree>(NAN)),   // 10
            ),
            (
                (Angle::new::<degree>(0.0), 1.0), // 11
                Angle::new::<degree>(-90.0),      // 11
                (1.0, Angle::new::<degree>(NAN)), // 11
            ),
            (
                (Angle::new::<degree>(45.0), 1.0), // 12
                Angle::new::<degree>(-90.0),       // 12
                (1.0, Angle::new::<degree>(NAN)),  // 12
            ),
            (
                (Angle::new::<degree>(90.0), 1.0), // 13
                Angle::new::<degree>(-90.0),       // 13
                (1.0, Angle::new::<degree>(NAN)),  // 13
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 14
                Angle::new::<degree>(-90.0),        // 14
                (1.0, Angle::new::<degree>(NAN)),   // 14
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 15
                Angle::new::<degree>(-90.0),        // 15
                (1.0, Angle::new::<degree>(NAN)),   // 15
            ),
            (
                (Angle::new::<degree>(0.0), 1.0),  // 16
                Angle::new::<degree>(0.0 + 720.0), // 16
                (1.0, Angle::new::<degree>(NAN)),  // 16
            ),
            (
                (Angle::new::<degree>(45.0), 1.0), // 17
                Angle::new::<degree>(0.0 + 720.0), // 17
                (1.0, Angle::new::<degree>(NAN)),  // 17
            ),
            (
                (Angle::new::<degree>(90.0), 1.0), // 18
                Angle::new::<degree>(0.0 + 720.0), // 18
                (1.0, Angle::new::<degree>(NAN)),  // 18
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 19
                Angle::new::<degree>(0.0 + 720.0),  // 19
                (1.0, Angle::new::<degree>(NAN)),   // 19
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 20
                Angle::new::<degree>(0.0 + 720.0),  // 20
                (1.0, Angle::new::<degree>(NAN)),   // 20
            ),
            (
                (Angle::new::<degree>(0.0), 1.0),   // 21
                Angle::new::<degree>(90.0 + 720.0), // 21
                (1.0, Angle::new::<degree>(NAN)),   // 21
            ),
            (
                (Angle::new::<degree>(45.0), 1.0),  // 22
                Angle::new::<degree>(90.0 + 720.0), // 22
                (1.0, Angle::new::<degree>(NAN)),   // 22
            ),
            (
                (Angle::new::<degree>(90.0), 1.0),  // 23
                Angle::new::<degree>(90.0 + 720.0), // 23
                (1.0, Angle::new::<degree>(NAN)),   // 23
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 24
                Angle::new::<degree>(90.0 + 720.0), // 24
                (1.0, Angle::new::<degree>(NAN)),   // 24
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 25
                Angle::new::<degree>(90.0 + 720.0), // 25
                (1.0, Angle::new::<degree>(NAN)),   // 25
            ),
            (
                (Angle::new::<degree>(0.0), 1.0),    // 26
                Angle::new::<degree>(-90.0 + 720.0), // 26
                (1.0, Angle::new::<degree>(NAN)),    // 26
            ),
            (
                (Angle::new::<degree>(45.0), 1.0),   // 27
                Angle::new::<degree>(-90.0 + 720.0), // 27
                (1.0, Angle::new::<degree>(NAN)),    // 27
            ),
            (
                (Angle::new::<degree>(90.0), 1.0),   // 28
                Angle::new::<degree>(-90.0 + 720.0), // 28
                (1.0, Angle::new::<degree>(NAN)),    // 28
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0),  // 29
                Angle::new::<degree>(-90.0 + 720.0), // 29
                (1.0, Angle::new::<degree>(NAN)),    // 29
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0),  // 30
                Angle::new::<degree>(-90.0 + 720.0), // 30
                (1.0, Angle::new::<degree>(NAN)),    // 30
            ),
            (
                (Angle::new::<degree>(0.0), 1.0),  // 31
                Angle::new::<degree>(0.0 - 720.0), // 31
                (1.0, Angle::new::<degree>(NAN)),  // 31
            ),
            (
                (Angle::new::<degree>(45.0), 1.0), // 32
                Angle::new::<degree>(0.0 - 720.0), // 32
                (1.0, Angle::new::<degree>(NAN)),  // 32
            ),
            (
                (Angle::new::<degree>(90.0), 1.0), // 33
                Angle::new::<degree>(0.0 - 720.0), // 33
                (1.0, Angle::new::<degree>(NAN)),  // 33
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 34
                Angle::new::<degree>(0.0 - 720.0),  // 34
                (1.0, Angle::new::<degree>(NAN)),   // 34
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 35
                Angle::new::<degree>(0.0 - 720.0),  // 35
                (1.0, Angle::new::<degree>(NAN)),   // 35
            ),
            (
                (Angle::new::<degree>(0.0), 1.0),   // 36
                Angle::new::<degree>(90.0 - 720.0), // 36
                (1.0, Angle::new::<degree>(NAN)),   // 36
            ),
            (
                (Angle::new::<degree>(45.0), 1.0),  // 37
                Angle::new::<degree>(90.0 - 720.0), // 37
                (1.0, Angle::new::<degree>(NAN)),   // 37
            ),
            (
                (Angle::new::<degree>(90.0), 1.0),  // 38
                Angle::new::<degree>(90.0 - 720.0), // 38
                (1.0, Angle::new::<degree>(NAN)),   // 38
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0), // 39
                Angle::new::<degree>(90.0 - 720.0), // 39
                (1.0, Angle::new::<degree>(NAN)),   // 39
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0), // 40
                Angle::new::<degree>(90.0 - 720.0), // 40
                (1.0, Angle::new::<degree>(NAN)),   // 40
            ),
            (
                (Angle::new::<degree>(0.0), 1.0),    // 41
                Angle::new::<degree>(-90.0 - 720.0), // 41
                (1.0, Angle::new::<degree>(NAN)),    // 41
            ),
            (
                (Angle::new::<degree>(45.0), 1.0),   // 42
                Angle::new::<degree>(-90.0 - 720.0), // 42
                (1.0, Angle::new::<degree>(NAN)),    // 42
            ),
            (
                (Angle::new::<degree>(90.0), 1.0),   // 43
                Angle::new::<degree>(-90.0 - 720.0), // 43
                (1.0, Angle::new::<degree>(NAN)),    // 43
            ),
            (
                (Angle::new::<degree>(-45.0), 1.0),  // 44
                Angle::new::<degree>(-90.0 - 720.0), // 44
                (1.0, Angle::new::<degree>(NAN)),    // 44
            ),
            (
                (Angle::new::<degree>(-90.0), 1.0),  // 45
                Angle::new::<degree>(-90.0 - 720.0), // 45
                (1.0, Angle::new::<degree>(NAN)),    // 45
            ),
            (
                (Angle::new::<degree>(90.0), -1.0), // 46
                Angle::new::<degree>(0.0),          // 46
                (1.0, Angle::new::<degree>(NAN)),   // 46
            ),
        ];

        let expected = vec![
            (1.0, Angle::new::<degree>(0.0)),            // 1
            (1.0, Angle::new::<degree>(45.0)),           // 2
            (1.0, Angle::new::<degree>(90.0)),           // 3
            (1.0, Angle::new::<degree>(-45.0)),          // 4
            (1.0, Angle::new::<degree>(-90.0)),          // 5
            (1.0, Angle::new::<degree>(90.0)),           // 6
            (1.0, Angle::new::<degree>(135.0)),          // 7
            (1.0, Angle::new::<degree>(180.0)),          // 8
            (1.0, Angle::new::<degree>(45.0)),           // 9
            (1.0, Angle::new::<degree>(0.0)),            // 10
            (1.0, Angle::new::<degree>(-90.0)),          // 11
            (1.0, Angle::new::<degree>(-45.0)),          // 12
            (1.0, Angle::new::<degree>(-0.0)),           // 13
            (1.0, Angle::new::<degree>(-135.0)),         // 14
            (1.0, Angle::new::<degree>(-180.0)),         // 15
            (1.0, Angle::new::<degree>(0.0 + 720.0)),    // 16
            (1.0, Angle::new::<degree>(45.0 + 720.0)),   // 17
            (1.0, Angle::new::<degree>(90.0 + 720.0)),   // 18
            (1.0, Angle::new::<degree>(-45.0 + 720.0)),  // 19
            (1.0, Angle::new::<degree>(-90.0 + 720.0)),  // 20
            (1.0, Angle::new::<degree>(90.0 + 720.0)),   // 21
            (1.0, Angle::new::<degree>(135.0 + 720.0)),  // 22
            (1.0, Angle::new::<degree>(180.0 + 720.0)),  // 23
            (1.0, Angle::new::<degree>(45.0 + 720.0)),   // 24
            (1.0, Angle::new::<degree>(0.0 + 720.0)),    // 25
            (1.0, Angle::new::<degree>(-90.0 + 720.0)),  // 26
            (1.0, Angle::new::<degree>(-45.0 + 720.0)),  // 27
            (1.0, Angle::new::<degree>(-0.0 + 720.0)),   // 28
            (1.0, Angle::new::<degree>(-135.0 + 720.0)), // 29
            (1.0, Angle::new::<degree>(-180.0 + 720.0)), // 30
            (1.0, Angle::new::<degree>(0.0 - 720.0)),    // 31
            (1.0, Angle::new::<degree>(45.0 - 720.0)),   // 32
            (1.0, Angle::new::<degree>(90.0 - 720.0)),   // 33
            (1.0, Angle::new::<degree>(-45.0 - 720.0)),  // 34
            (1.0, Angle::new::<degree>(-90.0 - 720.0)),  // 35
            (1.0, Angle::new::<degree>(90.0 - 720.0)),   // 36
            (1.0, Angle::new::<degree>(135.0 - 720.0)),  // 37
            (1.0, Angle::new::<degree>(180.0 - 720.0)),  // 38
            (1.0, Angle::new::<degree>(45.0 - 720.0)),   // 39
            (1.0, Angle::new::<degree>(0.0 - 720.0)),    // 40
            (1.0, Angle::new::<degree>(-90.0 - 720.0)),  // 41
            (1.0, Angle::new::<degree>(-45.0 - 720.0)),  // 42
            (1.0, Angle::new::<degree>(-0.0 - 720.0)),   // 43
            (1.0, Angle::new::<degree>(-135.0 - 720.0)), // 44
            (1.0, Angle::new::<degree>(-180.0 - 720.0)), // 45
            (-1.0, Angle::new::<degree>(90.0)),          // 46
        ];

        let mut results = Vec::new();

        for input in inputs {
            results.push(add_difs_to_setpoints(input.0, input.1, input.2));
        }

        let mut i = 1.0;
        println!("      (speed, angle)");
        for result in results.clone() {
            println!("result {}: ({}, {})", i, result.0, result.1.get::<degree>());
            i += 1.0;
        }

        for tuple in expected.iter().zip(results.iter()) {
            assert_approx_eq!(f64, tuple.0.0, tuple.1.0);
            assert_approx_eq!(f64, tuple.0.1.get::<degree>(), tuple.1.1.get::<degree>());
        }
    }

    #[test]
    fn abs_encoder_test() {
        // abs_zero_position: Angle,
        // abs_reading: Angle,
        // target: Angle,
        // current_angle: Angle,
        let inputs = vec![(0.418, 0.669, 0.0, 0.052)];

        let expected = vec![-0.198];

        let mut results = Vec::new();

        for input in inputs {
            results.push(abs_offset(
                Angle::new::<revolution>(input.0),
                Angle::new::<revolution>(input.1),
                Angle::new::<revolution>(input.2),
                Angle::new::<revolution>(input.3),
            ))
        }

        let mut i = 1;
        for tuple in results.iter().zip(expected.iter()) {
            println!("result {}: {}", i, tuple.0.get::<revolution>());
            i += 1;
            assert_approx_eq!(
                f64,
                tuple.1.clone(),
                tuple.0.get::<revolution>(),
                epsilon = 0.002
            );
        }
    }
}
