use crate::constants::config;
use crate::constants::drivetrain::{DRIVETRAIN_ERROR_THRESHOLD, SWERVE_TURN_RATIO};
use crate::constants::robotmap::drivetrain_map::{
    BL_DRIVE_ID, BL_ENCODER_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, DRIVETRAIN_CANBUS,
    FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID, FR_TURN_ID, GYRO_ID,
};
use crate::subsystems::swerve::kinematics::Kinematics;
use crate::subsystems::swerve::odometry::{Odometry, RobotPoseEstimate};
use crate::subsystems::vision::Vision;
use frcrs::ctre::{CanCoder, ControlMode, Pigeon, Talon};
use nalgebra::{Rotation2, Vector2, vector};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use std::time::Duration;
use tokio::time::timeout;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::Angle;
use uom::si::length::meter;

/// Drivetrain struct.
/// kinematics field interfaces with inverse kinematics functions.
/// motor_encoder_offsets are the absolute positions of the CANCoders on startup. These allow us to start the robot without physically zeroing the wheels.
pub struct Drivetrain {
    kinematics: Kinematics,
    pub(in crate::subsystems::swerve) odometry: Odometry,
    pub(in crate::subsystems::swerve) gyro: Pigeon,

    limelight: Vision,

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
            gyro: Pigeon::new(GYRO_ID, DRIVETRAIN_CANBUS),

            limelight,

            motor_encoder_offsets,

            fl_encoder,
            fl_drive: Talon::new(FL_DRIVE_ID, DRIVETRAIN_CANBUS),
            fl_turn: Talon::new(FL_TURN_ID, DRIVETRAIN_CANBUS),

            bl_encoder,
            bl_drive: Talon::new(BL_DRIVE_ID, DRIVETRAIN_CANBUS),
            bl_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

            br_encoder,
            br_drive: Talon::new(BR_DRIVE_ID, DRIVETRAIN_CANBUS),
            br_turn: Talon::new(BL_TURN_ID, DRIVETRAIN_CANBUS),

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
        self.gyro.reset();
    }

    /// Field-orientate input from the driverstation.
    /// target_transformation is the x and y input from the driverstation put into a vector.
    /// This function rotates the driver's field orientated input to be robot oriented but the same direction.
    fn field_orientate(&self, target_transformation: Vector2<f64>) -> Vector2<f64> {
        Rotation2::new(-self.gyro.get_angle()) * target_transformation
    }

    /// Optimizes the setpoints.
    /// For example, instead of turning to 135 degrees from 0 degrees, turn to -45 degrees and invert speed.
    fn optimize_setpoints(&self, setpoints: Vec<(f64, Angle)>) -> Vec<(f64, Angle)> {
        // get angles, account for non-zero starting angle
        let measured_angles = vec![
            Angle::new::<revolution>(self.fl_turn.get_position()) + self.motor_encoder_offsets[0],
            Angle::new::<revolution>(self.bl_turn.get_position()) + self.motor_encoder_offsets[1],
            Angle::new::<revolution>(self.br_turn.get_position()) + self.motor_encoder_offsets[2],
            Angle::new::<revolution>(self.fr_turn.get_position()) + self.motor_encoder_offsets[3],
        ];

        //iterate through the setpoints and current angles
        setpoints
            .into_iter()
            .zip(measured_angles)
            .map(|((mut speed, target_angle), current_angle)| {
                //convert both angles to degrees for comparison
                let target_angle = target_angle.get::<degree>();
                let current = current_angle.get::<degree>();

                //calculate the shortest angular difference between target and current
                let mut difference = (target_angle - current + 180.0) % 360.0 - 180.0;
                if difference < -180.0 {
                    difference += 360.0;
                }

                //if rotating more than 90° flip speed and rotate in opposite direction
                if difference.abs() > 90.0 {
                    speed *= -1.0;
                    if difference > 0.0 {
                        difference += -180.0;
                    } else {
                        difference += 180.0;
                    }
                }

                //apply optimized angle adjustment to current angle
                let optimized_angle = current_angle + Angle::new::<degree>(difference);
                (speed, optimized_angle)
            })
            .collect()
    }

    /// ## Sets drivetrain motor speeds.
    pub fn set_speeds(&mut self, targets: Vec<(f64, Angle)>) {
        // set drive motor speeds based on targets
        self.fl_drive.set(ControlMode::Percent, targets[0].0);
        self.bl_drive.set(ControlMode::Percent, targets[1].0);
        self.br_drive.set(ControlMode::Percent, targets[2].0);
        self.fr_drive.set(ControlMode::Percent, targets[3].0);

        // set turn motors based on targets
        self.fl_turn.set(
            ControlMode::Percent,
            -(targets[0].1.get::<revolution>() - self.motor_encoder_offsets[0].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
        self.bl_turn.set(
            ControlMode::Percent,
            -(targets[1].1.get::<revolution>() - self.motor_encoder_offsets[1].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
        self.br_turn.set(
            ControlMode::Percent,
            -(targets[2].1.get::<revolution>() - self.motor_encoder_offsets[2].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
        self.fr_turn.set(
            ControlMode::Percent,
            -(targets[3].1.get::<revolution>() - self.motor_encoder_offsets[3].get::<revolution>())
                * SWERVE_TURN_RATIO,
        );
    }

    /// Control the drivetrain.
    /// x, y, and rotation are driverstation inputs.
    pub fn control_drivetrain(&mut self, x: f64, y: f64, rotation: f64) {
        let target_transformation = match config::FIELD_ORIENTED {
            true => self.field_orientate(vector![x, y]),
            false => vector![x, y],
        };

        let targets = self.kinematics.get_targets(target_transformation, rotation);
        let optimized_targets = self.optimize_setpoints(targets);
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
            // get both of the figures of merit higher is better
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

            if fused_yaw < 0. {
                fused_yaw += std::f64::consts::PI * 2.
            }

            //
            fused_pose.x = uom::si::f64::Length::new::<meter>(fused_x);
            fused_pose.y = uom::si::f64::Length::new::<meter>(fused_y);
            fused_pose.angle = uom::si::f64::Angle::new::<radian>(fused_yaw);

            // update fom to reflect fused confidence
            fused_pose.fom = (odo_fom * vision_fom) / (odo_fom + vision_fom);
        }

        // set odo to fused fom
        self.set_pose_estimate(fused_pose.clone());
        self.set_next_frame_module_odometry();
        RobotPoseEstimate::new(fused_pose.fom, fused_pose.x, fused_pose.y, fused_pose.angle)
    }
}
