use crate::constants::config::{
    FIELD_ORIENTED, MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND,
    MINIMUM_MODULE_VELOCITY_METERS_PER_SECOND,
};
use crate::constants::drivetrain::{
    BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS, BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS,
    DRIVETRAIN_ANGLE_SNAP_KP, FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS,
    FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS, SWERVE_DRIVE_RATIO, SWERVE_TURN_RATIO,
};
use crate::constants::robotmap::drivetrain_map::{
    BL_DRIVE_ID, BL_ENCODER_ID, BL_TURN_ID, BR_DRIVE_ID, BR_ENCODER_ID, BR_TURN_ID,
    DRIVETRAIN_CANBUS, FL_DRIVE_ID, FL_ENCODER_ID, FL_TURN_ID, FR_DRIVE_ID, FR_ENCODER_ID,
    FR_TURN_ID, GYRO_ID,
};
use crate::constants::robotmap::shooter::SHOOTER_CANBUS;
use crate::subsystems::swerve::kinematics::Kinematics;
use crate::subsystems::vision::Vision;
use frcrs::ctre::{CanCoder, ControlMode, Pigeon, Talon};
use pid::Pid;
use std::net::{IpAddr, Ipv4Addr, SocketAddr};
use tokio::time::Instant;
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};

pub struct Drivetrain {
    pub gyro: Pigeon,
    pub gyro_zero: Angle,
    pub limelight_side: Vision,
    pub limelight_front: Vision,
    timer: Instant,
    pub(in crate::subsystems::swerve) kinematics: Kinematics,
    pub turn_pid: Pid<f64>,

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
    pub fn new() -> Drivetrain {
        // make the encoders before rest of robot - we need them to get CANCoder offsets
        let fl_encoder = CanCoder::new(FL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let bl_encoder = CanCoder::new(BL_ENCODER_ID, DRIVETRAIN_CANBUS);
        let br_encoder = CanCoder::new(BR_ENCODER_ID, DRIVETRAIN_CANBUS);
        let fr_encoder = CanCoder::new(FR_ENCODER_ID, DRIVETRAIN_CANBUS);

        let limelight_front = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 14)),
            5807,
        ));

        let limelight_side = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        let turn_pid = Pid::new(0.0, MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND);

        Drivetrain {
            gyro: Pigeon::new(GYRO_ID, Some(SHOOTER_CANBUS.to_string())),
            gyro_zero: Angle::new::<degree>(0.0),
            limelight_front,
            limelight_side,
            timer: Instant::now(),
            kinematics: Kinematics::new(),
            turn_pid,

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

    pub fn get_dt_heading(&self) -> Angle {
        get_angle_difs(
            self.gyro_zero,
            Angle::new::<radian>(self.gyro.get_rotation().z),
        )
    }

    pub fn set_gyro_zero(&mut self) {
        self.gyro_zero = Angle::new::<radian>(self.gyro.get_rotation().z);
    }

    pub fn field_orientate(&mut self, angle: Angle) -> Angle {
        let yaw = self.get_dt_heading();
        -yaw + angle
    }

    /// ## Sets drivetrain motor speeds.
    pub fn set_speeds(&mut self, targets: Vec<(f64, Angle)>) {
        let mut update_modules = false;
        for tuple in &targets {
            if tuple.0.abs() > MINIMUM_MODULE_VELOCITY_METERS_PER_SECOND {
                update_modules = true;
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

        if update_modules {
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
        } else {
            self.fl_drive.set(ControlMode::Percent, 0.0);
            self.bl_drive.set(ControlMode::Percent, 0.0);
            self.br_drive.set(ControlMode::Percent, 0.0);
            self.fr_drive.set(ControlMode::Percent, 0.0);
        }
    }

    /// Control the drivetrain.
    /// magnitude and theta are for polar coordinate translation, rotation is for robot rotation.
    pub fn control_drivetrain(&mut self, theta: Angle, magnitude: Length, rotation: Angle) {
        let new_theta = match FIELD_ORIENTED {
            true => self.field_orientate(theta),
            false => theta,
        };

        let targets = self.kinematics.get_targets(new_theta, magnitude, rotation);

        self.set_speeds(targets);
    }

    pub fn turn_to(&mut self, theta: Angle, mag: Length, desired: Angle) {
        let yaw = self.get_dt_heading();
        self.turn_pid.setpoint(desired.get::<radian>());

        self.turn_pid.p(
            DRIVETRAIN_ANGLE_SNAP_KP,
            MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND,
        );

        let output = self
            .turn_pid
            .next_control_output(yaw.get::<radian>())
            .output;

        let rotation_rate = Angle::new::<radian>(-output);
        self.control_drivetrain(theta, mag, rotation_rate);
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

pub fn abs_offset(abs_zero_position: Angle, abs_reading: Angle, target: Angle) -> Angle {
    wrap_angle(target - (abs_reading - abs_zero_position))
}
