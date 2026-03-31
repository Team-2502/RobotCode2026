use crate::constants::config::{
    MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND, SHOOTER_INITAL_DISTANCE_OFFSET_FEET,
};
use crate::constants::localization::{
    COMMANDED_VELOCITY_WEIGHT, POSE_ANTICIPATION_TIMESTEP_SECS, YAW_ANTICIPATION_TIMESTEP_SECS,
};
use crate::constants::robotmap::shooter::{
    HOOD_MOTOR_ID, SHOOTER_CANBUS, SHOOTER_MOTOR_LEFT_ID, SHOOTER_MOTOR_RIGHT_ID,
};
use crate::constants::shooter::{MAX_FLYWHEEL_SPEED, SHOOTER_DISTANCE_ERROR_SMUDGE};
use crate::constants::turret::{
    HOOD_MAX_SOFTSTOP, HOOD_MIN_SOFTSTOP, ORIGIN_TO_TURRET_CENTER_X_INCHES,
    ORIGIN_TO_TURRET_CENTER_Y_INCHES,
};
use crate::subsystems::localization::RobotPose;
use crate::subsystems::swerve::drivetrain::get_angle_difs;
use crate::subsystems::turret::{Turret, get_angle_to};
use crate::subsystems::vision::distance;
use frcrs::ctre::{ControlMode, Talon};
use nalgebra::{Rotation2, Vector2};
use std::fs::File;
use std::io::{Read, Write};
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{foot, inch, meter};

#[derive(PartialEq, Clone)]
pub enum ShootingTarget {
    Hub,
    PassTop,
    PassBottom,
    PassTelemetry,
}

pub struct Shooter {
    shooter_motor_left: Talon,
    shooter_motor_right: Talon,
    hood_motor: Talon,
    pub distance_offset: Length,
    pub turret: Turret,
    hood_offset: Angle,
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor_left =
            Talon::new(SHOOTER_MOTOR_LEFT_ID, Some(SHOOTER_CANBUS.to_string()));
        let shooter_motor_right =
            Talon::new(SHOOTER_MOTOR_RIGHT_ID, Some(SHOOTER_CANBUS.to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some(SHOOTER_CANBUS.to_string()));

        let turret = Turret::new();

        let file_result = File::open("/tmp/hood_zero");
        let mut hood_offset: Option<Angle> = None;

        if file_result.is_ok() {
            let mut file = file_result.unwrap();
            let metadata = file.metadata();
            if metadata.is_ok() {
                if metadata.unwrap().len() == 8 {
                    let mut buf = [0u8; 8];
                    if file.read_exact(&mut buf).is_ok() {
                        hood_offset = Some(Angle::new::<revolution>(f64::from_ne_bytes(buf)));
                    }
                }
            }
        }

        if hood_offset.is_none() {
            let zero = hood_motor.get_position();
            hood_offset = Some(Angle::new::<revolution>(zero));

            let zero_bytes = zero.to_ne_bytes();
            let new_file = File::create("/tmp/hood_zero");

            #[allow(unused_must_use)]
            if new_file.is_ok() {
                let mut created_file = new_file.unwrap();
                created_file.write_all(&zero_bytes);
                created_file.flush();
            }
        }

        Shooter {
            shooter_motor_left,
            shooter_motor_right,
            hood_motor,
            distance_offset: Length::new::<foot>(SHOOTER_INITAL_DISTANCE_OFFSET_FEET),
            hood_offset: hood_offset.unwrap(),

            turret,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Percent, speed);
        self.shooter_motor_right.set(ControlMode::Percent, -speed);
    }

    pub fn set_hood(&mut self, angle: f64) {
        self.hood_motor.set(
            ControlMode::Position,
            (angle + self.hood_offset.get::<revolution>())
                .clamp(HOOD_MIN_SOFTSTOP, HOOD_MAX_SOFTSTOP),
        );
    }

    pub fn shoot_to(
        &mut self,
        pose: &RobotPose,
        target: Vector2<Length>,
        commanded_theta: Angle,
        commanded_magnitude: Length,
    ) {
        let current_pose = Vector2::new(pose.x, pose.y);
        let future_pose = Vector2::new(
            pose.x + pose.vx * POSE_ANTICIPATION_TIMESTEP_SECS,
            pose.y + pose.vy * POSE_ANTICIPATION_TIMESTEP_SECS,
        );

        let mut vector_to_turret_center_f64 = Vector2::new(
            ORIGIN_TO_TURRET_CENTER_X_INCHES,
            ORIGIN_TO_TURRET_CENTER_Y_INCHES,
        );

        vector_to_turret_center_f64 =
            Rotation2::new(pose.yaw.get::<radian>()) * vector_to_turret_center_f64;

        let vector_to_turret_center = Vector2::new(
            Length::new::<inch>(vector_to_turret_center_f64.x),
            Length::new::<inch>(vector_to_turret_center_f64.y),
        );

        let velocity_vec = Vector2::new(pose.vx.get::<meter>(), pose.vy.get::<meter>());
        let commanded_vec = Vector2::new(
            f64::cos(commanded_theta.get::<radian>()) * commanded_magnitude.get::<meter>(),
            f64::sin(commanded_theta.get::<radian>()) * commanded_magnitude.get::<meter>(),
        );

        let weighted_vel_f64 = (1.0 - COMMANDED_VELOCITY_WEIGHT) * velocity_vec
            + COMMANDED_VELOCITY_WEIGHT * commanded_vec;
        let weighted_vel = Vector2::new(
            Length::new::<meter>(weighted_vel_f64.x),
            Length::new::<meter>(weighted_vel_f64.y),
        );

        let tr_velocity = get_target_relative_velocity_vector(weighted_vel, pose, target);
        let vx = tr_velocity.x.get::<meter>();
        let vy = tr_velocity.y.get::<meter>();

        let dist = distance(target, current_pose + vector_to_turret_center);
        let future_dist = distance(target, future_pose + vector_to_turret_center);
        let current_flywheel_speed = self.get_speed();

        let speed = predict_speed(future_dist, vx, vy);
        let hood = predict_hood(dist, vx, vy, current_flywheel_speed);
        let angle_offset =
            Angle::new::<radian>(predict_yaw(dist, vx, vy, current_flywheel_speed, hood));

        let angle_target = get_angle_to(future_pose + vector_to_turret_center, target);
        let turret_relative_angle = get_angle_difs(
            pose.yaw + pose.vr * YAW_ANTICIPATION_TIMESTEP_SECS,
            angle_target + angle_offset,
        );

        self.set_velocity(speed);
        self.set_hood(hood);
        self.turret.set_angle(turret_relative_angle);
    }

    pub fn pass_to(&mut self, pose: &RobotPose, target: Vector2<Length>) {
        let current_pose = Vector2::new(pose.x, pose.y);

        let mut vector_to_turret_center_f64 = Vector2::new(
            ORIGIN_TO_TURRET_CENTER_X_INCHES,
            ORIGIN_TO_TURRET_CENTER_Y_INCHES,
        );

        vector_to_turret_center_f64 =
            Rotation2::new(pose.yaw.get::<radian>()) * vector_to_turret_center_f64;

        let vector_to_turret_center = Vector2::new(
            Length::new::<inch>(vector_to_turret_center_f64.x),
            Length::new::<inch>(vector_to_turret_center_f64.y),
        );

        let distance_target =
            Length::new::<meter>(distance(target, current_pose + vector_to_turret_center));
        let angle_target = get_angle_to(current_pose + vector_to_turret_center, target);
        let turret_relative_angle = get_angle_difs(pose.yaw, angle_target);

        let current_flywheel_speed = self.get_speed();
        self.set_velocity(get_passing_shooter_speed_target(distance_target));
        self.set_hood(get_passing_hood_angle_target(
            distance_target,
            current_flywheel_speed,
        ));

        self.turret.set_angle(turret_relative_angle);
    }

    pub fn get_speed(&self) -> f64 {
        (self.shooter_motor_left.get_velocity() + self.shooter_motor_right.get_velocity()) / 2.0
    }

    pub fn set_velocity(&self, speed: f64) {
        let scaled_speed = speed.clamp(30.0, 100.0);
        self.shooter_motor_left
            .set(ControlMode::Velocity, scaled_speed);
        self.shooter_motor_right
            .set(ControlMode::Velocity, scaled_speed);
    }

    pub fn get_hood(&self) -> f64 {
        self.hood_motor.get_position()
    }

    pub fn stop(&self) {
        self.hood_motor.stop();
        self.shooter_motor_left.stop();
        self.shooter_motor_right.stop();
        self.turret.stop();
    }
}

/// converts the given vector into a coordinate system where +y is towards target, +x is tangent to target, (0, 0) is robot center
fn get_target_relative_velocity_vector(
    vector: Vector2<Length>,
    robot_pose: &RobotPose,
    target: Vector2<Length>,
) -> Vector2<Length> {
    // todo: impl robot_pose.to_vec
    let target_f64 = Vector2::new(target.x.get::<meter>(), target.y.get::<meter>());
    let pose_f64 = Vector2::new(robot_pose.x.get::<meter>(), robot_pose.y.get::<meter>());
    let vector_f64 = Vector2::new(vector.x.get::<meter>(), vector.y.get::<meter>());

    let y_unit_vector = (target_f64 - pose_f64).normalize();
    let x_unit_vector = Vector2::new(y_unit_vector.y, -1.0 * y_unit_vector.x).normalize();

    let target_relative_vector = Vector2::new(
        x_unit_vector.dot(&vector_f64),
        y_unit_vector.dot(&vector_f64),
    );

    Vector2::new(
        Length::new::<meter>(target_relative_vector.x),
        Length::new::<meter>(target_relative_vector.y),
    )
}

pub fn get_drivetrain_max_speed(
    robot_pose: &RobotPose,
    theta: Angle,
    target: Vector2<Length>,
) -> Length {
    let pose = Vector2::new(robot_pose.x, robot_pose.y);
    let angle_to_hub = get_angle_to(pose, target);
    let angle_diff = get_angle_difs(theta, angle_to_hub).abs();

    if angle_diff < Angle::new::<degree>(45.0) {
        let percent = angle_diff.get::<degree>() / 45.0;
        Length::new::<meter>(1.0 + (MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND - 1.0) * percent)
    } else {
        Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND)
    }
}

pub fn get_scoring_shooter_speed_target(distance: Length) -> f64 {
    let distance_feet: f64 = distance.get::<foot>() * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let target =
        (0.0652772 * (distance_feet * distance_feet)) + (0.954121 * distance_feet) + 38.92606;

    target.clamp(0.0, MAX_FLYWHEEL_SPEED)
}

pub fn get_scoring_hood_angle_target(distance: Length, current_speed: f64) -> f64 {
    let zero_hood_dist =
        -0.05179036 + 0.00224117 * current_speed + -0.02321219 * distance.get::<meter>();
    let max_hood_dist =
        -0.20176568 + 0.00665605 * current_speed + -0.02321219 * distance.get::<meter>();
    let angle_f64 = (-2.03750054 - zero_hood_dist) * zero_hood_dist / max_hood_dist;
    angle_f64.clamp(0.0, 2.0)
}

// todo: regressions
pub fn get_passing_shooter_speed_target(distance: Length) -> f64 {
    let distance_feet: f64 =
        distance.get::<meter>() as f64 * 3.28084 * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let target = (0.0 * (distance_feet * distance_feet)) + (0.0 * distance_feet) + 0.0;

    target.clamp(0.0, MAX_FLYWHEEL_SPEED)
}

// todo: regressions
pub fn get_passing_hood_angle_target(distance: Length, current_speed: f64) -> f64 {
    let distance_feet: f64 =
        distance.get::<meter>() as f64 * 3.28084 * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let min_speed = (0.0 * (distance_feet * distance_feet)) + (0.0 * distance_feet) + 0.0;
    let max_speed = (0.0 * (distance_feet * distance_feet)) + (0.0 * distance_feet) + 0.0;
    let max_angle = (0.0 * (distance_feet * distance_feet)) + (0.0 * distance_feet) + 0.0;

    if max_angle < 0.0 || current_speed > max_speed {
        0.0
    } else if current_speed < min_speed {
        max_angle
    } else {
        let t = 1.0 - (current_speed - min_speed) / (max_speed - min_speed);
        max_angle * t
    }
}

fn predict_yaw(dist: f64, vx: f64, vy: f64, speed: f64, hood: f64) -> f64 {
    let launch_velocity = speed * (0.14577997574526508 - 0.00015049121874030828 * hood);
    let launch_angle = (69.91396399643477 - 12.56207643932571 * hood).to_radians();
    let tof = 0.1019367991845056 * (launch_velocity * launch_angle.sin()) + 0.8648181411925046;
    if tof.abs() < 1e-9 {
        return 0.0;
    }
    vx.atan2(1.34043298837828 * (dist / tof) - vy + 0.24598678147992)
}

fn predict_hood(dist: f64, vx: f64, vy: f64, speed: f64) -> f64 {
    let flywheel = speed * 0.14;
    let vx2 = vx * vx;
    let vy2 = vy * vy;
    let dist2 = dist * dist;
    let flywheel2 = flywheel * flywheel;
    let term = 14.577997574526508 * flywheel + vy;
    if term.abs() < 1e-9 {
        return 0.0;
    }
    let inv_term = 1.0 / term;

    let num = -0.36839033462703 + 0.02061306141286 * vy + 0.00838512238623 * vx2
        - 0.14755937948384 * dist
        + 0.10020701135136 * flywheel
        + 0.0515687877688 * vy * dist
        + 0.00284413077565 * vy * flywheel
        - 0.00058493307566 * vy * dist * flywheel
        - 0.00252780913557 * vy * dist2
        + 0.00651966765799 * vy2 * dist
        - 0.00898418209993 * vy2 * flywheel
        - 0.00022600092684 * vy2 * dist * flywheel
        + 0.0004097653597 * vy2 * flywheel2
        - 7.4616806e-07 * vy2 * dist2
        + 0.0294495829453 * vx2 * dist
        - 0.01529770258126 * vx2 * flywheel
        - 0.00125497957061 * vx2 * dist * flywheel
        - 0.0078604492234 * vx2 * dist2
        + 0.00086469303317 * vx2 * flywheel2
        - 5.93345503e-05 * vx2 * dist * flywheel2
        + 0.00097282272656 * vx2 * dist2 * flywheel
        - 2.906229428e-05 * vx2 * dist2 * flywheel2
        + 0.6418027181806 * dist * inv_term;

    let den =
        0.6127404877969 - 0.0413350616315 * vy + 0.01488951018526 * vx2 + 0.06990762290123 * dist
            - 0.13889287249169 * flywheel
            - 0.00590932395805 * vy * dist
            + 0.00535759152548 * vy * flywheel
            + 0.00035896826208 * vy * dist * flywheel
            - 0.00052528256156 * vy * dist2
            + 0.00069984300315 * vy2 * dist
            - 0.0004567784541 * vy2 * flywheel
            + 1.377256345e-05 * vy2 * dist * flywheel
            + 7.38593401e-06 * vy2 * flywheel2
            - 1.341855546e-05 * vy2 * dist2
            + 0.01016311678925 * vx2 * dist
            - 0.00584988955478 * vx2 * flywheel
            - 9.662471901e-05 * vx2 * dist * flywheel
            - 0.00360819016392 * vx2 * dist2
            + 0.00034990676645 * vx2 * flywheel2
            - 5.200759247e-05 * vx2 * dist * flywheel2
            + 0.00042243645456 * vx2 * dist2 * flywheel
            - 1.136145234e-05 * vx2 * dist2 * flywheel2
            - 0.11970122446327 * dist * inv_term;
    if den.abs() < 1e-9 {
        return 0.0;
    }
    num / den
}

fn predict_speed(dist: f64, vx: f64, vy: f64) -> f64 {
    let vx2 = vx * vx;
    let vy2 = vy * vy;
    let vxvy2 = vx2 + vy2;
    let dist2 = dist * dist;
    let speed = 37.48768924052001 + 1.85950428954673 * dist + 0.99263955984153 * dist2
        - 0.29979616450495 * vy
        - 0.06945847505217 * vy * dist
        - 0.22288224790744 * vy * dist2
        + 0.22589500122137 * vxvy2
        + 0.11034899634029 * vxvy2 * dist;
    speed.clamp(30.0, 90.0)
}
