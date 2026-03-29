use crate::constants::config::{
    MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND, SHOOTER_INITAL_DISTANCE_OFFSET_FEET,
};
use crate::constants::robotmap::shooter::{
    HOOD_MOTOR_ID, SHOOTER_CANBUS, SHOOTER_MOTOR_LEFT_ID, SHOOTER_MOTOR_RIGHT_ID,
};
use crate::constants::shooter::{MAX_FLYWHEEL_SPEED, SHOOTER_DISTANCE_ERROR_SMUDGE};
use crate::constants::turret::{
    HOOD_MAX_SOFTSTOP, HOOD_MIN_SOFTSTOP, HOOD_ZERO_POSE, ORIGIN_TO_TURRET_CENTER_X_INCHES,
    ORIGIN_TO_TURRET_CENTER_Y_INCHES,
};
use crate::subsystems::localization::RobotPose;
use crate::subsystems::swerve::drivetrain::get_angle_difs;
use crate::subsystems::turret::{Turret, get_angle_to};
use crate::subsystems::vision::distance;
use frcrs::ctre::{ControlMode, Talon};
use frcrs::telemetry::Telemetry;
use nalgebra::{Rotation2, Vector2};
use uom::si::angle::{degree, radian};
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
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor_left =
            Talon::new(SHOOTER_MOTOR_LEFT_ID, Some(SHOOTER_CANBUS.to_string()));
        let shooter_motor_right =
            Talon::new(SHOOTER_MOTOR_RIGHT_ID, Some(SHOOTER_CANBUS.to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some(SHOOTER_CANBUS.to_string()));

        let turret = Turret::new();

        Shooter {
            shooter_motor_left,
            shooter_motor_right,
            hood_motor,
            distance_offset: Length::new::<foot>(SHOOTER_INITAL_DISTANCE_OFFSET_FEET),

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
            (angle + HOOD_ZERO_POSE).clamp(HOOD_MIN_SOFTSTOP, HOOD_MAX_SOFTSTOP),
        );
    }

    pub fn shoot_to(&mut self, pose: &RobotPose, target: Vector2<Length>) {
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
        println!(
            "shooter::shoot_to: angle target: {}, turret_rel_angle (set_to_call): {}",
            angle_target.get::<degree>(),
            turret_relative_angle.get::<degree>()
        );

        let current_flywheel_speed = self.get_speed();
        self.set_velocity(get_scoring_shooter_speed_target(distance_target));
        self.set_hood(get_scoring_hood_angle_target(
            distance_target,
            current_flywheel_speed,
        ));

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
        self.shooter_motor_left.get_velocity()
    }

    pub fn set_velocity(&self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Velocity, speed);
        self.shooter_motor_right.set(ControlMode::Velocity, speed);
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

pub fn get_drivetrain_max_speed(
    robot_pose: &RobotPose,
    robot_velocity: Vector2<Length>,
    target: Vector2<Length>,
) -> Length {
    let hub_centric_velocity =
        get_target_relative_velocity_vector(robot_velocity, &robot_pose, target);
    scale_drivetrain_speed(hub_centric_velocity)
}

fn scale_drivetrain_speed(hub_centric_velocity: Vector2<Length>) -> Length {
    let angle = Angle::new::<radian>(
        hub_centric_velocity
            .y
            .get::<meter>()
            .atan2(hub_centric_velocity.x.get::<meter>()),
    );

    let abs_angle_diff = get_angle_difs(Angle::new::<degree>(90.0), angle).abs();

    if abs_angle_diff < Angle::new::<degree>(45.0) {
        let percent = abs_angle_diff.get::<degree>() / 45.0;
        Length::new::<meter>(1.0 + (MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND - 1.0) * percent)
    } else {
        println!("{}", MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND);
        Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND)
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

#[cfg(test)]
mod shooter_tests {
    use float_cmp::assert_approx_eq;

    use super::*;

    #[test]
    fn get_target_relative_velocity_vector_test() {
        // (vector.x, vector.y), (pose.x, pose.y, pose.yaw), (target.x, target.y)
        let inputs = vec![
            ((10.0, 10.0), (10.0, 0.0, 10.0), (0.0, 0.0)),    // 1
            ((-10.0, 10.0), (10.0, 0.0, 10.0), (0.0, 0.0)),   // 2
            ((-10.0, -10.0), (10.0, 0.0, 10.0), (0.0, 0.0)),  // 3
            ((10.0, -10.0), (10.0, 0.0, 10.0), (0.0, 0.0)),   // 4
            ((10.0, 10.0), (10.0, 10.0, 10.0), (0.0, 0.0)),   // 5
            ((-10.0, 10.0), (10.0, 10.0, 10.0), (0.0, 0.0)),  // 6
            ((-10.0, -10.0), (10.0, 10.0, 10.0), (0.0, 0.0)), // 7
            ((10.0, -10.0), (10.0, 10.0, 10.0), (0.0, 0.0)),  // 8
            ((10.0, 10.0), (0.0, 0.0, 0.0), (5.0, 5.0)),      // 9
            ((10.0, 10.0), (0.0, 0.0, 0.0), (-5.0, -5.0)),    // 10
        ];

        // (vector.x, vector.y)
        let expected = vec![
            (10.0, -10.0),         // 1
            (10.0, 10.0),          // 2
            (-10.0, 10.0),         // 3
            (-10.0, -10.0),        // 4
            (0.0, -14.1421356237), // 5
            (14.1421356237, 0.0),  // 6
            (0.0, 14.1421356237),  // 7
            (-14.1421356237, 0.0), // 8
            (0.0, 14.1421356237),  // 9
            (0.0, -14.1421356237), // 10
        ];

        let mut results: Vec<Vector2<Length>> = vec![];

        for input in inputs {
            let vector = Vector2::new(
                Length::new::<meter>(input.0.0),
                Length::new::<meter>(input.0.1),
            );

            let robot_pose = RobotPose {
                x: Length::new::<meter>(input.1.0),
                y: Length::new::<meter>(input.1.1),
                yaw: Angle::new::<degree>(input.1.2),
            };

            let target = Vector2::new(
                Length::new::<meter>(input.2.0),
                Length::new::<meter>(input.2.1),
            );

            results.push(get_target_relative_velocity_vector(
                vector,
                &robot_pose,
                target,
            ));
        }

        let mut i = 1;
        for result in &results {
            println!(
                "result {}: ({}, {})",
                i,
                result.x.get::<meter>(),
                result.y.get::<meter>()
            );
            i += 1;
        }

        for tuple in expected.iter().zip(results.iter()) {
            assert_approx_eq!(f64, tuple.0.0, tuple.1.x.get::<meter>(), epsilon = 0.001);
            assert_approx_eq!(f64, tuple.0.1, tuple.1.y.get::<meter>(), epsilon = 0.001);
        }
    }

    #[test]
    fn scale_drivetrain_speed_test() {
        let inputs = vec![
            (0.0, 10.0),              // 1
            (10.0, 10.0),             // 2
            (10.0, 0.0),              // 3
            (10.0, -10.0),            // 4
            (-10.0, 0.0),             // 5
            (-10.0, -10.0),           // 6
            (-10.0, 0.0),             // 7
            (-10.0, 10.0),            // 8
            (3.82683432, 9.23879533), // 9
        ];

        // (vector.x, vector.y)
        let expected: Vec<f64> = vec![
            1.0, // 1
            6.0, // 2
            6.0, // 3
            6.0, // 4
            6.0, // 5
            6.0, // 6
            6.0, // 7
            6.0, // 8
            3.5, // 9
        ];

        let mut results: Vec<Length> = vec![];

        for input in inputs {
            let vector = Vector2::new(Length::new::<meter>(input.0), Length::new::<meter>(input.1));

            results.push(scale_drivetrain_speed(vector));
        }

        let mut i = 1;
        for result in &results {
            println!("result {}: {}", i, result.get::<meter>());
            i += 1;
        }

        for tuple in expected.iter().zip(results.iter()) {
            assert_approx_eq!(
                f64,
                tuple.0.to_owned(),
                tuple.1.get::<meter>(),
                epsilon = 0.001
            );
        }
    }
}
