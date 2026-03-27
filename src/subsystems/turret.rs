use crate::constants::config::MANUAL_TURRET_YAW_CHANGE_SCALAR;
use crate::constants::robotmap::drivetrain_map::DRIVETRAIN_CANBUS;
use crate::constants::robotmap::shooter::SHOOTER_CANBUS;
use crate::constants::robotmap::turret::{ENCODER_ID, SPIN_MOTOR_ID};
use crate::constants::turret::{
    GEAR_RATIO, ORIGIN_TO_TURRET_CENTER_X_INCHES, ORIGIN_TO_TURRET_CENTER_Y_INCHES,
    TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS, TURRET_CLAMP,
};
use crate::subsystems::localization::RobotPose;
use frcrs::alliance_station;
use frcrs::ctre::{CanCoder, ControlMode, Talon};
use nalgebra::{Rotation2, Vector2};
use uom::si::angle::radian;
use uom::si::angle::{degree, revolution};
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::length::{inch, meter};

#[derive(PartialEq, Clone)]
pub enum TurretMode {
    Track,
    Manual,
    Idle,
    Test,
}

pub struct Turret {
    spin_motor: Talon,
    drivetrain_angle: Angle,
    encoder: CanCoder,

    pub turret_angle: Angle,
    pub desired_angle: Angle,
    pub yaw_offset: Angle,
}

impl TurretMode {
    pub fn name(&self) -> &'static str {
        match self {
            TurretMode::Idle => "idle",
            TurretMode::Manual => "man",
            TurretMode::Track => "track",
            TurretMode::Test => "test",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![
            TurretMode::Idle,
            TurretMode::Test,
            TurretMode::Manual,
            TurretMode::Track,
        ]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

    pub fn to_mode(s: &str) -> Self {
        match s {
            "idle" => TurretMode::Idle,
            "man" => TurretMode::Manual,
            "track" => TurretMode::Track,
            "test" => TurretMode::Test,
            _ => TurretMode::Idle,
        }
    }
}

impl Turret {
    pub fn new() -> Self {
        let spin_motor = Talon::new(SPIN_MOTOR_ID, DRIVETRAIN_CANBUS);
        let encoder = CanCoder::new(ENCODER_ID, Some(SHOOTER_CANBUS.to_string()));

        Turret {
            spin_motor,
            drivetrain_angle: Angle::new::<degree>(0.0),
            encoder,

            turret_angle: Angle::new::<degree>(0.0),
            desired_angle: Angle::new::<degree>(0.0),
            yaw_offset: Angle::new::<degree>(0.0),
        }
    }

    pub fn update_turret(&mut self, drivetrain_angle: Angle) {
        self.drivetrain_angle = drivetrain_angle;
    }

    pub fn move_to_angle(&self, angle: Angle) {
        let position = self.spin_motor.get_position();

        let target_rot = (apply_soft_stop(angle).get::<revolution>() * GEAR_RATIO)
            .clamp(position - TURRET_CLAMP, position + TURRET_CLAMP);
        self.spin_motor.set(ControlMode::Position, target_rot);
    }

    pub fn set_angle(&mut self, robot_turret_angle: Angle) {
        self.move_to_angle(robot_turret_angle);
    }

    pub fn set_speed(&self, speed: f64) {
        self.spin_motor.set(ControlMode::Percent, speed);
    }

    pub fn offset_yaw(&mut self, amount: Angle) {
        self.yaw_offset = self.yaw_offset + amount;
    }

    pub fn man_yaw(&mut self, mut joystick: f64) {
        if alliance_station().blue() {
            joystick = -joystick;
        }
        let angle = self.turret_angle.get::<degree>() + MANUAL_TURRET_YAW_CHANGE_SCALAR * joystick;
        // println!("here: {}", angle);

        self.move_to_angle(Angle::new::<degree>(angle));
        // println!("moved? {}", self.apply_soft_stop(angle));
    }

    pub fn slow(&self, angle1: f64, angle2: f64) {
        if (angle1 - angle2).abs() > 30.0 {}
    }

    pub fn stop(&self) {
        self.spin_motor.stop();
    }
}

pub fn get_angle_to(pose: Vector2<Length>, target: Vector2<Length>) -> Angle {
    let x = pose.x.get::<meter>();
    let y = pose.y.get::<meter>();
    let dx = target.x.get::<meter>() - x;
    let dy = target.y.get::<meter>() - y;

    Angle::new::<degree>(dy.atan2(dx).to_degrees())
}

pub fn apply_soft_stop(desired_deg: Angle) -> Angle {
    Angle::new::<radian>(f64::atan2(
        desired_deg.get::<radian>().sin(),
        desired_deg.get::<radian>().cos(),
    ))
}

pub fn get_turret_velocity(
    linear_velocity: Vector2<Length>,
    angular_velocity: Angle,
    pose: &RobotPose,
) -> Vector2<Length> {
    let origin_to_turret_center_dist = (Length::new::<inch>(ORIGIN_TO_TURRET_CENTER_X_INCHES)
        .get::<meter>()
        * Length::new::<inch>(ORIGIN_TO_TURRET_CENTER_X_INCHES).get::<meter>()
        + Length::new::<inch>(ORIGIN_TO_TURRET_CENTER_Y_INCHES).get::<meter>()
            * Length::new::<inch>(ORIGIN_TO_TURRET_CENTER_Y_INCHES).get::<meter>())
    .sqrt();

    let field_velocity_f64 = Rotation2::new(pose.yaw.get::<radian>())
        * Vector2::new(
            0.0,
            origin_to_turret_center_dist * angular_velocity.get::<radian>(),
        );

    Vector2::new(
        Length::new::<meter>(field_velocity_f64.x),
        Length::new::<meter>(field_velocity_f64.y),
    ) + linear_velocity
}

#[cfg(test)]
mod tests {
    // #[test]
    // pub fn test_angle_to_hub() {
    //     let pose = RobotPoseEstimate::new(
    //         1.,
    //         Length::new::<meter>(-1.),
    //         Length::new::<meter>(-1.),
    //         Angle::new::<radian>(0.),
    //     );

    //     let result = get_angle_to_hub(pose).get::<degree>();
    //     let expected = Angle::new::<degree>(45.);

    //     println!("results: {:?}", result);
    //     println!("expected: {:?}", expected.get::<degree>());

    //     assert_eq!(result, expected.get::<degree>());
    // }
}
