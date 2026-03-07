use crate::constants::robotmap::shooter::{
    HOOD_MOTOR_ID, SHOOTER_MOTOR_LEFT_ID, SHOOTER_MOTOR_RIGHT_ID,
};
use crate::constants::shooter::{MAX_FLYWHEEL_SPEED, SHOOTER_DISTANCE_ERROR_SMUDGE};

use crate::subsystems::swerve::drivetrain::get_angle_difs;

use crate::subsystems::turret::{Turret, get_angle_to_hub};
use crate::subsystems::vision::distance;

use frcrs::ctre::{ControlMode, Talon};

use nalgebra::Vector2;

use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::length::meter;

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
    pub manual_toggle: bool,
    pub idle_toggle: bool,

    pub turret: Turret,
}
impl ShootingTarget {
    pub fn name(&self) -> &'static str {
        match self {
            ShootingTarget::Hub => "hub",
            ShootingTarget::PassTop => "pass_l",
            ShootingTarget::PassBottom => "pass_r",
            ShootingTarget::PassTelemetry => "telem",
        }
    }
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor_left = Talon::new(SHOOTER_MOTOR_LEFT_ID, Some("can0".to_string()));
        let shooter_motor_right = Talon::new(SHOOTER_MOTOR_RIGHT_ID, Some("can0".to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some("can0".to_string()));

        let turret = Turret::new();

        Shooter {
            shooter_motor_left,
            shooter_motor_right,
            hood_motor,
            distance_offset: Length::new::<meter>(0.0),
            manual_toggle: false,
            idle_toggle: false,

            turret,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Percent, speed);
        self.shooter_motor_right.set(ControlMode::Percent, -speed);
    }

    // pub fn shoot(&self, on: bool) {
    //     if on {
    //         self.shooter_motor.set(ControlMode::Percent, SHOOTER_SPEED);
    //     } else {
    //         self.shooter_motor.set(ControlMode::Percent, 0.0);
    //     }
    // }

    pub fn set_hood(&mut self, angle: f64) {
        self.hood_motor.set(ControlMode::Position, angle);
    }

    pub fn shoot_to(
        &mut self,
        current_pose: Vector2<Length>,
        current_yaw: Angle,
        target: Vector2<Length>,
    ) {
        let distance_target = Length::new::<meter>(distance(target, current_pose));

        let current_flywheel_speed = self.get_speed();
        self.set_velocity(get_shooter_speed_target(
            distance_target + self.distance_offset,
        ));
        self.set_hood(get_hood_angle_target(
            distance_target + self.distance_offset,
            current_flywheel_speed,
        ));
        self.turret.set_angle(get_angle_difs(
            current_yaw,
            get_angle_to_hub(current_pose) + self.turret.yaw_offset,
        ))
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

pub fn get_shooter_speed_target(distance: Length) -> f64 {
    let distance_feet: f64 =
        distance.get::<meter>() as f64 * 3.28084 * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let target =
        (0.0652772 * (distance_feet * distance_feet)) + (0.954121 * distance_feet) + 38.92606;

    target.clamp(0.0, MAX_FLYWHEEL_SPEED)
}

pub fn get_hood_angle_target(distance: Length, current_speed: f64) -> f64 {
    let distance_feet: f64 =
        distance.get::<meter>() as f64 * 3.28084 * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let min_speed =
        (0.0917639 * (distance_feet * distance_feet)) + (-0.53771 * distance_feet) + 46.28489;
    let max_speed =
        (0.0496249 * (distance_feet * distance_feet)) + (1.84238 * distance_feet) + 34.54472;
    let max_angle =
        (-0.0076574 * (distance_feet * distance_feet)) + (0.24567 * distance_feet) + -0.978109;

    if max_angle < 0.0 || current_speed > max_speed {
        0.0
    } else if current_speed < min_speed {
        max_angle
    } else {
        let t = 1.0 - (current_speed - min_speed) / (max_speed - min_speed);
        max_angle * t
    }
}
