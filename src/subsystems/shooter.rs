use crate::constants::robotmap::shooter::{
    HOOD_MOTOR_ID, SHOOTER_CANBUS, SHOOTER_MOTOR_LEFT_ID, SHOOTER_MOTOR_RIGHT_ID,
};
use crate::constants::turret::{HOOD_MAX_SOFTSTOP, HOOD_MIN_SOFTSTOP};
use crate::subsystems::turret::Turret;

use frcrs::ctre::{ControlMode, Talon};
use std::fs::File;
use std::io::{Read, Write};
use uom::si::angle::revolution;
use uom::si::f64::Angle;

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
            hood_offset: hood_offset.unwrap(),

            turret,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Percent, speed);
        self.shooter_motor_right.set(ControlMode::Percent, -speed);
    }

    pub fn get_hood(&self) -> f64 {
        self.hood_motor.get_position()
    }

    pub fn set_hood(&mut self, angle: f64) {
        self.hood_motor.set(
            ControlMode::Position,
            angle.clamp(HOOD_MIN_SOFTSTOP, HOOD_MAX_SOFTSTOP)
                + self.hood_offset.get::<revolution>(),
        );
    }

    pub fn set_velocity(&self, speed: f64) {
        let scaled_speed = speed.clamp(10.0, 100.0);
        self.shooter_motor_left
            .set(ControlMode::Velocity, scaled_speed);
        self.shooter_motor_right
            .set(ControlMode::Velocity, scaled_speed);
    }

    pub fn stop(&self) {
        self.hood_motor.stop();
        self.shooter_motor_left.stop();
        self.shooter_motor_right.stop();
        self.turret.stop();
    }
}
