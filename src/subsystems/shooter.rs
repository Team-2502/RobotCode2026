use crate::constants::robotmap::shooter::{HOOD_MOTOR_ID, SHOOTER_MOTOR_ID, SHOOTER_SPEED};
use frcrs::ctre::{ControlMode, Talon};

pub struct Shooter {
    shooter_motor: Talon,
    hood_motor: Talon,
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor = Talon::new(SHOOTER_MOTOR_ID, Some("can0".to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some("can1".to_string()));
        Shooter {
            shooter_motor,
            hood_motor,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor.set(ControlMode::Percent, speed);
    }

    pub fn shoot(&self, on: bool) {
        if on {
            self.shooter_motor.set(ControlMode::Percent, SHOOTER_SPEED);
        } else {
            self.shooter_motor.set(ControlMode::Percent, 0.0);
        }
    }

    pub fn set_hood_motor(&mut self, position: f64) {
        self.hood_motor.set(ControlMode::Position, position);
    }
}
