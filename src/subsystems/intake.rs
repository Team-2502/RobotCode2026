use crate::constants::robotmap::intake::{INTAKE_BOTTOM_MOTOR_ID, INTAKE_TOP_MOTOR_ID};
use frcrs::ctre::{ControlMode, Talon};

pub struct Intake {
    intake_top: Talon,
    intake_bottom: Talon,
    pivot_top: Talon,
    pivot_bottom: Talon,
    indexer_motor: Talon,
}

impl Intake {
    pub fn new() -> Self {
        let intake_top = Talon::new(INTAKE_TOP_MOTOR_ID, Some("can0".to_string()));
        let intake_bottom = Talon::new(INTAKE_BOTTOM_MOTOR_ID, Some("can0".to_string()));
        let pivot_top = Talon::new(INTAKE_TOP_MOTOR_ID, Some("can0".to_string()));
        let pivot_bottom = Talon::new(INTAKE_BOTTOM_MOTOR_ID, Some("can0".to_string()));
        let indexer_motor = Talon::new(INTAKE_BOTTOM_MOTOR_ID, Some("can0".to_string()));

        Intake {
            intake_top,
            intake_bottom,
            pivot_top,
            pivot_bottom,
            indexer_motor,
        }
    }

    pub fn set_intake_speed(&self, speed: f64) {
        self.intake_top.set(ControlMode::Percent, speed);
        self.intake_bottom.set(ControlMode::Percent, speed);
        self.indexer_motor.set(ControlMode::Percent, speed);
    }

    pub fn set_pivot_position(&self, position: f64) {
        self.pivot_top.set(ControlMode::Position, position);
        self.pivot_bottom.set(ControlMode::Position, position);
    }

    pub fn stop(&self) {
        self.intake_top.stop();
        self.intake_bottom.stop();
        self.pivot_top.stop();
        self.pivot_bottom.stop();
        self.indexer_motor.stop();
    }
}
