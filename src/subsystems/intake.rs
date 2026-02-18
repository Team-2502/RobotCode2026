use crate::constants::robotmap::intake::{HANDOFF_MOTOR_ID, HANDOFF_SPEED, INDEXER_MOTOR_ID, INTAKE_BOTTOM_MOTOR_ID, INTAKE_DOWN_POSITION, INTAKE_IN_SPEED, INTAKE_TOP_MOTOR_ID, INTAKE_UP_POSITION};
use frcrs::ctre::{ControlMode, Talon};

pub struct Intake {
    intake_top: Talon,
    intake_bottom: Talon,
    pivot_top: Talon,
    pivot_bottom: Talon,
    indexer_motor: Talon,
    handoff_motor: Talon,
}

impl Intake {
    pub fn new() -> Self {
        let intake_top = Talon::new(INTAKE_TOP_MOTOR_ID, None);
        let intake_bottom = Talon::new(INTAKE_BOTTOM_MOTOR_ID, None);
        let pivot_top = Talon::new(INTAKE_TOP_MOTOR_ID, None);
        let pivot_bottom = Talon::new(INTAKE_BOTTOM_MOTOR_ID, None);
        let indexer_motor = Talon::new(INDEXER_MOTOR_ID, None);
        let handoff_motor = Talon::new(HANDOFF_MOTOR_ID, None);

        Intake {
            intake_top,
            intake_bottom,
            pivot_top,
            pivot_bottom,
            indexer_motor,
            handoff_motor,
        }
    }

    pub fn set_intake_speed(&self, speed: f64) {
        self.intake_top.set(ControlMode::Percent, -speed);
        self.intake_bottom.set(ControlMode::Percent, speed);
        //self.indexer_motor.set(ControlMode::Percent, speed);
    }
    
    pub fn set_handoff(&self, speed: f64) {
        self.indexer_motor.set(ControlMode::Percent, speed);
        self.handoff_motor.set(ControlMode::Percent, -speed);
    }

    pub fn set_pivot_position(&self, position: f64) {
        self.pivot_top.set(ControlMode::Position, position);
        self.pivot_bottom.set(ControlMode::Position, position);
    }
    
    pub fn intake(&self, deployed: bool) {
        match deployed {
            true => {
                self.set_pivot_position(INTAKE_DOWN_POSITION);
                self.set_intake_speed(INTAKE_IN_SPEED);
                self.set_handoff(HANDOFF_SPEED);
            }
            false => {
                self.set_pivot_position(INTAKE_UP_POSITION);
                self.set_intake_speed(0.0);
                self.set_handoff(0.0);
            }
        }
    }

    pub fn stop(&self) {
        self.intake_top.stop();
        self.intake_bottom.stop();
        self.pivot_top.stop();
        self.pivot_bottom.stop();
        self.indexer_motor.stop();
        self.handoff_motor.stop();
    }
}
