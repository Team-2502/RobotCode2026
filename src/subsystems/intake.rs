use std::{f64::consts::PI, time::Instant};

use crate::constants::robotmap::intake::{
    HANDOFF_MOTOR_ID, INDEXER_MOTOR_ID, INTAKE_BOTTOM_MOTOR_ID, INTAKE_SPEED_OSCILLATION_TIME_SECS,
    INTAKE_TOP_MOTOR_ID,
};
use frcrs::ctre::{ControlMode, Talon};

pub struct Intake {
    intake_top: Talon,
    intake_bottom: Talon,
    indexer_motor: Talon,
    handoff_motor: Talon,
    timer: Instant,
}

impl Intake {
    pub fn new() -> Self {
        let intake_top = Talon::new(INTAKE_TOP_MOTOR_ID, None);
        let intake_bottom = Talon::new(INTAKE_BOTTOM_MOTOR_ID, None);
        let indexer_motor = Talon::new(INDEXER_MOTOR_ID, None);
        let handoff_motor = Talon::new(HANDOFF_MOTOR_ID, None);

        Intake {
            intake_top,
            intake_bottom,
            indexer_motor,
            handoff_motor,
            timer: Instant::now(),
        }
    }

    pub fn set_intake_speed(&self, speed: f64) {
        let cos = f64::cos(
            Instant::now().duration_since(self.timer).as_secs_f64()
                * INTAKE_SPEED_OSCILLATION_TIME_SECS
                * PI,
        );
        let oscillated_speed = speed * 0.5 + 0.5 * speed * cos * cos;
        self.intake_top.set(ControlMode::Percent, -oscillated_speed);
        self.intake_bottom
            .set(ControlMode::Percent, oscillated_speed);
    }

    pub fn set_handoff(&self, speed: f64) {
        self.indexer_motor.set(ControlMode::Percent, speed);
        self.handoff_motor.set(ControlMode::Percent, -speed);
    }

    pub fn stop(&self) {
        self.intake_top.stop();
        self.intake_bottom.stop();
        self.indexer_motor.stop();
        self.handoff_motor.stop();
    }
}
