use crate::constants::config::MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND;
use crate::constants::robotmap::intake::{HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};

use crate::debouncer::Debouncer;
use crate::subsystems::intake::Intake;
use crate::subsystems::shooter::Shooter;
use crate::subsystems::swerve::drivetrain::Drivetrain;
use frcrs::input::{Joystick, RobotState};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;
use tokio::time::Instant;

pub mod constants;
pub mod control;
pub mod debouncer;
pub mod subsystems;

#[derive(Clone)]
pub struct Controllers {
    pub left_drive: Joystick,
    pub right_drive: Joystick,
    pub operator: Joystick,
}

pub struct Ferris {
    pub controllers: Controllers,
    pub turret_toggle_debouncer: Debouncer,

    pub start_time: Instant,

    pub drivetrain: Rc<RefCell<Drivetrain>>,
    pub shooter: Rc<RefCell<Shooter>>,
    pub intake: Rc<RefCell<Intake>>,

    pub shooter_offset: f64,
    pub dt: Duration,
    pub state: RobotState,
    pub shooter_enabled: bool,
}

impl Default for Ferris {
    fn default() -> Self {
        Self::new()
    }
}

impl Ferris {
    pub fn new() -> Self {
        Ferris {
            controllers: Controllers {
                left_drive: Joystick::new(constants::joystick_map::LEFT_DRIVE),
                right_drive: Joystick::new(constants::joystick_map::RIGHT_DRIVE),
                operator: Joystick::new(constants::joystick_map::OPERATOR),
            },
            turret_toggle_debouncer: Debouncer::new(),

            start_time: Instant::now(),

            // todo: figure out start pose stuff
            // temp hardcoded startpose
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            shooter: Rc::new(RefCell::new(Shooter::new())),
            intake: Rc::new(RefCell::new(Intake::new())),

            shooter_offset: 0.0,

            dt: Duration::from_millis(0),
            state: RobotState::get(),
            shooter_enabled: false,
        }
    }

    pub fn update_state(&mut self) {
        self.state = RobotState::get();
    }

    pub fn stop(&self) {
        if let Ok(drivetrain) = self.drivetrain.try_borrow() {
            drivetrain.stop();
        }
        if let Ok(intake) = self.intake.try_borrow() {
            intake.stop();
        }
        if let Ok(shooter) = self.shooter.try_borrow() {
            shooter.stop();
        }
    }
}
