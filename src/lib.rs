// use crate::auto::path::drive;
use crate::auto::path::Auto;

use crate::constants::config::MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND;
use crate::constants::robotmap::intake::{HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};

use crate::debouncer::Debouncer;
use crate::subsystems::intake::Intake;
use crate::subsystems::shooter::{Shooter, get_drivetrain_max_speed};
use crate::subsystems::swerve::drivetrain::{Drivetrain, update_drivetrain_telemetry};
use frcrs::alliance_shift;
use frcrs::input::{Joystick, RobotState};
use frcrs::telemetry::Telemetry;
use nalgebra::Vector2;
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;
use tokio::time::Instant;
use uom::si::f64::Length;
use uom::si::length::meter;

pub mod auto;
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
    pub idle_toggle_debouncer: Debouncer,
    pub man_toggle_debouncer: Debouncer,

    pub drivetrain: Rc<RefCell<Drivetrain>>,
    pub shooter: Rc<RefCell<Shooter>>,
    pub intake: Rc<RefCell<Intake>>,

    pub shooter_offset: f64,
    pub dt: Duration,
    pub state: RobotState,

    pub auto: Auto,
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
            idle_toggle_debouncer: Debouncer::new(),
            man_toggle_debouncer: Debouncer::new(),

            // todo: figure out start pose stuff
            // temp hardcoded startpose
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            shooter: Rc::new(RefCell::new(Shooter::new())),
            intake: Rc::new(RefCell::new(Intake::new())),

            shooter_offset: 0.0,

            dt: Duration::from_millis(0),
            state: RobotState::get(),

            auto: Auto::new(),
        }
    }

    pub fn update_state(&mut self) {
        self.state = RobotState::get();
    }

    pub fn auto_init(&mut self) {
        self.auto.start_time = Instant::now();
    }

    pub async fn auto_periodic(&mut self) {
        if let Ok(mut drivetrain) = self.drivetrain.try_borrow_mut() {
            drivetrain.update_pose().await;
            let pose = drivetrain.localization.get_state();
            update_drivetrain_telemetry(&pose).await;

            self.auto
                .move_to_sample("test_triangle", &mut drivetrain, self.auto.current_sample)
                .await;
        }
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

pub async fn post_shift(match_time: f64) {
    if match_time <= 30.0 || match_time >= 130.0 {
        Telemetry::put_color("shift", frcrs::telemetry::TelemetryColor::Purple).await;
    } else {
        let flip = if alliance_shift().red() { 1 } else { 0 };
        let shift = ((130 - match_time as i32) / 25 + flip) % 2;
        match shift {
            0 => {
                Telemetry::put_color("shift", frcrs::telemetry::TelemetryColor::Red).await;
            }
            _ => {
                Telemetry::put_color("shift", frcrs::telemetry::TelemetryColor::Blue).await;
            }
        }
    }
}

pub fn square_vec(vec: Vector2<f64>) -> Vector2<f64> {
    Vector2::new(vec.x * vec.x, vec.y * vec.y)
}

pub fn vec_f64(vec: Vector2<Length>) -> Vector2<f64> {
    Vector2::new(vec.x.get::<meter>(), vec.y.get::<meter>())
}
