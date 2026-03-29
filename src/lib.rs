// use crate::auto::path::drive;
use crate::auto::path::Auto;
use crate::auto::path::mirror_vec;
use crate::constants::config::{
    BLUE_PASS_BOTTOM_OFFSET_METERS, BLUE_PASS_TOP_OFFSET_METERS, HUB_BLUE, HUB_RED,
    MANUAL_TURRET_MODE_DISTANCE_MAX_METERS, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND,
    RED_PASS_BOTTOM_OFFSET_METERS, RED_PASS_TOP_OFFSET_METERS,
};
use crate::constants::robotmap::intake::{HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use crate::control::swerve::Swerve;
use crate::debouncer::Debouncer;
use crate::subsystems::intake::Intake;
use crate::subsystems::shooter::{
    Shooter, ShootingTarget, get_drivetrain_max_speed, get_scoring_hood_angle_target,
    get_scoring_shooter_speed_target,
};
use crate::subsystems::swerve::drivetrain::FieldZone::{
    BlueBottom, BlueTop, MiddleBottom, MiddleTop, RedBottom, RedTop,
};
use crate::subsystems::swerve::drivetrain::{Drivetrain, get_zone, update_drivetrain_telemetry};
use crate::subsystems::turret::TurretMode;
use crate::subsystems::vision::distance;
use frcrs::alliance_shift;
use frcrs::input::{Joystick, RobotState};
use frcrs::telemetry::Telemetry;
use frcrs::{alliance_station, deadzone};
use nalgebra::Vector2;
use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;
use std::time::Duration;
use tokio::join;
use tokio::time::Instant;
use uom::si::angle::{degree, radian};
use uom::si::f64::{Angle, Length};
use uom::si::length::{foot, inch, meter};

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
