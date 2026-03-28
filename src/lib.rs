// use crate::auto::path::drive;
use crate::auto::path::Auto;
use crate::auto::path::mirror_vec;
use crate::constants::config::{
    BLUE_PASS_BOTTOM_OFFSET_METERS, BLUE_PASS_TOP_OFFSET_METERS, HUB_BLUE, HUB_RED,
    MANUAL_TURRET_MODE_DISTANCE_MAX_METERS, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND,
    RED_PASS_BOTTOM_OFFSET_METERS, RED_PASS_TOP_OFFSET_METERS,
};
use crate::constants::robotmap::intake::{HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
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
pub mod input;
pub mod subsystems;

#[derive(Clone)]
pub struct Controllers {
    pub left_drive: Joystick,
    pub right_drive: Joystick,
    pub operator: Joystick,
}

#[derive(Clone)]
pub struct Ferris {
    pub controllers: Controllers,

    pub drivetrain: Rc<RefCell<Drivetrain>>,
    pub shooter: Rc<RefCell<Shooter>>,
    pub intake: Rc<RefCell<Intake>>,

    pub shooter_target: ShootingTarget,
    pub turret_mode: TurretMode,

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

            // todo: figure out start pose stuff
            // temp hardcoded startpose
            drivetrain: Rc::new(RefCell::new(Drivetrain::new())),
            shooter: Rc::new(RefCell::new(Shooter::new())),
            intake: Rc::new(RefCell::new(Intake::new())),

            shooter_target: ShootingTarget::Hub,
            turret_mode: TurretMode::Idle,

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
            let (linear_velocity, angular_velocity) = drivetrain.localization.get_velocities();
            update_drivetrain_telemetry(&pose, &linear_velocity, &angular_velocity).await;

            //println!("auto");
            self.auto
                .move_to_sample("test_triangle", &mut drivetrain, self.auto.current_sample)
                .await;
            //self.auto.set_target(mirror_vec(Vector2::new(Length::new::<meter>(4.74182), Length::new::<meter>(5.88162))));
            //"x":4.74182, "y":5.88162
            //self.auto.move_to(&mut drivetrain, 0.0, "test_triangle", 0).await;
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

pub async fn teleop(ferris: &mut Ferris) {
    if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
        drivetrain.update_pose().await;
        let pose = drivetrain.localization.get_state();
        let (linear_velocity, angular_velocity) = drivetrain.localization.get_velocities();
        update_drivetrain_telemetry(&pose, &linear_velocity, &angular_velocity).await;

        // shooter state can scale drivetrain speeds. therefore, run shooter functions before rest of dt things.
        let mut max_dt_speed = Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND);
        // let turret_velocity = get_turret_velocity(linear_velocity, angular_velocity, &pose);
        let turret_velocity = linear_velocity;

        // Run Shooter Functions
        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            shooter.turret.update_turret(pose.yaw);

            if ferris.controllers.operator.get(6) {
                shooter.distance_offset += Length::new::<inch>(0.5);
            }

            if ferris.controllers.operator.get(9) {
                shooter.distance_offset -= Length::new::<inch>(0.5);
            }

            if ferris.controllers.operator.get(10) {
                shooter.turret.yaw_offset += Angle::new::<degree>(0.5);
            }

            if ferris.controllers.operator.get(8) {
                shooter.turret.yaw_offset -= Angle::new::<degree>(0.5);
            }

            if ferris.controllers.operator.get(11) {
                if shooter.manual_toggle {
                    shooter.manual_toggle = false;
                    shooter.idle_toggle = false;
                    ferris.turret_mode = TurretMode::Track;
                } else {
                    shooter.manual_toggle = true;
                    shooter.idle_toggle = false;
                    ferris.turret_mode = TurretMode::Manual;
                }
            }

            if ferris.controllers.operator.get(16) {
                if shooter.idle_toggle {
                    shooter.manual_toggle = false;
                    shooter.idle_toggle = false;
                    ferris.turret_mode = TurretMode::Track;
                } else {
                    shooter.idle_toggle = false;
                    shooter.idle_toggle = true;
                    ferris.turret_mode = TurretMode::Idle;
                }
            }

            // get targets based on alliance
            let (hub, pass_top, pass_bottom) = match alliance_station().red() {
                true => {
                    let hub = Vector2::new(
                        Length::new::<meter>(HUB_RED.x),
                        Length::new::<meter>(HUB_RED.y),
                    );
                    let pass_top = hub
                        + Vector2::new(
                            Length::new::<meter>(RED_PASS_TOP_OFFSET_METERS.x),
                            Length::new::<meter>(RED_PASS_TOP_OFFSET_METERS.y),
                        );
                    let pass_bottom = hub
                        + Vector2::new(
                            Length::new::<meter>(RED_PASS_BOTTOM_OFFSET_METERS.x),
                            Length::new::<meter>(RED_PASS_BOTTOM_OFFSET_METERS.y),
                        );
                    (hub, pass_top, pass_bottom)
                }
                false => {
                    let hub = Vector2::new(
                        Length::new::<meter>(HUB_BLUE.x),
                        Length::new::<meter>(HUB_BLUE.y),
                    );
                    let pass_top = hub
                        + Vector2::new(
                            Length::new::<meter>(BLUE_PASS_TOP_OFFSET_METERS.x),
                            Length::new::<meter>(BLUE_PASS_TOP_OFFSET_METERS.y),
                        );
                    let pass_bottom = hub
                        + Vector2::new(
                            Length::new::<meter>(BLUE_PASS_BOTTOM_OFFSET_METERS.x),
                            Length::new::<meter>(BLUE_PASS_BOTTOM_OFFSET_METERS.y),
                        );
                    (hub, pass_top, pass_bottom)
                }
            };

            if shooter.manual_toggle {
                ferris.turret_mode = TurretMode::Manual;
            } else if shooter.idle_toggle {
                ferris.turret_mode = TurretMode::Idle;
            }

            match ferris.turret_mode {
                TurretMode::Track => {
                    // auto-assign shooting target
                    let zone = get_zone(&pose);

                    if alliance_station().red() {
                        match zone {
                            BlueTop => ferris.shooter_target = ShootingTarget::PassTop,
                            BlueBottom => ferris.shooter_target = ShootingTarget::PassBottom,
                            MiddleTop => ferris.shooter_target = ShootingTarget::PassTop,
                            MiddleBottom => ferris.shooter_target = ShootingTarget::PassBottom,
                            RedTop => ferris.shooter_target = ShootingTarget::Hub,
                            RedBottom => ferris.shooter_target = ShootingTarget::Hub,
                        }
                    } else {
                        match zone {
                            BlueTop => ferris.shooter_target = ShootingTarget::Hub,
                            BlueBottom => ferris.shooter_target = ShootingTarget::Hub,
                            MiddleTop => ferris.shooter_target = ShootingTarget::PassTop,
                            MiddleBottom => ferris.shooter_target = ShootingTarget::PassBottom,
                            RedTop => ferris.shooter_target = ShootingTarget::PassTop,
                            RedBottom => ferris.shooter_target = ShootingTarget::PassBottom,
                        }
                    }

                    // manual override for shooting target
                    match alliance_station().red() {
                        true => {
                            if ferris.controllers.operator.get(3) {
                                ferris.shooter_target = ShootingTarget::PassBottom;
                            } else if ferris.controllers.operator.get(4) {
                                ferris.shooter_target = ShootingTarget::PassTop;
                            }
                        }
                        false => {
                            if ferris.controllers.operator.get(3) {
                                ferris.shooter_target = ShootingTarget::PassTop;
                            } else if ferris.controllers.operator.get(4) {
                                ferris.shooter_target = ShootingTarget::PassBottom;
                            }
                        }
                    }

                    if ferris.controllers.operator.get(2) {
                        ferris.shooter_target = ShootingTarget::Hub;
                    }

                    match ferris.shooter_target {
                        ShootingTarget::PassTop => {
                            shooter.pass_to(&pose, pass_top).await;
                            max_dt_speed =
                                get_drivetrain_max_speed(&pose, turret_velocity, pass_top);
                        }
                        ShootingTarget::Hub => {
                            shooter.shoot_to(&pose, hub).await;
                            max_dt_speed = get_drivetrain_max_speed(&pose, turret_velocity, hub);
                            println!("lib::teleop: turret v:{:.2?}", turret_velocity);

                            Telemetry::put_number(
                                "Hub Distance",
                                distance(Vector2::new(pose.x, pose.y), hub),
                            )
                            .await;
                        }
                        ShootingTarget::PassBottom => {
                            shooter.shoot_to(&pose, pass_bottom).await;
                            max_dt_speed =
                                get_drivetrain_max_speed(&pose, turret_velocity, pass_bottom);
                        }
                        ShootingTarget::PassTelemetry => {
                            println!("ShootingTarget = PassTelemetry? Switching to Idle Mode");
                            ferris.turret_mode = TurretMode::Idle;
                        }
                    }
                }
                TurretMode::Manual => {
                    shooter.turret.man_yaw(ferris.controllers.operator.get_z());
                    let percent_distance = (ferris.controllers.operator.get_throttle() + 1.0) / 2.0;
                    let distance = percent_distance
                        * Length::new::<meter>(MANUAL_TURRET_MODE_DISTANCE_MAX_METERS);
                    let current_flywheel_speed = shooter.get_speed();
                    shooter.set_velocity(get_scoring_shooter_speed_target(distance));
                    shooter.set_hood(get_scoring_hood_angle_target(
                        distance,
                        current_flywheel_speed,
                    ));
                }
                TurretMode::Idle => {
                    shooter.turret.stop();
                    shooter.stop();
                }
                TurretMode::Test => {
                    shooter.turret.stop();
                    shooter.stop();
                }
            }

            Telemetry::put_string("turret_mode", String::from(ferris.turret_mode.name())).await;
            Telemetry::put_string("shooter_target", String::from(ferris.shooter_target.name()))
                .await;
            Telemetry::put_number("DISTANCE OFFSET", shooter.distance_offset.get::<foot>()).await;
            Telemetry::put_number("YAW OFFSET", shooter.turret.yaw_offset.get::<degree>()).await;

            // Run Intake Functions
            if let Ok(intake) = ferris.intake.try_borrow_mut() {
                if ferris.controllers.operator.get(1) {
                    intake.set_intake_speed(INTAKE_IN_SPEED);
                    intake.set_handoff(HANDOFF_SPEED);
                } else if ferris.controllers.operator.get(2) {
                    intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
                    intake.set_handoff(-HANDOFF_SPEED);
                } else {
                    intake.stop();
                }
            }
        }

        let deadzone_output_range = 0.0..1.0;
        let deadzone_input_range = 0.05..1.0;
        drivetrain.control_drivetrain(
            deadzone(
                -ferris.controllers.left_drive.get_x(),
                &deadzone_input_range,
                &deadzone_output_range,
            ),
            deadzone(
                ferris.controllers.left_drive.get_y(),
                &deadzone_input_range,
                &deadzone_output_range,
            ),
            deadzone(
                ferris.controllers.right_drive.get_z(),
                &deadzone_input_range,
                &deadzone_output_range,
            ),
            max_dt_speed,
        );
        if ferris.controllers.right_drive.get(1) {
            drivetrain.reset_heading();
        }

        if ferris.controllers.right_drive.get(4) {
            drivetrain.set_gyro_offset();
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
