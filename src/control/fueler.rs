use std::collections::VecDeque;

use crate::constants::config::{
    BLUE_PASS_BOTTOM_OFFSET_METERS, BLUE_PASS_TOP_OFFSET_METERS, HUB_BLUE, HUB_RED,
    MANUAL_TURRET_MODE_DISTANCE_MAX_METERS, RED_PASS_BOTTOM_OFFSET_METERS,
    RED_PASS_TOP_OFFSET_METERS,
};
use crate::subsystems::shooter::{
    ShootingTarget, get_drivetrain_max_speed, get_scoring_hood_angle_target,
    get_scoring_shooter_speed_target,
};
use crate::subsystems::swerve::drivetrain::FieldZone::{
    self, BlueBottom, BlueTop, MiddleBottom, MiddleTop, RedBottom, RedTop,
};
use crate::subsystems::swerve::drivetrain::get_zone;
use crate::subsystems::turret::TurretMode;
use crate::subsystems::vision::distance;
use crate::{Ferris, HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use frcrs::alliance_station;
use frcrs::telemetry::Telemetry;
use nalgebra::{RowOVector, RowSVector, Vector2};
use uom::si::f64::Length;
use uom::si::length::meter;

enum TargetingMode {
    Idle,
    Manual,
    Track,
    Telemetry,
}

impl TargetingMode {
    fn name(&self) -> String {
        match self {
            TargetingMode::Idle => "Idle".to_string(),
            TargetingMode::Manual => "Manual".to_string(),
            TargetingMode::Track => "Track".to_string(),
            TargetingMode::Telemetry => "Telemetry".to_string(),
        }
    }
}

#[derive(Clone)]
enum TargetType {
    Hub,
    Passing,
}

#[derive(Clone)]
struct Target {
    target_type: TargetType,
    target_location: Vector2<Length>,
    name: String,
}

impl Target {
    fn new(target_type: TargetType, target_location: Vector2<Length>, name: String) -> Target {
        Target {
            target_type,
            target_location,
            name,
        }
    }
}

struct Targeting {
    mode: TargetingMode,
    target: Target,

    blue_hub: Target,
    blue_top: Target,
    blue_bottom: Target,
    red_hub: Target,
    red_top: Target,
    red_bottom: Target,
}

impl Targeting {
    fn new() -> Targeting {
        let starting_target = Target::new(
            TargetType::Passing,
            Vector2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
            "Custom".to_string(),
        );

        let blue_hub = Target::new(
            TargetType::Hub,
            Vector2::new(
                Length::new::<meter>(HUB_BLUE.x),
                Length::new::<meter>(HUB_BLUE.y),
            ),
            "Blue Hub".to_string(),
        );

        let blue_top = Target::new(
            TargetType::Passing,
            Vector2::new(
                Length::new::<meter>(HUB_BLUE.x + BLUE_PASS_TOP_OFFSET_METERS.x),
                Length::new::<meter>(HUB_BLUE.y + BLUE_PASS_TOP_OFFSET_METERS.y),
            ),
            "Passing Blue Top".to_string(),
        );

        let blue_bottom = Target::new(
            TargetType::Passing,
            Vector2::new(
                Length::new::<meter>(HUB_BLUE.x + BLUE_PASS_BOTTOM_OFFSET_METERS.x),
                Length::new::<meter>(HUB_BLUE.y + BLUE_PASS_BOTTOM_OFFSET_METERS.y),
            ),
            "Passing Blue Bottom".to_string(),
        );

        let red_hub = Target::new(
            TargetType::Hub,
            Vector2::new(
                Length::new::<meter>(HUB_RED.x),
                Length::new::<meter>(HUB_RED.y),
            ),
            "Red Hub".to_string(),
        );

        let red_top = Target::new(
            TargetType::Passing,
            Vector2::new(
                Length::new::<meter>(HUB_RED.x + RED_PASS_TOP_OFFSET_METERS.x),
                Length::new::<meter>(HUB_RED.y + RED_PASS_TOP_OFFSET_METERS.y),
            ),
            "Passing Red Top".to_string(),
        );

        let red_bottom = Target::new(
            TargetType::Passing,
            Vector2::new(
                Length::new::<meter>(HUB_RED.x + RED_PASS_BOTTOM_OFFSET_METERS.x),
                Length::new::<meter>(HUB_RED.y + RED_PASS_BOTTOM_OFFSET_METERS.y),
            ),
            "Passing Red Bottom".to_string(),
        );

        Targeting {
            mode: TargetingMode::Idle,
            target: starting_target,

            blue_hub,
            blue_top,
            blue_bottom,
            red_hub,
            red_top,
            red_bottom,
        }
    }

    async fn update(&mut self, ferris: &mut Ferris) {
        let pose = if let Ok(drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.localization.get_state()
        } else {
            return;
        };

        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            shooter.turret.update_turret(pose.yaw);

            let zone = get_zone(&pose);

            if alliance_station().red() {
                match zone {
                    FieldZone::BlueBottom => self.target = self.red_bottom.clone(),
                    FieldZone::BlueTop => self.target = self.red_top.clone(),
                    FieldZone::MiddleBottom => self.target = self.red_bottom.clone(),
                    FieldZone::MiddleTop => self.target = self.red_top.clone(),
                    FieldZone::RedBottom => self.target = self.red_hub.clone(),
                    FieldZone::RedTop => self.target = self.red_hub.clone(),
                }
            } else {
                match zone {
                    FieldZone::BlueBottom => self.target = self.blue_hub.clone(),
                    FieldZone::BlueTop => self.target = self.blue_hub.clone(),
                    FieldZone::MiddleBottom => self.target = self.blue_bottom.clone(),
                    FieldZone::MiddleTop => self.target = self.blue_top.clone(),
                    FieldZone::RedBottom => self.target = self.blue_bottom.clone(),
                    FieldZone::RedTop => self.target = self.blue_top.clone(),
                }
            }

            if ferris
                .man_toggle_debouncer
                .debounce(ferris.controllers.operator.get(16))
            {
                match self.mode {
                    TargetingMode::Idle | TargetingMode::Telemetry | TargetingMode::Track => {
                        self.mode = TargetingMode::Manual
                    }
                    TargetingMode::Manual => self.mode = TargetingMode::Idle,
                }
            }

            if ferris
                .idle_toggle_debouncer
                .debounce(ferris.controllers.operator.get(11))
            {
                match self.mode {
                    TargetingMode::Manual | TargetingMode::Telemetry | TargetingMode::Idle => {
                        self.mode = TargetingMode::Track
                    }
                    TargetingMode::Track => self.mode = TargetingMode::Idle,
                }
            }
            Telemetry::put_string("turret_mode", String::from(self.mode.name())).await;
            Telemetry::put_string("shooter_target", String::from(&self.target.name)).await;
        }
    }

    fn act(&mut self, ferris: &mut Ferris) {
        let pose = if let Ok(drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.localization.get_state()
        } else {
            return;
        };

        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            match self.mode {
                TargetingMode::Track => match self.target.target_type {
                    TargetType::Hub => shooter.shoot_to(&pose, self.target.target_location),
                    TargetType::Passing => shooter.shoot_to(&pose, self.target.target_location),
                },
                TargetingMode::Manual => {
                    // shooter.turret.man_yaw(ferris.controllers.operator.get_z());
                    // let percent_distance = (ferris.controllers.operator.get_throttle() + 1.0) / 2.0;
                    // let distance = percent_distance
                    //     * Length::new::<meter>(MANUAL_TURRET_MODE_DISTANCE_MAX_METERS);
                    // let current_flywheel_speed = shooter.get_speed();
                    // shooter.set_velocity(get_scoring_shooter_speed_target(distance));
                    // shooter.set_hood(get_scoring_hood_angle_target(
                    //     distance,
                    //     current_flywheel_speed,
                    // ));
                    shooter.turret.stop();
                    shooter.stop();
                }
                TargetingMode::Idle => {
                    shooter.turret.stop();
                    shooter.stop();
                }
                TargetingMode::Telemetry => {
                    shooter.turret.stop();
                    shooter.stop();
                }
            }
        }
    }
}

struct Launcher {
    currently_shooting: bool,
}

impl Launcher {
    fn new() -> Launcher {
        Launcher {
            currently_shooting: false,
        }
    }

    async fn update(&mut self, ferris: &mut Ferris) {}

    fn act(&mut self, ferris: &mut Ferris) {
        if let Ok(intake) = ferris.intake.try_borrow_mut() {
            if ferris.controllers.operator.get(1) {
                intake.set_intake_speed(INTAKE_IN_SPEED);
                intake.set_handoff(HANDOFF_SPEED);
                self.currently_shooting = true;
            } else if ferris.controllers.operator.get(2) {
                intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
                intake.set_handoff(-HANDOFF_SPEED);
            } else {
                intake.stop();
                self.currently_shooting = false;
            }
        }
    }
}

pub struct Fueler {
    targeting: Targeting,
    launcher: Launcher,
}

impl Fueler {
    pub fn new() -> Fueler {
        Fueler {
            targeting: Targeting::new(),
            launcher: Launcher::new(),
        }
    }

    pub async fn update(&mut self, ferris: &mut Ferris) {
        self.targeting.update(ferris).await;
        self.launcher.update(ferris).await;
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        self.targeting.act(ferris);
        self.launcher.act(ferris);
    }
}
