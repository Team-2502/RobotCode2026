use crate::constants::config::{
    BLUE_PASS_BOTTOM_OFFSET_METERS, BLUE_PASS_TOP_OFFSET_METERS, HALF_FIELD_LENGTH_METERS,
    HALF_FIELD_WIDTH_METERS, HUB_BLUE, HUB_RED, RED_PASS_BOTTOM_OFFSET_METERS,
    RED_PASS_TOP_OFFSET_METERS,
};
use crate::control::fueler::{Target, TargetType, TargetingMode};
use crate::subsystems::swerve::drivetrain::FieldZone;
use crate::subsystems::swerve::drivetrain::get_zone;
use crate::{Ferris, HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use frcrs::alliance_station;
use frcrs::telemetry::Telemetry;
use nalgebra::Vector2;
use uom::si::angle::degree;
use uom::si::f64::Length;
use uom::si::length::meter;
use uom::si::quantities::Angle;

struct AutonTargeting {
    mode: TargetingMode,
    target: Target,

    blue_hub: Target,
    blue_top: Target,
    blue_bottom: Target,
    red_hub: Target,
    red_top: Target,
    red_bottom: Target,
}

impl AutonTargeting {
    fn new() -> AutonTargeting {
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

        AutonTargeting {
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

            Telemetry::put_string("turret_mode", String::from(self.mode.name())).await;
            Telemetry::put_string("shooter_target", String::from(&self.target.name)).await;
        }
    }

    fn aim(&mut self, ferris: &mut Ferris, target: Target) {
        let (pose, cmd_ang, cmd_mag) = if let Ok(drivetrain) = ferris.drivetrain.try_borrow_mut() {
            (
                drivetrain.localization.get_state(),
                drivetrain.commanded_angle,
                drivetrain.commanded_magnitude,
            )
        } else {
            return;
        };

        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            shooter.shoot_to(&pose, target.target_location, cmd_ang, cmd_mag)
        }
    }
}

struct AutonLauncher {
    currently_shooting: bool,
}

impl AutonLauncher {
    fn new() -> AutonLauncher {
        AutonLauncher {
            currently_shooting: false,
        }
    }

    fn intake(&mut self, ferris: &mut Ferris, speed: f64) {
        if let Ok(intake) = ferris.intake.try_borrow_mut() {
            intake.set_intake_speed(speed);
        }
    }

    fn handoff(&mut self, ferris: &mut Ferris, speed: f64) {
        if let Ok(intake) = ferris.intake.try_borrow_mut() {
            intake.set_handoff(speed);
        }
    }
}

pub struct AutonFueler {
    auton_targeting: AutonTargeting,
    auton_launcher: AutonLauncher,
}

impl AutonFueler {
    pub fn new() -> AutonFueler {
        AutonFueler {
            auton_targeting: AutonTargeting::new(),
            auton_launcher: AutonLauncher::new(),
        }
    }

    pub async fn update(&mut self, ferris: &mut Ferris) {
        self.auton_targeting.update(ferris).await;
    }

    pub fn shoot(&mut self, ferris: &mut Ferris) {
        self.auton_launcher.handoff(ferris, HANDOFF_SPEED);
        self.auton_launcher.intake(ferris, INTAKE_IN_SPEED);
        if alliance_station().red() {
            self.auton_targeting
                .aim(ferris, self.auton_targeting.red_hub.clone());
        } else {
            self.auton_targeting
                .aim(ferris, self.auton_targeting.blue_hub.clone());
        }
    }

    pub fn get_target(&self) -> Target {
        self.auton_targeting.target.clone()
    }

    pub fn get_mode(&self) -> TargetingMode {
        self.auton_targeting.mode.clone()
    }
}
