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
    BlueBottom, BlueTop, MiddleBottom, MiddleTop, RedBottom, RedTop,
};
use crate::subsystems::swerve::drivetrain::get_zone;
use crate::subsystems::turret::TurretMode;
use crate::subsystems::vision::distance;
use crate::{Ferris, HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use frcrs::alliance_station;
use frcrs::telemetry::Telemetry;
use nalgebra::Vector2;
use uom::si::f64::Length;
use uom::si::length::meter;

struct Targeting {
    hub: Vector2<Length>,
    pass_top: Vector2<Length>,
    pass_bottom: Vector2<Length>,
}

impl Targeting {
    fn new() -> Targeting {
        let zeros = Vector2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0));
        Targeting {
            hub: zeros,
            pass_top: zeros,
            pass_bottom: zeros,
        }
    }

    async fn update(&mut self, ferris: &mut Ferris) {
        let pose = if let Ok(drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.localization.get_state()
        } else {
            return;
        };

        // Run Shooter Functions
        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            shooter.turret.update_turret(pose.yaw);

            if ferris
                .debouncer
                .calculate(ferris.controllers.operator.get(11))
            {
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
            (self.hub, self.pass_top, self.pass_bottom) = match alliance_station().red() {
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

            Telemetry::put_string("turret_mode", String::from(ferris.turret_mode.name())).await;
            Telemetry::put_string("shooter_target", String::from(ferris.shooter_target.name()))
                .await;
        }
    }

    fn act(&mut self, ferris: &mut Ferris) {
        let pose = if let Ok(drivetrain) = ferris.drivetrain.try_borrow_mut() {
            drivetrain.localization.get_state()
        } else {
            return;
        };

        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
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
                            shooter.pass_to(&pose, self.pass_top);
                        }
                        ShootingTarget::Hub => {
                            shooter.shoot_to(&pose, self.hub);
                            Telemetry::put_number(
                                "Hub Distance",
                                distance(Vector2::new(pose.x, pose.y), self.hub),
                            );
                        }
                        ShootingTarget::PassBottom => {
                            shooter.shoot_to(&pose, self.pass_bottom);
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
        }
    }
}

struct Launcher {}

impl Launcher {
    fn new() -> Launcher {
        Launcher {}
    }

    async fn update(&mut self, ferris: &mut Ferris) {}

    fn act(&mut self, ferris: &mut Ferris) {
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
}

pub struct Shooter {
    targeting: Targeting,
    launcher: Launcher,
}

impl Shooter {
    pub fn new() -> Shooter {
        Shooter {
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
