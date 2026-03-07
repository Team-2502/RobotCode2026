// use crate::auto::path::drive;
use crate::constants::config::{HUB_RED, HUB_BLUE, PASS_LEFT, PASS_RIGHT};
use crate::constants::robotmap::intake::{HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use crate::constants::robotmap::shooter::HOOD_MAX;
use crate::subsystems::intake::Intake;
use crate::subsystems::shooter::{
    Shooter, ShootingTarget, flip, get_hood_angle_target, get_turret_speed_target,
};
use crate::subsystems::swerve::drivetrain::{get_angle_difs, Drivetrain};
use crate::subsystems::swerve::kinematics::RobotPoseEstimate;
use crate::subsystems::turret::{TurretMode, get_angle_to_hub};
use crate::subsystems::vision::distance;
use frcrs::input::Joystick;
use frcrs::telemetry::Telemetry;
use frcrs::{alliance_station, deadzone};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::Duration;
use uom::si::angle::{degree, radian};
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

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

            drivetrain: Rc::new(RefCell::new(Drivetrain::new(RobotPoseEstimate::new(
                1.,
                Length::new::<meter>(0.),
                Length::new::<meter>(0.),
                Angle::new::<radian>(0.),
            )))),
            shooter: Rc::new(RefCell::new(Shooter::new())),
            intake: Rc::new(RefCell::new(Intake::new())),

            shooter_target: ShootingTarget::Idle,
            turret_mode: TurretMode::Idle,

            shooter_offset: 0.0,

            dt: Duration::from_millis(0),
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
    // run drivetrain functions each frame
    let deadzone_output_range = 0.0..1.0;
    let deadzone_input_range = 0.1..1.0;
    if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
        drivetrain.update_pose().await;
        let (_, yaw, _, _) = drivetrain.localization.get_state();
        Telemetry::put_number("localized yaw", yaw.get::<degree>()).await;
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
        );
        if ferris.controllers.right_drive.get(1) {
            drivetrain.reset_heading();
        }

        if ferris.controllers.right_drive.get(4) {
            drivetrain.set_gyro_offset();
        }

        let (pose, yaw, _, _) = drivetrain.localization.get_state();
        Telemetry::set_robot_pose(
            (
                pose.x.get::<meter>() / 17.55,
                pose.y.get::<meter>() / 8.05,
                yaw.get::<degree>(),
            ),
            alliance_station().red(),
        )
        .await;

        // shooter logic here because it needs velocities and pose
        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            let (pose, yaw, _, _) = drivetrain.localization.get_state();
            let robot_pose = pose;
            shooter.turret.update_turret(yaw);
            match ferris.turret_mode {
                TurretMode::Track => {
                    shooter
                        .turret
                        .set_angle(get_angle_difs(yaw, get_angle_to_hub(robot_pose.clone())));

                    //println!("{:?}", robot_pose.clone());
                    // let distance_hub = Length::new::<meter>(distance(target, robot_pose.clone()));

                    // let current_flywheel_speed = shooter.get_speed();
                    // shooter.set_velocity(get_turret_speed_target(distance_hub));
                    // shooter.set_hood(get_hood_angle_target(distance_hub, current_flywheel_speed));
                }
                TurretMode::Manual => {
                    shooter.turret.man_move(ferris.controllers.operator.get_z());
                    //shooter.set_hood(ferris.controllers.operator.get_throttle() * HOOD_MAX);
                }
                TurretMode::Idle => {
                    shooter.turret.stop();
                    shooter.stop();
                }
                TurretMode::Test => {
                    shooter.turret.set_angle(Angle::new::<degree>(0.0));
                    if ferris.controllers.operator.get_throttle() * -3. >= -3.
                        && ferris.controllers.operator.get_throttle() * -3. <= 0.
                    {
                        shooter.set_hood(ferris.controllers.operator.get_throttle() * -3.);
                    }
                }
            }

            match ferris.shooter_target {
                ShootingTarget::Hub => {
                    let target = match alliance_station().red() {
                        true => HUB_RED,
                        false => HUB_BLUE,
                    };

                    println!("red?: {:?}", alliance_station().red());

                    let (pose, _, _, _) = drivetrain.localization.get_state();
                    let distance_hub = Length::new::<meter>(distance(target, pose) /*+ ferris.controllers.operator.get_throttle()*/);
                    println!("da hub {:?}", distance_hub);
                    let current_flywheel_speed = shooter.get_speed();
                    shooter.set_velocity(get_turret_speed_target(distance_hub));
                    shooter.set_hood(get_hood_angle_target(distance_hub, current_flywheel_speed));
                }
                ShootingTarget::Idle => { shooter.stop() }
                ShootingTarget::PassLeft => {
                    let target = match alliance_station().red() {
                        true => PASS_LEFT,
                        false => flip(PASS_LEFT),
                    };

                    let distance_hub = Length::new::<meter>(distance(target, robot_pose.clone()));

                    let current_flywheel_speed = shooter.get_speed();
                    shooter.set_velocity(get_turret_speed_target(distance_hub));
                    shooter.set_hood(get_hood_angle_target(distance_hub, current_flywheel_speed));
                }
                ShootingTarget::PassRight => {
                    let target = match alliance_station().red() {
                        true => PASS_RIGHT,
                        false => flip(PASS_RIGHT),
                    };

                    let distance_hub = Length::new::<meter>(distance(target, robot_pose.clone()));

                    let current_flywheel_speed = shooter.get_speed();
                    shooter.set_velocity(get_turret_speed_target(distance_hub));
                    shooter.set_hood(get_hood_angle_target(distance_hub, current_flywheel_speed));
                }
                ShootingTarget::PassTelemetry => {}
            }

            // if ferris.controllers.left_drive.get(3) {
            //     shooter.turret.offset_turret(180.0);
            // }
            // if ferris.controllers.left_drive.get(4) {
            //     shooter.turret.offset_turret(0.0);
            // }

            //println!("got here");

            if ferris.controllers.operator.get(2) {
                ferris.shooter_target = ShootingTarget::Hub;
                ferris.turret_mode = TurretMode::Track;
            } else if ferris.controllers.operator.get(3) {
                ferris.shooter_target = ShootingTarget::PassLeft;
                ferris.turret_mode = TurretMode::Track;
            } else if ferris.controllers.operator.get(4) {
                ferris.shooter_target = ShootingTarget::PassRight;
                ferris.turret_mode = TurretMode::Track;
            } else if ferris.controllers.operator.get(7) {
                ferris.turret_mode = TurretMode::Idle;
                ferris.shooter_target = ShootingTarget::Idle
            } else if ferris.controllers.operator.get(11) {
                ferris.turret_mode = TurretMode::Manual

            }

            // if ferris.controllers.operator.get(8) {
            //     shooter.turret.offset_turret(0.5);
            // }
            // if ferris.controllers.operator.get(10) {
            //     shooter.turret.offset_turret(-0.5);
            // }
            // if ferris.controllers.operator.get(6) {
            //     ferris.shooter_offset = ferris.shooter_offset + 0.1;
            // }
            // if ferris.controllers.operator.get(9) {
            //     ferris.shooter_offset = ferris.shooter_offset - 0.1;
            // }

            Telemetry::put_string("turret_mode", String::from(ferris.turret_mode.name())).await;
            Telemetry::put_string("shooter_target", String::from(ferris.shooter_target.name()))
                .await;

            // Telemetry::put_number("shooter_offset", ferris.shooter_offset).await;
            // Telemetry::put_number("turret_offset", shooter.turret.offset).await;

            // if let Some(turret_mode) = Telemetry::get_selection("justice for cam :)").await {
            //     ferris.turret_mode = TurretMode::to_mode(turret_mode.as_str());
            // }

            let hub_distance = distance(HUB_RED, robot_pose.clone());
            Telemetry::put_number("da hub", hub_distance).await;
            Telemetry::put_number("ll_X", pose.x.get::<meter>()).await;
            Telemetry::put_number("ll_Y", pose.y.get::<meter>()).await;
            Telemetry::put_number("ll_YAW", yaw.get::<degree>()).await;

            if let Ok(intake) = ferris.intake.try_borrow_mut() {
                //shooter.set_shooter(ferris.controllers.operator.get_throttle());
                //
                // if ferris.controllers.right_drive.get_throttle() >= 0.0
                //     && ferris.controllers.right_drive.get_throttle() >= 0.0
                // {
                //     let distance = Length::new::<meter>(
                //         ((ferris.controllers.right_drive.get_throttle() * 15.0) + 5.0) / 3.28084,
                //     );

                //     println!(
                //         "[DEBUG]: distance: {:.01}",
                //         distance.get::<meter>() * 3.28084
                //     );

                //     let current_flywheel_speed = shooter.get_speed();
                //     shooter.set_velocity(get_turret_speed_target(distance));
                //     shooter.set_hood(get_hood_angle_target(distance, current_flywheel_speed));
                // }
                // maybe zero maybe one i forogt what trigger is
                // TODO: make toggle work (probably also a debouncer then...)
                if ferris.controllers.operator.get(1) {
                    intake.set_intake_speed(INTAKE_IN_SPEED);
                    intake.set_handoff(HANDOFF_SPEED);
                    //shooter.set_shooter(ferris.controllers.operator.get_throttle());
                }
                // } else if ferris.controllers.operator.get(2) {
                //     intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
                //     intake.set_handoff(-HANDOFF_SPEED);
                // }
                else {
                    intake.stop();
                }
            }
        }
    }
}
