// use crate::auto::path::drive;
use crate::constants::config::{HUB, MAX_ITER};
use crate::constants::robotmap::intake::{HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use crate::constants::robotmap::shooter::HOOD_MAX;
use crate::subsystems::intake::Intake;
use crate::subsystems::shooter::{Shooter, ShootingTarget};
use crate::subsystems::swerve::drivetrain::Drivetrain;
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
            turret_mode: TurretMode::Manual,

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
        // drivetrain.update_limelight().await;
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

        Telemetry::put_number(
            "justics for cam (i32 editon)",
            drivetrain.limelight.results.stdev_mt2[0],
        )
        .await;
        // drivetrain.update_localization().await;

        let pose = drivetrain.limelight.get_pose();
        Telemetry::set_robot_pose(
            (
                pose.clone().x.get::<meter>() / 17.55,
                pose.clone().y.get::<meter>() / 8.05,
                pose.clone().angle.get::<degree>(),
            ),
            alliance_station().red(),
        )
        .await;

        Telemetry::put_number("ll_yaw", drivetrain.limelight.get_yaw().get::<degree>()).await;
        println!("yaw: {}", drivetrain.limelight.get_yaw().get::<degree>());

        // shooter logic here because it needs velocities and pose
        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            shooter.turret.update_turret(drivetrain.limelight.get_yaw());
            match ferris.turret_mode {
                TurretMode::Track => {
                    // shooter
                    //     .shoot_on_move(
                    //         pose.clone(),
                    //         (drivetrain.velocity.x, drivetrain.velocity.y),
                    //         drivetrain.angular_velocity,
                    //         MAX_ITER,
                    //         ferris.shooter_target.clone(),
                    //     )
                    //     .await;

                    shooter
                        .turret
                        .set_angle(get_angle_to_hub(pose.clone()).get::<degree>());
                }
                TurretMode::Manual => {
                    shooter.turret.man_move(ferris.controllers.operator.get_z());
                    shooter.set_hood(ferris.controllers.operator.get_throttle() * HOOD_MAX);
                    // println!("{:?}", shooter.turret.turret_angle);
                    // println!("{:?}", ferris.controllers.operator.get_z());
                }
                TurretMode::Idle => {
                    shooter.turret.stop();
                }
                TurretMode::Test => {
                    shooter.turret.set_angle(0.0);
                    if ferris.controllers.operator.get_throttle() * -3. >= -3.
                        && ferris.controllers.operator.get_throttle() * -3. <= 0.
                    {
                        shooter.set_hood(ferris.controllers.operator.get_throttle() * -3.);
                    }
                }
            }

            if let Some(turret_mode) = Telemetry::get_selection("justice for cam :)").await {
                ferris.turret_mode = TurretMode::to_mode(turret_mode.as_str());
            }

            let hub_distance = distance(HUB, pose.clone());
            Telemetry::put_number("da hub", hub_distance).await;
            Telemetry::put_number("ll_X", pose.clone().x.get::<meter>()).await;
            Telemetry::put_number("ll_Y", pose.clone().y.get::<meter>()).await;
            Telemetry::put_number("ll_YAW", pose.clone().angle.get::<degree>()).await;

            if let Ok(mut intake) = ferris.intake.try_borrow_mut() {
                // maybe zero maybe one i forogt what trigger is
                // TODO: make toggle work (probably also a debouncer then...)
                if ferris.controllers.left_drive.get(1) {
                    intake.set_intake_speed(INTAKE_IN_SPEED);
                    intake.set_handoff(HANDOFF_SPEED);
                    shooter.set_shooter(ferris.controllers.left_drive.get_throttle());
                } else if ferris.controllers.left_drive.get(2) {
                    intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
                    intake.set_handoff(-HANDOFF_SPEED);
                } else {
                    intake.stop();
                }
            }
        }
    }
}
