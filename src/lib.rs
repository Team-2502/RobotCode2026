use crate::constants::robotmap::intake::{INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED};
use crate::subsystems::intake::Intake;
use crate::subsystems::shooter::{Shooter, ShootingTarget};
use crate::subsystems::swerve::drivetrain::Drivetrain;
use crate::subsystems::swerve::odometry::RobotPoseEstimate;
use crate::subsystems::turret::{TurretMode, get_angle_to_hub};
use frcrs::alliance_station;
use frcrs::input::Joystick;
use frcrs::telemetry::Telemetry;
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
            turret_mode: TurretMode::Idle,

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
    if let Ok(mut drivetrain) = ferris.drivetrain.try_borrow_mut() {
        drivetrain.control_drivetrain(
            ferris.controllers.left_drive.get_x(),
            ferris.controllers.left_drive.get_y(),
            ferris.controllers.right_drive.get_z(),
        );
        drivetrain.update_limelight().await;
        drivetrain.update_localization().await;

        let pose = drivetrain.get_pose_estimate();
        // TODO: make work with 0-1 cord system or fix telem also maybe flip degrees so forward is forward (if arrow ever works)
        Telemetry::set_robot_pose(
            (
                pose.x.get::<meter>(),
                pose.y.get::<meter>(),
                pose.angle.get::<degree>(),
            ),
            alliance_station().red(),
        )
        .await;

        // shooter logic here because it needs velocities and pose
        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            shooter.turret.update_turret(pose.angle);
            match ferris.turret_mode {
                TurretMode::Track => {
                    //shoot on the fly track function here
                }
                TurretMode::Manual => {}
                TurretMode::Idle => {
                    shooter.turret.stop();
                }
            }

            //this is for initial test
            shooter
                .turret
                .set_angle(get_angle_to_hub(pose).get::<degree>());
        }
    }

    if let Ok(mut intake) = ferris.intake.try_borrow_mut() {
        // maybe zero maybe one i forogt what trigger is
        // TODO: make toggle work (probably also a debouncer then...)
        if ferris.controllers.left_drive.get(0) {
            intake.set_intake_speed(INTAKE_IN_SPEED);
        }
        if ferris.controllers.left_drive.get(1) {
            intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
        }
    }
}
