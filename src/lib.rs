use crate::subsystems::swerve::drivetrain::Drivetrain;
use crate::subsystems::swerve::odometry::RobotPoseEstimate;
use frcrs::input::{Joystick, RobotState};
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
    //other subsystems here
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
                0.,
                Length::new::<meter>(0.),
                Length::new::<meter>(0.),
                Angle::new::<radian>(0.),
            )))),
            // other subsystems here
            dt: Duration::from_millis(0),
        }
    }

    pub fn stop(&self) {
        if let Ok(drivetrain) = self.drivetrain.try_borrow() {
            drivetrain.stop();
        }
        // other subsystems here
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
    }

    // other subsystem logic here
}
