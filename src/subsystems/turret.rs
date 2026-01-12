use crate::constants::robotmap::turret::SPIN_MOTOR_ID;
use crate::constants::turret::GEAR_RATIO;
use crate::subsystems::vision::Vision;
use frcrs::ctre::{ControlMode, Talon};
use std::net::{IpAddr, Ipv4Addr, SocketAddr};

pub struct Turret {
    limelight: Vision,
    spin_motor: Talon,
    //turret_offset: f64,
}

impl Turret {
    pub fn new() -> Self {
        let spin_motor = Talon::new(SPIN_MOTOR_ID, Some("can0".to_string()));
        let limelight = Vision::new(SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(10, 25, 2, 12)),
            5807,
        ));

        Turret {
            limelight,
            spin_motor,
        }
    }

    pub fn set_angle(&mut self, angle: f64) {}

    pub fn move_to_angle(&self, angle: f64) {
        let target_rot = angle / 360.0 * GEAR_RATIO;

        self.spin_motor.set(ControlMode::MotionMagic, target_rot);
    }
    pub fn set_speed(&self, speed: f64) {
        self.spin_motor.set(ControlMode::Percent, speed);
    }
}
