use std::fs::File;
use std::io::{Read, Write};

use crate::constants::robotmap::drivetrain_map::DRIVETRAIN_CANBUS;
use crate::constants::robotmap::shooter::SHOOTER_CANBUS;
use crate::constants::robotmap::turret::{ENCODER_ID, SPIN_MOTOR_ID};
use crate::constants::turret::{
    ABS_TO_REL_RATIO, RELATIVE_TO_TURRET_RATIO, TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS,
    TURRET_CLAMP,
};
use frcrs::ctre::{CanCoder, ControlMode, Talon};
use nalgebra::Vector2;
use uom::si::angle::radian;
use uom::si::angle::{degree, revolution};
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::length::meter;

pub struct Turret {
    spin_motor: Talon,
    drivetrain_angle: Angle,
    #[allow(unused)]
    encoder: CanCoder,

    pub man_turret_angle: Angle,
    pub desired_angle: Angle,
    pub yaw_offset: Angle,
    relative_turret_zero: Angle,
}

impl Turret {
    pub fn new() -> Self {
        let spin_motor = Talon::new(SPIN_MOTOR_ID, DRIVETRAIN_CANBUS);
        let encoder = CanCoder::new(ENCODER_ID, Some(SHOOTER_CANBUS.to_string()));

        let file_result = File::open("/tmp/turret_zero");
        let mut relative_turret_zero: Option<Angle> = None;

        if file_result.is_ok() {
            let mut file = file_result.unwrap();
            let metadata = file.metadata();
            if metadata.is_ok() {
                if metadata.unwrap().len() == 8 {
                    let mut buf = [0u8; 8];
                    if file.read_exact(&mut buf).is_ok() {
                        relative_turret_zero =
                            Some(Angle::new::<revolution>(f64::from_ne_bytes(buf)));
                    } else {
                        panic!("cant read buffer");
                    }
                } else {
                    panic!("metadata bad or len != 8");
                }
            } else {
                panic!("metadata bad");
            }
            println!(
                "read from file: turret zero: {}",
                relative_turret_zero.clone().unwrap().get::<revolution>()
            )
        }

        if relative_turret_zero.is_none() {
            let zero = spin_motor.get_position()
                + (encoder.get_absolute() - TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS)
                    * ABS_TO_REL_RATIO;
            relative_turret_zero = Some(Angle::new::<revolution>(zero));

            println!(
                "found no turret zero, writing: {}",
                relative_turret_zero.clone().unwrap().get::<revolution>()
            );

            let zero_bytes = zero.to_ne_bytes();
            let new_file = File::create("/tmp/turret_zero");

            #[allow(unused_must_use)]
            if new_file.is_ok() {
                let mut created_file = new_file.unwrap();
                created_file.write_all(&zero_bytes);
                created_file.flush();
            } else {
                panic!("cannot write turret zero file");
            }
        }

        Turret {
            spin_motor,
            drivetrain_angle: Angle::new::<degree>(0.0),
            encoder,
            relative_turret_zero: relative_turret_zero.unwrap(),

            man_turret_angle: Angle::new::<degree>(0.0),
            desired_angle: Angle::new::<degree>(0.0),
            yaw_offset: Angle::new::<degree>(0.0),
        }
    }

    pub fn update_turret(&mut self, drivetrain_angle: Angle) {
        self.drivetrain_angle = drivetrain_angle;
    }

    pub fn move_to_angle(&mut self, angle: Angle) {
        let position = self.spin_motor.get_position();

        let target_rot = ((apply_soft_stop(angle)).get::<revolution>() * RELATIVE_TO_TURRET_RATIO
            + self.relative_turret_zero.get::<revolution>())
        .clamp(position - TURRET_CLAMP, position + TURRET_CLAMP);
        self.spin_motor.set(ControlMode::Position, target_rot);
    }

    pub fn set_angle(&mut self, robot_turret_angle: Angle) {
        self.move_to_angle(robot_turret_angle);
    }

    pub fn set_speed(&self, speed: f64) {
        self.spin_motor.set(ControlMode::Percent, speed);
    }

    pub fn offset_yaw(&mut self, amount: Angle) {
        self.yaw_offset = self.yaw_offset + amount;
    }

    pub fn slow(&self, angle1: f64, angle2: f64) {
        if (angle1 - angle2).abs() > 30.0 {}
    }

    pub fn stop(&self) {
        self.spin_motor.stop();
    }
}

pub fn get_angle_to(pose: Vector2<Length>, target: Vector2<Length>) -> Angle {
    let x = pose.x.get::<meter>();
    let y = pose.y.get::<meter>();
    let dx = target.x.get::<meter>() - x;
    let dy = target.y.get::<meter>() - y;

    Angle::new::<degree>(dy.atan2(dx).to_degrees())
}

fn apply_soft_stop(desired_deg: Angle) -> Angle {
    Angle::new::<radian>(f64::atan2(
        desired_deg.get::<radian>().sin(),
        desired_deg.get::<radian>().cos(),
    ))
}
