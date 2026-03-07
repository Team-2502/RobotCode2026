use crate::constants::config::{HUB_RED, HUB_BLUE};
use crate::constants::robotmap::turret::SPIN_MOTOR_ID;
use crate::constants::turret::{GEAR_RATIO, TURRET_CLAMP, TURRET_MAX, TURRET_MIN};
use crate::subsystems::shooter::flip;
use crate::subsystems::swerve::kinematics::RobotPoseEstimate;
use frcrs::alliance_station;
use frcrs::ctre::{ControlMode, Talon};
use nalgebra::Vector2;
use uom::si::angle::degree;
use uom::si::f64::Length;
use uom::si::f64::Angle;
use uom::si::length::meter;

#[derive(PartialEq, Clone)]
pub enum TurretMode {
    Track,
    Manual,
    Idle,
    Test,
}

pub struct Turret {
    spin_motor: Talon,
    //turret_offset: f64,
    drivetrain_angle: Angle,

    pub turret_angle: Angle,
    pub desired_angle: Angle,
    pub offset: f64,
}

impl TurretMode {
    pub fn name(&self) -> &'static str {
        match self {
            TurretMode::Idle => "idle",
            TurretMode::Manual => "man",
            TurretMode::Track => "track",
            TurretMode::Test => "test",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![
            TurretMode::Idle,
            TurretMode::Test,
            TurretMode::Manual,
            TurretMode::Track,
        ]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

    pub fn to_mode(s: &str) -> Self {
        match s {
            "idle" => TurretMode::Idle,
            "man" => TurretMode::Manual,
            "track" => TurretMode::Track,
            "test" => TurretMode::Test,
            _ => TurretMode::Idle,
        }
    }
}

// ball park max -2.5
impl Turret {
    pub fn new() -> Self {
        let spin_motor = Talon::new(SPIN_MOTOR_ID, None);

        Turret {
            spin_motor,
            drivetrain_angle: Angle::new::<degree>(0.),

            turret_angle: Angle::new::<degree>(0.),
            desired_angle: Angle::new::<degree>(0.),
            offset: 0.0,
        }
    }

    pub fn update_turret(&mut self, drivetrain_angle: Angle) {
        self.drivetrain_angle = drivetrain_angle;
    }

    pub fn move_to_angle(&self, angle: f64) {
        let position = self.spin_motor.get_position();
        let target_rot =
            (angle / 360.0 * GEAR_RATIO).clamp(position - TURRET_CLAMP, position + TURRET_CLAMP);
        // println!("target_rot {}", target_rot);
        self.spin_motor.set(ControlMode::Position, target_rot);
    }

    pub fn set_angle(&mut self, robot_turret_angle: Angle) {
        println!("set_angle: robot angle {}", robot_turret_angle.get::<degree>());
        let new_angle = apply_soft_stop(robot_turret_angle.get::<degree>());
        // println!("{}", field_relative_angle);
        // let angle_new = self.apply_soft_stop(field_relative_angle);
        // println!("cool: {}", field_relative_angle);
        self.move_to_angle(new_angle);
    }

    pub fn set_speed(&self, speed: f64) {
        self.spin_motor.set(ControlMode::Percent, speed);
    }

    pub fn offset_turret(&mut self, amount: f64) {
        self.offset = self.offset + amount;
    }

    // fn apply_soft_stop(&self, desired_deg: f64) -> f64 {
    //     let current = self.turret_angle.get::<degree>() % 360.;
    //     // println!("attempting current {:?}", current);
    //     let mut best = current;
    //     let mut found = false;

    //     for i in -1..=1 {
    //         let candidate = desired_deg + 360.0 * i as f64;

    //         if candidate < TURRET_MIN || candidate > TURRET_MAX {
    //             continue;
    //         }

    //         // println!("testing {:?}", candidate);

    //         if !found || (candidate - current).abs() < (best - current).abs() {
    //             best = candidate;
    //             found = true;
    //         }
    //     }

    //     best
    // }

    pub fn man_move(&mut self, mut joystick: f64) {
        if alliance_station().blue() {
            joystick = -joystick;
        }
        let angle = self.turret_angle.get::<degree>() + joystick;
        // println!("here: {}", angle);
        self.turret_angle = Angle::new::<degree>(angle);
        self.move_to_angle(apply_soft_stop(angle));
        // println!("moved? {}", self.apply_soft_stop(angle));
    }

    pub fn slow(&self, angle1: f64, angle2: f64) {
        if (angle1 - angle2).abs() > 30.0 {}
    }

    pub fn stop(&self) {
        self.spin_motor.stop();
    }
}

pub fn get_angle_to_hub(pose: Vector2<Length>) -> Angle {
    let target = match alliance_station().red() {
        true => HUB_RED,
        false => HUB_BLUE,
    };

    let x = pose.x.get::<meter>();
    let y = pose.y.get::<meter>();
    let dx = target.x - x;
    let dy = target.y - y;

    Angle::new::<degree>(dy.atan2(dx).to_degrees())
}

pub fn apply_soft_stop(desired_deg: f64) -> f64 {
    if desired_deg > TURRET_MAX + 360.0 {
        panic!("PANIC: turret.rs soft stop max check: desired angle out of bounds: {}", desired_deg);
    }
    if desired_deg < TURRET_MIN - 360.0 {
        panic!("PANIC: turret.rs soft stop min check: desired angle out of bounds: {}", desired_deg);
    }
    if desired_deg > TURRET_MAX {
        desired_deg - 360.0
    } else if desired_deg < TURRET_MIN {
        desired_deg + 360.0
    } else {
        desired_deg
    }
}

// pub fn apply_soft_stop(desired_deg: f64) -> f64 {
//     // let current = self.turret_angle.get::<degree>() % 360.;
//     // println!("attempting current {:?}", current);
//     let mut current = desired_deg;
//     let mut best = 0.0;
//     let mut found = false;

//     for i in -1..=1 {
//         let candidate = desired_deg + 360.0 * i as f64;

//         if candidate < TURRET_MIN || candidate > TURRET_MAX {
//             continue;
//         }

//         // println!("testing {:?}", candidate);

//         if !found || (candidate - current).abs() < (best - current).abs() {
//             best = candidate;
//             found = true;
//         }
//     }

//     best
// }

#[cfg(test)]
mod tests {
    use crate::subsystems::swerve::kinematics::RobotPoseEstimate;
    use crate::subsystems::turret::{apply_soft_stop, get_angle_to_hub};
    use uom::si::angle::{degree, radian};
    use uom::si::f64::{Angle, Length};
    use uom::si::length::meter;

    // #[test]
    // pub fn test_angle_to_hub() {
    //     let pose = RobotPoseEstimate::new(
    //         1.,
    //         Length::new::<meter>(-1.),
    //         Length::new::<meter>(-1.),
    //         Angle::new::<radian>(0.),
    //     );

    //     let result = get_angle_to_hub(pose).get::<degree>();
    //     let expected = Angle::new::<degree>(45.);

    //     println!("results: {:?}", result);
    //     println!("expected: {:?}", expected.get::<degree>());

    //     assert_eq!(result, expected.get::<degree>());
    // }

    #[test]
    pub fn test_soft_stop() {
        let expected1 = 100.0;
        let expected2 = -170.0;
        let expected3 = 170.0;

        let result1 = apply_soft_stop(100.0);
        let result2 = apply_soft_stop(190.0);
        let result3 = apply_soft_stop(-190.0);

        assert_eq!(result1, expected1);
        assert_eq!(result2, expected2);
        assert_eq!(result3, expected3);
    }
}
