use crate::Controllers;
use crate::constants::config::HUB;
use crate::constants::robotmap::turret::SPIN_MOTOR_ID;
use crate::constants::turret::{GEAR_RATIO, TURRET_MAX, TURRET_MIN};
use crate::subsystems::swerve::odometry::RobotPoseEstimate;
use frcrs::ctre::{ControlMode, Talon};
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length};
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
}

impl TurretMode {
    pub fn name(&self) -> &'static str {
        match self {
            TurretMode::Idle => "idle",
            TurretMode::Manual => "man",
            TurretMode::Track => "track",
            TurretMode::Test => "test",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![TurretMode::Idle]
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
        }
    }

    pub fn update_turret(&mut self, drivetrain_angle: Angle) {
        self.drivetrain_angle = drivetrain_angle;
    }

    pub fn move_to_angle(&self, angle: f64) {
        let target_rot = angle / 360.0 * GEAR_RATIO;
        println!("target_rot {}", target_rot);
        self.spin_motor.set(ControlMode::Position, target_rot);
    }

    pub fn set_angle(&mut self, mut angle: f64) {
        angle = self.apply_soft_stop(angle);
        let field_relative_angle = angle - self.drivetrain_angle.get::<degree>();
        self.move_to_angle(field_relative_angle);
    }

    pub fn set_speed(&self, speed: f64) {
        self.spin_motor.set(ControlMode::Percent, speed);
    }

    // idk if this is still optimal now that we are likely using 2 static lls but its still here anyway :)
    // status update its not but its being reused
    pub fn track(&mut self, pose: RobotPoseEstimate, distance: Length) {
        //shoot_on_move();
    }

    fn apply_soft_stop(&self, desired_deg: f64) -> f64 {
        let current = self.turret_angle.get::<degree>() % 360.;
        println!("attempting current {:?}", current);
        let mut best = current;
        let mut found = false;

        for i in -1..=1 {
            let candidate = desired_deg + 360.0 * i as f64;

            if candidate < TURRET_MIN || candidate > TURRET_MAX {
                continue;
            }

            println!("testing {:?}", candidate);

            if !found || (candidate - current).abs() < (best - current).abs() {
                best = candidate;
                found = true;
            }
        }

        best
    }

    pub fn man_move(&mut self, joystick: f64) {
        let angle = self.turret_angle.get::<degree>() + joystick;
        println!("here: {}", angle);
        self.turret_angle = Angle::new::<degree>(angle);
        self.move_to_angle(self.apply_soft_stop(angle));
        println!("moved? {}", self.apply_soft_stop(angle));
    }

    pub fn stop(&self) {
        self.spin_motor.stop();
    }
}

pub fn get_angle_to_hub(pose: RobotPoseEstimate) -> Angle {
    let x = pose.x.get::<meter>();
    let y = pose.y.get::<meter>();
    let dx = HUB.x - x;
    let dy = HUB.y - y;

    Angle::new::<degree>(dy.atan2(dx).to_degrees())
}

#[cfg(test)]
mod tests {
    use crate::subsystems::swerve::odometry::RobotPoseEstimate;
    use crate::subsystems::turret::get_angle_to_hub;
    use uom::si::angle::{degree, radian};
    use uom::si::f64::{Angle, Length};
    use uom::si::length::meter;

    #[test]
    pub fn test_angle_to_hub() {
        let pose = RobotPoseEstimate::new(
            1.,
            Length::new::<meter>(-1.),
            Length::new::<meter>(-1.),
            Angle::new::<radian>(0.),
        );

        let result = get_angle_to_hub(pose).get::<degree>();
        let expected = Angle::new::<degree>(45.);

        println!("results: {:?}", result);
        println!("expected: {:?}", expected.get::<degree>());

        assert_eq!(result, expected.get::<degree>());
    }

    #[test]
    pub fn test_soft_stop() {}
}
