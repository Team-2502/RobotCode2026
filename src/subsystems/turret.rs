use crate::Controllers;
use crate::constants::config::HUB_X;
use crate::constants::robotmap::turret::SPIN_MOTOR_ID;
use crate::constants::turret::{GEAR_RATIO, TURRET_MAX, TURRET_MIN};
use crate::subsystems::shooter::Shooter;
use crate::subsystems::swerve::odometry::RobotPoseEstimate;
use frcrs::ctre::{ControlMode, Talon};
use uom::si::angle::degree;
use uom::si::f64::Angle;
use uom::si::length::meter;

pub struct Turret {
    spin_motor: Talon,
    //turret_offset: f64,
    drivetrain_angle: Angle,
    turret_angle: Angle,

    shooter: Shooter,
}

impl Turret {
    pub fn new() -> Self {
        let spin_motor = Talon::new(SPIN_MOTOR_ID, Some("can0".to_string()));
        let shooter = Shooter::new();

        Turret {
            spin_motor,
            drivetrain_angle: Angle::new::<degree>(0.),
            turret_angle: Angle::new::<degree>(0.),

            shooter,
        }
    }

    pub fn update_turret(&mut self, drivetrain_angle: Angle) {
        self.drivetrain_angle = drivetrain_angle;
    }

    pub fn move_to_angle(&self, angle: f64) {
        let target_rot = angle / 360.0 * GEAR_RATIO;

        self.spin_motor.set(ControlMode::MotionMagic, target_rot);
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
    pub fn track(&mut self, turn: Angle) {
        self.set_angle(turn.get::<degree>());
    }

    // TODO: write a unit test for this i do not trust
    fn apply_soft_stop(&self, desired_deg: f64) -> f64 {
        let current = self.turret_angle.get::<degree>();
        let mut best = current;
        let mut found = false;

        for i in -1..=1 {
            let candidate = desired_deg + 360.0 * i as f64;

            if candidate < TURRET_MIN || candidate > TURRET_MAX {
                continue;
            }

            // this might have a logic error - rishi at lunch
            if !found || (candidate - current).abs() < (best - current).abs() {
                best = candidate;
                found = true;
            }
        }

        best
    }
}

pub fn get_angle_to_hub(pose: RobotPoseEstimate) -> Angle {
    let x = pose.x.get::<meter>();
    let y = pose.y.get::<meter>();
    let dx = HUB_X - x;
    let dy = HUB_X - y;

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
            Length::new::<meter>(0.),
            Length::new::<meter>(0.),
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
