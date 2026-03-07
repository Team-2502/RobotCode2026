use crate::constants::config::{HALF_FIELD_LENGTH_METERS, HALF_FIELD_WIDTH_METERS};
use crate::constants::robotmap::shooter::{
    HOOD_MOTOR_ID, SHOOTER_MOTOR_LEFT_ID, SHOOTER_MOTOR_RIGHT_ID,
};
use crate::constants::shooter::{MAX_FLYWHEEL_SPEED, SHOOTER_DISTANCE_ERROR_SMUDGE};
use crate::constants::turret::{OFFSET, TOLERANCE};
use crate::subsystems::swerve::drivetrain::get_angle_difs;
use crate::subsystems::swerve::kinematics::RobotPoseEstimate;
use crate::subsystems::turret::{Turret, get_angle_to_hub};
use crate::subsystems::vision::distance;
use frcrs::alliance_station;
use frcrs::ctre::{ControlMode, Talon};
use frcrs::telemetry::Telemetry;
use nalgebra::Vector2;
use uom::si::angle::radian;
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::length::meter;

#[derive(PartialEq, Clone)]
pub enum ShootingTarget {
    Hub,
    PassTop,
    PassBottom,
    PassTelemetry,
}

#[derive(Clone, Copy)]
pub struct ShooterData {
    distance: f64,
    flywheel_speed: f64,
    time_of_flight: f64,
    hood_angle: f64,
}

#[derive(Clone, Copy)]
pub struct ShotSolution {
    pub hood_angle: f64,
    pub flywheel_speed: f64,
    pub time_of_flight: f64,
    pub turret_angle: f64,
}

pub const SHOOTING_TABLE: [ShooterData; 10] = [
    ShooterData {
        // 91 in
        distance: 2.31,
        flywheel_speed: 0.69,
        //<77 >60
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        // 62 37 0.596
        // 153 in
        // 128 3.25
        distance: 3.89,
        flywheel_speed: 0.78,
        time_of_flight: 0.0,
        hood_angle: 0.0,
        // -1.5
    },
    ShooterData {
        distance: 0.0,
        // 60 in
        flywheel_speed: 0.0,
        // 0.55
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        // 18
        distance: 0.0,
        flywheel_speed: 0.0,
        // 0.0 1.0 98
        time_of_flight: 0.0,
        hood_angle: 0.0,
        // 2.5 77 0.78
        //
        // 1.25  .67 73
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    // at 5ft hood: 0.02 speed: ~46 +-0.2
    //
    // at 7ft hood: 0.02 speed: ~48.5 +-0.2
    // at 7ft hood: 0.41 speed: ~46.5 +-0.2
    //
    // at 9ft hood: 0.57 speed: ~49.5 +-0.2
    // at 9ft hood: 0.67 speed: ~49.5 +-0.2
    // at 9ft hood: 0.01 speed: ~54.5 +-0.2
    //
    // at 11ft hood: 0.02 speed: ~61.5 +-0.2
    // at 11ft hood: 0.78 speed: ~51.5 +-0.2
    //
    // at 13ft hood: 0.01 speed: ~67.5 +-0.2
    // at 13ft hood: 0.67 speed: ~56.5 +-0.2
    // at 13ft hood: 0.87 speed: ~54.5 +-0.2
    //
    // at 17ft hood: 0.01 speed: ~78.5 +-0.2
    //
    // at 18ft hood: 0.01 speed: ~83.5 +-0.2
    // at 18ft hood: 0.98 speed: ~66.5 +-0.2
];

pub const PASSING_TABLE: [ShooterData; 10] = [
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
    ShooterData {
        distance: 0.0,
        flywheel_speed: 0.0,
        time_of_flight: 0.0,
        hood_angle: 0.0,
    },
];

pub struct Shooter {
    shooter_motor_left: Talon,
    shooter_motor_right: Talon,
    hood_motor: Talon,
    

    pub turret: Turret,
}
impl ShootingTarget {
    pub fn name(&self) -> &'static str {
        match self {
            ShootingTarget::Hub => "hub",
            ShootingTarget::PassTop => "pass_l",
            ShootingTarget::PassBottom => "pass_r",
            //ShootingTarget::Idle => "idle",
            ShootingTarget::PassTelemetry => "telem",
        }
    }
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor_left = Talon::new(SHOOTER_MOTOR_LEFT_ID, Some("can0".to_string()));
        let shooter_motor_right = Talon::new(SHOOTER_MOTOR_RIGHT_ID, Some("can0".to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some("can0".to_string()));

        let turret = Turret::new();

        Shooter {
            shooter_motor_left,
            shooter_motor_right,
            hood_motor,

            turret,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Percent, speed);
        self.shooter_motor_right.set(ControlMode::Percent, -speed);
    }

    // pub fn shoot(&self, on: bool) {
    //     if on {
    //         self.shooter_motor.set(ControlMode::Percent, SHOOTER_SPEED);
    //     } else {
    //         self.shooter_motor.set(ControlMode::Percent, 0.0);
    //     }
    // }

    pub fn set_hood(&mut self, angle: f64) {
        self.hood_motor.set(ControlMode::Position, angle);
    }

    pub fn shoot_to(
        &mut self,
        current_pose: Vector2<Length>,
        current_yaw: Angle,
        target: Vector2<Length>,
    ) {
        let distance_target = Length::new::<meter>(distance(target, current_pose));

        let current_flywheel_speed = self.get_speed();
        self.set_velocity(get_shooter_speed_target(distance_target));
        self.set_hood(get_hood_angle_target(distance_target, current_flywheel_speed));
        self.turret
            .set_angle(get_angle_difs(current_yaw, get_angle_to_hub(current_pose)))
    }

    pub fn get_speed(&self) -> f64 {
        self.shooter_motor_left.get_velocity()
    }

    pub fn set_velocity(&self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Velocity, speed);
        self.shooter_motor_right.set(ControlMode::Velocity, speed);
    }

    pub fn get_hood(&self) -> f64 {
        self.hood_motor.get_position()
    }

    pub fn stop(&self) {
        self.hood_motor.stop();
        self.shooter_motor_left.stop();
        self.shooter_motor_right.stop();
        self.turret.stop();
    }
}

pub fn lookup_shot(distance: f64, target: ShootingTarget) -> ShooterData {
    let table = match target {
        ShootingTarget::Hub => SHOOTING_TABLE,
        _ => PASSING_TABLE,
    };

    if distance <= table[0].distance {
        return table[0];
    }

    let last = table.len() - 1;
    if distance >= table[last].distance {
        return table[last];
    }

    for i in 0..last {
        let s0 = table[i];
        let s1 = table[i + 1];

        if distance >= s0.distance && distance <= s1.distance {
            let t = (distance - s0.distance) / (s1.distance - s0.distance);

            return ShooterData {
                distance,
                hood_angle: s0.hood_angle + (s1.hood_angle - s0.hood_angle) * t,
                flywheel_speed: s0.flywheel_speed + (s1.flywheel_speed - s0.flywheel_speed) * t,
                time_of_flight: s0.time_of_flight + (s1.time_of_flight - s0.time_of_flight) * t,
            };
        }
    }

    unreachable!("if you see this its broken");
}

pub fn flip(target: Vector2<f64>) -> Vector2<f64> {
    if alliance_station().blue() {
        Vector2::new(
            target.x - HALF_FIELD_WIDTH_METERS + target.x,
            target.y - HALF_FIELD_LENGTH_METERS + target.y,
        )
    } else {
        target
    }
}

pub fn get_shooter_speed_target(distance: Length) -> f64 {
    let distance_feet: f64 =
        distance.get::<meter>() as f64 * 3.28084 * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let target =
        (0.0652772 * (distance_feet * distance_feet)) + (0.954121 * distance_feet) + 38.92606;

    target.clamp(0.0, MAX_FLYWHEEL_SPEED)
}

pub fn get_hood_angle_target(distance: Length, current_speed: f64) -> f64 {
    let distance_feet: f64 =
        distance.get::<meter>() as f64 * 3.28084 * SHOOTER_DISTANCE_ERROR_SMUDGE;
    let min_speed =
        (0.0917639 * (distance_feet * distance_feet)) + (-0.53771 * distance_feet) + 46.28489;
    let max_speed =
        (0.0496249 * (distance_feet * distance_feet)) + (1.84238 * distance_feet) + 34.54472;
    let max_angle =
        (-0.0076574 * (distance_feet * distance_feet)) + (0.24567 * distance_feet) + -0.978109;

    if max_angle < 0.0 || current_speed > max_speed {
        0.0
    } else if current_speed < min_speed {
        max_angle
    } else {
        let t = 1.0 - (current_speed - min_speed) / (max_speed - min_speed);
        max_angle * t
    }
}
