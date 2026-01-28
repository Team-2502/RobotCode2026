use crate::constants::config::{HUB_X, HUB_Y};
use crate::constants::robotmap::shooter::{HOOD_MOTOR_ID, SHOOTER_MOTOR_ID, SHOOTER_SPEED};
use crate::constants::turret::{OFFSET, TOLERANCE};
use crate::subsystems::swerve::odometry::RobotPoseEstimate;
use frcrs::ctre::{ControlMode, Talon};
use uom::si::angle::radian;
use uom::si::angular_velocity::radian_per_second;
use uom::si::f32::Length;
use uom::si::length::meter;

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
    shooter_motor: Talon,
    hood_motor: Talon,
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor = Talon::new(SHOOTER_MOTOR_ID, Some("can0".to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some("can0".to_string()));
        Shooter {
            shooter_motor,
            hood_motor,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor.set(ControlMode::Percent, speed);
    }

    pub fn shoot(&self, on: bool) {
        if on {
            self.shooter_motor.set(ControlMode::Percent, SHOOTER_SPEED);
        } else {
            self.shooter_motor.set(ControlMode::Percent, 0.0);
        }
    }

    pub fn set_hood_motor(&mut self, position: f64) {
        self.hood_motor.set(ControlMode::Position, position);
    }
}

// fr ccw+ i think rad/s
// TODO: make unit tests to make sure im not crazy
pub fn shoot_on_move(
    pose: RobotPoseEstimate,
    linear: (f64, f64),
    angular: uom::si::f64::AngularVelocity,
    max_iter: i32,
) -> ShotSolution {
    let (vx, vy) = linear;

    let angle: f64 = pose.angle.get::<radian>();
    let base_x: f64 = pose.x.get::<meter>();
    let base_y: f64 = pose.y.get::<meter>();
    let av: f64 = angular.get::<radian_per_second>();

    let ox: f64 = OFFSET.x;
    let oy: f64 = OFFSET.y;

    let rx = ox * angle.cos() - oy * angle.sin();
    let ry = ox * angle.sin() + oy * angle.cos();

    let turret_vx = vx - av * ry;
    let turret_vy = vy + av * rx;

    let mut dx = HUB_X - (base_x + rx);
    let mut dy = HUB_Y - (base_y + ry);
    let mut distance = (dx * dx + dy * dy).sqrt();

    let mut shot = lookup_shot(distance);
    let mut t = shot.time_of_flight;

    for _ in 0..max_iter {
        let px = base_x + rx + turret_vx * t;
        let py = base_y + ry + turret_vy * t;

        dx = HUB_X - px;
        dy = HUB_Y - py;
        distance = (dx * dx + dy * dy).sqrt();

        let new_shot = lookup_shot(distance);

        if (new_shot.time_of_flight - t).abs() < TOLERANCE {
            shot = new_shot;
            break;
        }

        t = new_shot.time_of_flight;
        shot = new_shot;
    }

    let future_angle = angle + av * shot.time_of_flight;
    let turret_angle = dy.atan2(dx) - future_angle;

    ShotSolution {
        hood_angle: shot.hood_angle,
        flywheel_speed: shot.flywheel_speed,
        time_of_flight: shot.time_of_flight,
        turret_angle,
    }
}

pub fn lookup_shot(distance: f64) -> ShooterData {
    if distance <= SHOOTING_TABLE[0].distance {
        return SHOOTING_TABLE[0];
    }

    let last = SHOOTING_TABLE.len() - 1;
    if distance >= SHOOTING_TABLE[last].distance {
        return SHOOTING_TABLE[last];
    }

    for i in 0..last {
        let s0 = SHOOTING_TABLE[i];
        let s1 = SHOOTING_TABLE[i + 1];

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
