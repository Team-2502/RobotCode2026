use crate::constants::config::{
    MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND, SHOOTER_INITAL_DISTANCE_OFFSET_FEET,
};
use crate::constants::localization::{
    COMMANDED_VELOCITY_WEIGHT, POSE_ANTICIPATION_TIMESTEP_SECS, YAW_ANTICIPATION_TIMESTEP_SECS,
};
use crate::constants::robotmap::shooter::{
    HOOD_MOTOR_ID, SHOOTER_CANBUS, SHOOTER_MOTOR_LEFT_ID, SHOOTER_MOTOR_RIGHT_ID,
};
use crate::constants::turret::{
    DISTANCE_SMUDGE_METERS, HOOD_MAX_SOFTSTOP, HOOD_MIN_SOFTSTOP, ORIGIN_TO_TURRET_CENTER_X_INCHES,
    ORIGIN_TO_TURRET_CENTER_Y_INCHES,
};
use crate::subsystems::localization::RobotPose;
use crate::subsystems::swerve::drivetrain::get_angle_difs;
use crate::subsystems::turret::{Turret, get_angle_to};
use crate::subsystems::vision::distance;
use frcrs::ctre::{ControlMode, Talon};
use nalgebra::{Rotation2, Vector2};
use std::fs::File;
use std::io::{Read, Write};
use uom::si::angle::{degree, radian, revolution};
use uom::si::f64::{Angle, Length};
use uom::si::length::{foot, inch, meter};

#[derive(PartialEq, Clone)]
pub enum ShootingTarget {
    Hub,
    PassTop,
    PassBottom,
    PassTelemetry,
}

pub struct Shooter {
    shooter_motor_left: Talon,
    shooter_motor_right: Talon,
    hood_motor: Talon,
    pub distance_offset: Length,
    pub turret: Turret,
    hood_offset: Angle,
}

impl Shooter {
    pub fn new() -> Shooter {
        let shooter_motor_left =
            Talon::new(SHOOTER_MOTOR_LEFT_ID, Some(SHOOTER_CANBUS.to_string()));
        let shooter_motor_right =
            Talon::new(SHOOTER_MOTOR_RIGHT_ID, Some(SHOOTER_CANBUS.to_string()));
        let hood_motor = Talon::new(HOOD_MOTOR_ID, Some(SHOOTER_CANBUS.to_string()));

        let turret = Turret::new();

        let file_result = File::open("/tmp/hood_zero");
        let mut hood_offset: Option<Angle> = None;

        if file_result.is_ok() {
            let mut file = file_result.unwrap();
            let metadata = file.metadata();
            if metadata.is_ok() {
                if metadata.unwrap().len() == 8 {
                    let mut buf = [0u8; 8];
                    if file.read_exact(&mut buf).is_ok() {
                        hood_offset = Some(Angle::new::<revolution>(f64::from_ne_bytes(buf)));
                    }
                }
            }
        }

        if hood_offset.is_none() {
            let zero = hood_motor.get_position();
            hood_offset = Some(Angle::new::<revolution>(zero));

            let zero_bytes = zero.to_ne_bytes();
            let new_file = File::create("/tmp/hood_zero");

            #[allow(unused_must_use)]
            if new_file.is_ok() {
                let mut created_file = new_file.unwrap();
                created_file.write_all(&zero_bytes);
                created_file.flush();
            }
        }

        Shooter {
            shooter_motor_left,
            shooter_motor_right,
            hood_motor,
            distance_offset: Length::new::<foot>(SHOOTER_INITAL_DISTANCE_OFFSET_FEET),
            hood_offset: hood_offset.unwrap(),

            turret,
        }
    }

    pub fn set_shooter(&mut self, speed: f64) {
        self.shooter_motor_left.set(ControlMode::Percent, speed);
        self.shooter_motor_right.set(ControlMode::Percent, -speed);
    }

    pub fn set_hood(&mut self, angle: f64) {
        self.hood_motor.set(
            ControlMode::Position,
            angle.clamp(HOOD_MIN_SOFTSTOP, HOOD_MAX_SOFTSTOP)
                + self.hood_offset.get::<revolution>(),
        );
    }

    pub fn shoot_to(
        &mut self,
        pose: &RobotPose,
        target: Vector2<Length>,
        commanded_theta: Angle,
        commanded_magnitude: Length,
    ) {
        let current_pose = Vector2::new(pose.x, pose.y);
        let future_pose = Vector2::new(
            pose.x + pose.vx * POSE_ANTICIPATION_TIMESTEP_SECS,
            pose.y + pose.vy * POSE_ANTICIPATION_TIMESTEP_SECS,
        );

        let mut vector_to_turret_center_f64 = Vector2::new(
            ORIGIN_TO_TURRET_CENTER_X_INCHES,
            ORIGIN_TO_TURRET_CENTER_Y_INCHES,
        );

        vector_to_turret_center_f64 =
            Rotation2::new(pose.yaw.get::<radian>()) * vector_to_turret_center_f64;

        let vector_to_turret_center = Vector2::new(
            Length::new::<inch>(vector_to_turret_center_f64.x),
            Length::new::<inch>(vector_to_turret_center_f64.y),
        );

        let velocity_vec = Vector2::new(pose.vx.get::<meter>(), pose.vy.get::<meter>());
        let commanded_vec = Vector2::new(
            f64::cos(commanded_theta.get::<radian>()) * commanded_magnitude.get::<meter>(),
            f64::sin(commanded_theta.get::<radian>()) * commanded_magnitude.get::<meter>(),
        );

        let weighted_vel_f64 = (1.0 - COMMANDED_VELOCITY_WEIGHT) * velocity_vec
            + COMMANDED_VELOCITY_WEIGHT * commanded_vec;
        let weighted_vel = Vector2::new(
            Length::new::<meter>(weighted_vel_f64.x),
            Length::new::<meter>(weighted_vel_f64.y),
        );

        let tr_velocity = get_target_relative_velocity_vector(weighted_vel, pose, target);
        let vx = tr_velocity.x.get::<meter>();
        let vy = tr_velocity.y.get::<meter>();

        let dist =
            distance(target, current_pose + vector_to_turret_center) + DISTANCE_SMUDGE_METERS;

        println!("vx {:0.5}, vy {:0.5}, dist: {:0.5}", vx, vy, dist);

        let future_dist = distance(target, future_pose + vector_to_turret_center);
        let current_flywheel_speed = self.get_speed();

        let speed = predict_hub_speed(future_dist, vx, vy);
        let hood = predict_hub_hood(dist, vx, vy, current_flywheel_speed);
        let angle_offset =
            Angle::new::<radian>(predict_yaw(dist, vx, vy, current_flywheel_speed, hood));

        let angle_target = get_angle_to(future_pose + vector_to_turret_center, target);
        let turret_relative_angle = get_angle_difs(
            pose.yaw + pose.vr * YAW_ANTICIPATION_TIMESTEP_SECS,
            angle_target + angle_offset,
        );

        self.set_velocity(speed);
        self.set_hood(hood);
        self.turret.set_angle(turret_relative_angle);
    }

    pub fn pass_to(
        &mut self,
        pose: &RobotPose,
        target: Vector2<Length>,
        commanded_theta: Angle,
        commanded_magnitude: Length,
    ) {
        let current_pose = Vector2::new(pose.x, pose.y);
        let future_pose = Vector2::new(
            pose.x + pose.vx * POSE_ANTICIPATION_TIMESTEP_SECS,
            pose.y + pose.vy * POSE_ANTICIPATION_TIMESTEP_SECS,
        );

        let mut vector_to_turret_center_f64 = Vector2::new(
            ORIGIN_TO_TURRET_CENTER_X_INCHES,
            ORIGIN_TO_TURRET_CENTER_Y_INCHES,
        );

        vector_to_turret_center_f64 =
            Rotation2::new(pose.yaw.get::<radian>()) * vector_to_turret_center_f64;

        let vector_to_turret_center = Vector2::new(
            Length::new::<inch>(vector_to_turret_center_f64.x),
            Length::new::<inch>(vector_to_turret_center_f64.y),
        );

        let velocity_vec = Vector2::new(pose.vx.get::<meter>(), pose.vy.get::<meter>());
        let commanded_vec = Vector2::new(
            f64::cos(commanded_theta.get::<radian>()) * commanded_magnitude.get::<meter>(),
            f64::sin(commanded_theta.get::<radian>()) * commanded_magnitude.get::<meter>(),
        );

        let weighted_vel_f64 = (1.0 - COMMANDED_VELOCITY_WEIGHT) * velocity_vec
            + COMMANDED_VELOCITY_WEIGHT * commanded_vec;
        let weighted_vel = Vector2::new(
            Length::new::<meter>(weighted_vel_f64.x),
            Length::new::<meter>(weighted_vel_f64.y),
        );

        let tr_velocity = get_target_relative_velocity_vector(weighted_vel, pose, target);
        let vx = tr_velocity.x.get::<meter>();
        let vy = tr_velocity.y.get::<meter>();

        let dist = distance(target, current_pose + vector_to_turret_center);

        let future_dist = distance(target, future_pose + vector_to_turret_center);
        let current_flywheel_speed = self.get_speed();

        let speed = predict_pass_speed(future_dist, vx, vy);
        let hood = predict_pass_hood(dist, vx, vy, current_flywheel_speed);
        let angle_offset =
            Angle::new::<radian>(predict_yaw(dist, vx, vy, current_flywheel_speed, hood));

        let angle_target = get_angle_to(future_pose + vector_to_turret_center, target);
        let turret_relative_angle = get_angle_difs(
            pose.yaw + pose.vr * YAW_ANTICIPATION_TIMESTEP_SECS,
            angle_target + angle_offset,
        );

        self.set_velocity(speed);
        self.set_hood(hood);
        self.turret.set_angle(turret_relative_angle);
    }

    pub fn get_speed(&self) -> f64 {
        (self.shooter_motor_left.get_velocity() + self.shooter_motor_right.get_velocity()) / 2.0
    }

    pub fn set_velocity(&self, speed: f64) {
        let scaled_speed = speed.clamp(30.0, 100.0);
        self.shooter_motor_left
            .set(ControlMode::Velocity, scaled_speed);
        self.shooter_motor_right
            .set(ControlMode::Velocity, scaled_speed);
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

/// converts the given vector into a coordinate system where +y is towards target, +x is tangent to target, (0, 0) is robot center
fn get_target_relative_velocity_vector(
    vector: Vector2<Length>,
    robot_pose: &RobotPose,
    target: Vector2<Length>,
) -> Vector2<Length> {
    // todo: impl robot_pose.to_vec
    let target_f64 = Vector2::new(target.x.get::<meter>(), target.y.get::<meter>());
    let pose_f64 = Vector2::new(robot_pose.x.get::<meter>(), robot_pose.y.get::<meter>());
    let vector_f64 = Vector2::new(vector.x.get::<meter>(), vector.y.get::<meter>());

    let y_unit_vector = (target_f64 - pose_f64).normalize();
    let x_unit_vector = Vector2::new(y_unit_vector.y, -1.0 * y_unit_vector.x).normalize();

    let target_relative_vector = Vector2::new(
        x_unit_vector.dot(&vector_f64),
        y_unit_vector.dot(&vector_f64),
    );

    Vector2::new(
        Length::new::<meter>(target_relative_vector.x),
        Length::new::<meter>(target_relative_vector.y),
    )
}

pub fn get_drivetrain_max_speed(
    robot_pose: &RobotPose,
    theta: Angle,
    target: Vector2<Length>,
) -> Length {
    let pose = Vector2::new(robot_pose.x, robot_pose.y);
    let angle_to_hub = get_angle_to(pose, target);
    let angle_diff = get_angle_difs(theta, angle_to_hub).abs();

    if angle_diff < Angle::new::<degree>(60.0) {
        let percent = angle_diff.get::<degree>() / 60.0;
        Length::new::<meter>(1.0 + (MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND - 1.0) * percent)
    } else {
        Length::new::<meter>(MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND)
    }
}

fn predict_hub_speed(dist: f64, vx: f64, vy: f64) -> f64 {
    let vx2 = vx * vx;
    let vy2 = vy * vy;
    let vxvy2 = vx2 + vy2;
    let dist2 = dist * dist;
    let speed = 48.26402033741077
        + 1.45313626590204 * dist
        + 0.28497933990905 * dist2
        + 2.08552942755273 * vy
        - 0.87492962508219 * vy * dist
        - 0.0310463487289 * vy * dist2
        + 0.59753175991412 * vxvy2
        + 0.02438070724366 * vxvy2 * dist;
    speed.clamp(30.0, 100.0)
}

fn predict_hub_hood(dist: f64, vx: f64, vy: f64, speed: f64) -> f64 {
    let flywheel = speed * 0.14577997574526508;
    let vx2 = vx * vx;
    let vy2 = vy * vy;
    let dist2 = dist * dist;
    let term1 = dist + flywheel;
    let term2 = flywheel + vy;
    if term1.abs() < 1e-9 || term2.abs() < 1e-9 {
        return 0.0;
    }
    let inv_term1 = 1.0 / term1;
    let inv_term2 = 1.0 / term2;
    let ux = vx;

    let uy = 2.31930632574723 + 0.87080314191958 * dist
        - 0.55831432371477 * flywheel
        - 1.00846814433622 * vy
        + 0.01653743700391 * dist2
        - 0.04748169042816 * dist * flywheel
        + 0.03367359917108 * flywheel * flywheel
        + 0.00499141699947 * vy * flywheel
        + 0.01341946850842 * vy * dist
        - 0.00095734093026 * vy * dist * flywheel
        + 0.02610786512318 * vx2
        - 0.00195469419643 * vx2 * flywheel
        - 0.00126738212072 * vy2
        + 0.08166170069528 * vx.hypot(vy) * inv_term1
        + 2.76899075054095 * dist * inv_term2;

    let uz = 0.09936205098565
        + 0.02104813399762 * dist
        + 1.11269483003193 * flywheel
        + 0.19605481403508 * vy
        - 0.02942888380699 * dist2
        + 0.02231266584791 * dist * flywheel
        - 0.00832592641409 * flywheel * flywheel
        - 0.01771030219332 * vy * flywheel
        + 0.0854337955929 * vy * dist
        - 0.00232108908698 * vy * dist * flywheel
        - 0.09090984838824 * vx2
        + 0.00395729340336 * vx2 * flywheel
        - 0.04211126605481 * vy2
        - 1.03180838142139 * vx.hypot(vy) * inv_term1
        - 3.13694287243635 * dist * inv_term2;
    let angle = uz.atan2(ux.hypot(uy)).to_degrees() / -12.56207643932571 + 5.565478313566728;
    angle + 0.17
}

fn predict_pass_speed(dist: f64, vx: f64, vy: f64) -> f64 {
    let vx2 = vx * vx;
    let vy2 = vy * vy;
    let vxvy2 = vx2 + vy2;
    let dist2 = dist * dist;
    let speed = 35.2969372164117 + 1.65508715011467 * dist + 0.3108259624189 * dist2
        - 1.46459651949985 * vy
        - 0.37911499468948 * vy * dist
        - 0.03985365245764 * vy * dist2
        + 0.14096777928633 * vxvy2
        + 0.03920102226153 * vxvy2 * dist;
    speed.clamp(30.0, 100.0)
}

fn predict_pass_hood(dist: f64, vx: f64, vy: f64, speed: f64) -> f64 {
    let flywheel = speed * 0.14577997574526508;
    let vx2 = vx * vx;
    let vy2 = vy * vy;
    let dist2 = dist * dist;
    let term1 = dist + flywheel;
    let term2 = flywheel + vy;
    if term1.abs() < 1e-9 || term2.abs() < 1e-9 {
        return 0.0;
    }
    let inv_term1 = 1.0 / term1;
    let inv_term2 = 1.0 / term2;
    let ux = vx;

    let uy = 2.03675058003524 + 0.95267923340142 * dist
        - 0.67709074682133 * flywheel
        - 0.91681616131232 * vy
        + 0.02489338139897 * dist2
        - 0.06862696241705 * dist * flywheel
        + 0.04705358150443 * flywheel * flywheel
        - 0.00308297663015 * vy * flywheel
        - 0.00664554065875 * vy * dist
        + 0.00046349869258 * vy * dist * flywheel
        + 0.07717481930137 * vx2
        - 0.00563683887608 * vx2 * flywheel
        + 0.00133046985465 * vy2
        - 0.29936462144772 * vx.hypot(vy) * inv_term1
        + 3.00745663885683 * dist * inv_term2;

    let uz = 0.36071337471016 - 0.29550545777051 * dist
        + 1.1566936615387 * flywheel
        + 0.13254521320313 * vy
        - 0.03946110656003 * dist2
        + 0.05787971099762 * dist * flywheel
        - 0.0158577396631 * flywheel * flywheel
        - 0.01459058154521 * vy * flywheel
        + 0.12068032746825 * vy * dist
        - 0.0044095155692 * vy * dist * flywheel
        - 0.13875113842158 * vx2
        + 0.00728004615683 * vx2 * flywheel
        - 0.04725195984191 * vy2
        - 0.53006481464723 * vx.hypot(vy) * inv_term1
        - 2.7343768311302 * dist * inv_term2;
    let angle = uz.atan2(ux.hypot(uy)).to_degrees() / -12.56207643932571 + 5.565478313566728;
    angle + 0.17
}

fn predict_yaw(dist: f64, vx: f64, vy: f64, speed: f64, hood: f64) -> f64 {
    let hood_calibrated = hood - 0.17;
    let launch_velocity = speed * (0.14577997574526508 - 0.00015049121874030828 * hood_calibrated);
    let launch_angle = (69.91396399643477 - 12.56207643932571 * hood_calibrated).to_radians();
    let tof = 0.1019367991845056 * (launch_velocity * launch_angle.sin()) + 0.8675195210938735;
    if tof.abs() < 1e-9 {
        return 0.0;
    }
    vx.atan2(1.34212043514101 * (dist / tof) - vy + 0.24667467825382)
}
