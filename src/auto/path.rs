use frcrs::alliance_station;
use frcrs::input::RobotState;
use std::time::Duration;
use tokio::fs::File;

use crate::constants::auto::{
    SWERVE_DRIVE_IE, SWERVE_DRIVE_KD, SWERVE_DRIVE_KF, SWERVE_DRIVE_KFA, SWERVE_DRIVE_KI,
    SWERVE_DRIVE_KP, SWERVE_DRIVE_MAX_ERR, SWERVE_TURN_KP,
};
use crate::constants::config::{HALF_FIELD_LENGTH_METERS, HALF_FIELD_WIDTH_METERS};
use crate::subsystems::swerve::drivetrain::Drivetrain;
use crate::vec_f64;
use frcrs::trajectory::Path;
use nalgebra::Vector2;
use tokio::io::AsyncReadExt;
use tokio::time::{Instant, sleep};
use uom::si::angle::{Angle, degree};
use uom::si::{
    angle::radian,
    f64::{Length, Time},
    length::meter,
    time::second,
    velocity::meter_per_second,
};

pub async fn drive(
    name: &str,
    drivetrain: &mut Drivetrain,
    waypoint_index: usize,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut path_content = String::new();
    File::open(format!("/home/lvuser/deploy/choreo/{}.traj", name))
        .await?
        .read_to_string(&mut path_content)
        .await?;

    let path = Path::from_trajectory(&path_content);
    let path_unwrapped = path?;
    let waypoints = path_unwrapped.waypoints();

    //println!("{}", waypoints.len());
    if waypoint_index >= waypoints.len() {
        return Err("waypoint index out of bounds".into());
    }

    let start_time = if waypoint_index == 0 {
        0.0
    } else {
        waypoints[waypoint_index - 1]
    };
    let end_time = waypoints[waypoint_index];

    follow_path_segment(drivetrain, path_unwrapped, start_time, end_time).await;

    drivetrain.control_drivetrain(0., 0., 0.);
    Ok(())
}

pub async fn follow_path_segment(
    drivetrain: &mut Drivetrain,
    path: Path,
    start_time: f64,
    end_time: f64,
) {
    let start = Instant::now();
    let _last_loop = Instant::now();

    loop {
        let state = RobotState::get();
        //let mut last_error = Vector2::zeros();
        let mut last_loop = Instant::now();
        let mut i = Vector2::zeros();

        if !state.enabled() {
            eprintln!("robot not enabled");
            return;
        }

        // drivetrain.update_limelight().await;
        // drivetrain.update_localization().await;
        drivetrain.update_pose().await;

        let now = Instant::now();
        let dt = now - last_loop;
        last_loop = now;

        let elapsed = start.elapsed().as_secs_f64() + start_time;

        if elapsed > end_time {
            return;
        }

        let pose = drivetrain.localization.get_state();

        let setpoint = if alliance_station().red() {
            path.get(Time::new::<second>(elapsed)).mirror(
                Length::new::<meter>(HALF_FIELD_LENGTH_METERS),
                Length::new::<meter>(HALF_FIELD_WIDTH_METERS),
            )
        } else {
            path.get(Time::new::<second>(elapsed))
        };

        let mut angle = setpoint.heading;
        let position = Vector2::new(setpoint.x, setpoint.y);
        println!("target: {:?} -- {:?}", position, angle);

        drivetrain.auto_set_angle(angle);
        drivetrain.auto_set_target(position);
        let velocity = Vector2::new(
            setpoint.velocity_x.get::<meter_per_second>(),
            setpoint.velocity_y.get::<meter_per_second>(),
        )
        .norm();
        drivetrain.auto_move(velocity);

        angle = Angle::new::<degree>(calculate_relative_target(
            pose.yaw.get::<degree>(),
            angle.get::<degree>(),
        ));

        let mut error_position =
            vec_f64(position) - Vector2::new(pose.x.get::<meter>(), pose.y.get::<meter>());
        let mut error_angle = angle;

        if error_position.abs().max() < SWERVE_DRIVE_IE {
            i += error_position;
        }

        if elapsed > path.length().get::<second>()
            && error_position.abs().max() < SWERVE_DRIVE_MAX_ERR
            && error_angle.abs().get::<radian>() < 0.075
        {
            break;
        }

        // error_angle *= SWERVE_TURN_KP;
        // error_position *= -SWERVE_DRIVE_KP;

        // let mut speed = error_position;

        // let velocity = Vector2::new(setpoint.velocity_x, setpoint.velocity_y);
        // let velocity = velocity.map(|x| x.get::<meter_per_second>());

        // let velocity_next = Vector2::new(setpoint.velocity_x, setpoint.velocity_y)
        //     .map(|x| x.get::<meter_per_second>());

        // let acceleration = (velocity_next - velocity) * 1000. / 20.;

        // speed += velocity * -SWERVE_DRIVE_KF;
        // speed += acceleration * -SWERVE_DRIVE_KFA;
        // speed += i * -SWERVE_DRIVE_KI * dt.as_secs_f64();

        // let speed_s = speed;
        // speed += (speed - last_error) * SWERVE_DRIVE_KD * dt.as_secs_f64();
        // last_error = speed_s;

        // drivetrain.control_drivetrain(speed.x, speed.y, error_angle.get::<degree>());

        sleep(Duration::from_millis(20)).await;
    }
}

pub fn calculate_relative_target(current: f64, target: f64) -> f64 {
    let target_relative = target + (current / 360.0).floor() * 360.0;

    // Ensure the relative target is the closest possible to the current angle
    if target_relative - current > 180.0 {
        target_relative - 360.0
    } else if target_relative - current < -180.0 {
        target_relative + 360.0
    } else {
        target_relative
    }
}

pub async fn get_waypoint(
    name: &str,
    waypoint_index: usize,
) -> Result<Vec<f64>, Box<dyn std::error::Error>> {
    let mut path_content = String::new();
    File::open(format!("/home/lvuser/deploy/choreo/{}.traj", name))
        .await?
        .read_to_string(&mut path_content)
        .await?;

    let path = Path::from_trajectory(&path_content)?;
    let waypoints = path.waypoints();

    if waypoint_index >= waypoints.len() {
        return Err("waypoint index out of bounds".into());
    }

    // Waypoints are times (seconds) along the trajectory
    let waypoint_time = waypoints[waypoint_index];

    let pose = path.get(Time::new::<second>(waypoint_time));

    Ok(vec![
        pose.x.get::<meter>(),
        pose.y.get::<meter>(),
        pose.heading.get::<degree>(),
    ])
}
