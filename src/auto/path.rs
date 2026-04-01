use frcrs::alliance_station;
use frcrs::input::RobotState;
use std::time::Duration;
use tokio::fs::File;

use crate::constants::config::MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND;
use crate::constants::config::{HALF_FIELD_LENGTH_METERS, HALF_FIELD_WIDTH_METERS};
use crate::subsystems::swerve::drivetrain::Drivetrain;
use crate::subsystems::swerve::drivetrain::get_angle_difs;
use frcrs::trajectory::Path;
use nalgebra::Vector2;
use pid::Pid;
use serde_json::Value;
use tokio::io::AsyncReadExt;
use tokio::time::{Instant, sleep};
use uom::si::angle::degree;
use uom::si::f64::Angle;
use uom::si::{
    angle::radian,
    f64::{Length, Time},
    length::meter,
    time::second,
    velocity::meter_per_second,
};

#[derive(Clone)]
pub struct Auto {
    pub start_time: Instant,
    pub pid_turn_to: Pid<f64>,
    pub target_point: Option<Vector2<Length>>,
    pub current_sample: usize,
}

pub struct Sample {
    pub x: f64,
    pub y: f64,
    pub heading: f64,

    pub vx: f64,
    pub vy: f64,
}

impl Auto {
    pub fn new() -> Self {
        let pid_turn_to: Pid<f64> =
            Pid::new(999.0, MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND);

        Self {
            start_time: Instant::now(),
            pid_turn_to,
            target_point: None,
            current_sample: 0,
        }
    }

    pub async fn drive(
        &mut self,
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

        if waypoint_index >= waypoints.len() {
            return Err("waypoint index out of bounds".into());
        }

        let start_time = if waypoint_index == 0 {
            0.0
        } else {
            waypoints[waypoint_index - 1]
        };
        let end_time = waypoints[waypoint_index];

        self.follow_path_segment(drivetrain, path_unwrapped, start_time, end_time)
            .await;

        drivetrain.control_drivetrain(
            Angle::new::<degree>(0.0),
            Length::new::<meter>(0.0),
            Angle::new::<degree>(0.0),
        );
        Ok(())
    }

    pub async fn follow_path_segment(
        &mut self,
        drivetrain: &mut Drivetrain,
        path: Path,
        start_time: f64,
        end_time: f64,
    ) {
        let start = self.start_time;

        loop {
            let state = RobotState::get();
            //let mut i = Vector2::zeros();

            if !state.enabled() {
                eprintln!("robot not enabled");
                return;
            }

            // drivetrain.update_limelight().await;
            // drivetrain.update_localization().await;
            drivetrain.update_pose().await;

            let elapsed = start.elapsed().as_secs_f64() + start_time;

            if elapsed > end_time {
                return;
            }

            let _pose = drivetrain.localization.get_state();

            let setpoint = if alliance_station().red() {
                path.get(Time::new::<second>(elapsed)).mirror(
                    Length::new::<meter>(HALF_FIELD_LENGTH_METERS),
                    Length::new::<meter>(HALF_FIELD_WIDTH_METERS),
                )
            } else {
                path.get(Time::new::<second>(elapsed))
            };

            let angle = setpoint.heading;
            let position = Vector2::new(setpoint.x, setpoint.y);
            println!("target: {:?} -- {:?}", position, angle);

            self.auto_set_angle(angle);
            self.set_target(position);
            let _velocity = Vector2::new(
                setpoint.velocity_x.get::<meter_per_second>(),
                setpoint.velocity_y.get::<meter_per_second>(),
            )
            .norm();
            //self.move_to(drivetrain, velocity);

            // angle = Angle::new::<degree>(calculate_relative_target(
            //     pose.yaw.get::<degree>(),
            //     angle.get::<degree>(),
            // ));

            // let error_position =
            //     vec_f64(position) - Vector2::new(pose.x.get::<meter>(), pose.y.get::<meter>());
            // let error_angle = angle;

            // if error_position.abs().max() < SWERVE_DRIVE_IE {
            //     i += error_position;
            // }

            // if elapsed > path.length().get::<second>()
            //     && error_position.abs().max() < SWERVE_DRIVE_MAX_ERR
            //     && error_angle.abs().get::<radian>() < 0.075
            // {
            //     break;
            // }

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

    pub fn move_towards(
        &mut self,
        drivetrain: &mut Drivetrain,
        angle: Angle,
        velocity: f64,
        rotate_rate: Angle,
    ) {
        drivetrain.control_drivetrain(angle, Length::new::<meter>(velocity), rotate_rate);
    }

    pub fn auto_set_angle(&mut self, angle: Angle) {
        if (angle.get::<radian>() - self.pid_turn_to.setpoint).abs() > 1e-4 {
            self.pid_turn_to = Pid::new(
                angle.get::<radian>(),
                MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND,
            );
            // 10.0
            self.pid_turn_to
                .p(2.0, MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND);
            // 40.0
            // self.auto_pid_turn_to.d(
            //     40.0,
            //     MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND * 10.0,
            // );
            //println!("setting angle");
        }
    }

    pub async fn move_to(
        &mut self,
        drivetrain: &mut Drivetrain,
        _velocity: f64,
        name: &str,
        index: usize,
    ) {
        let pose = drivetrain.localization.get_state();
        let setpoint = self.pid_turn_to.setpoint;
        let error = get_angle_difs(pose.yaw, Angle::new::<radian>(setpoint));
        let output = self
            .pid_turn_to
            .next_control_output(setpoint + error.get::<radian>());

        let distance = if self.target_point.is_some() {
            let current = Vector2::new(pose.x, pose.y);
            self.target_point.unwrap() - current
        } else {
            Vector2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0))
        };
        //println!("distance: {:?}", distance);
        //println!("pose: {:?}", pose);
        let angle = if index == 0 {
            f64::atan2(distance.y.get::<meter>(), distance.x.get::<meter>())
        } else {
            let current_sample = self.get_sample(name, index).await.unwrap();
            let prev_sample = self.get_sample(name, index - 1).await.unwrap();
            let current = Vector2::new(current_sample.x, current_sample.y);
            let prev = Vector2::new(prev_sample.x, prev_sample.y);
            let pose = Vector2::new(pose.x.get::<meter>(), pose.y.get::<meter>());

            let mut v = current - prev;
            let vm = v.norm();
            v = v / v.norm();

            let pose_from_start = pose - prev;
            let t = (v.x * pose_from_start.x + v.y * pose_from_start.y).clamp(0.0, vm);

            let tp = prev + (current - prev) * (0.5 * (vm + t) / vm);
            let distance = tp - pose;

            //println!("vm: {} t: {} tp: {}", vm, t, tp);
            f64::atan2(distance.y, distance.x)
        };
        //println!("angle: {}", angle);
        //let mut speed = (vec_f64(distance).magnitude() * 2.0).clamp(0.0, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND);
        // let mut speed = velocity.clamp(
        //     0.0,
        //     MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND,
        // );
        let mut speed = 1.0;
        //println!("speed: {}", speed);
        let err = distance.x.get::<meter>().abs() + distance.y.get::<meter>().abs();
        speed = if err < 0.05 { 0.0 } else { speed };
        self.move_towards(
            drivetrain,
            Angle::new::<radian>(angle),
            speed,
            Angle::new::<radian>(output.output),
        );
    }

    pub fn set_target(&mut self, target: Vector2<Length>) {
        self.target_point = Some(target);
    }

    pub async fn get_sample(
        &self,
        name: &str,
        index: usize,
    ) -> Result<Sample, Box<dyn std::error::Error>> {
        let mut path_content = String::new();
        File::open(format!("/home/lvuser/deploy/choreo/{}.traj", name))
            .await?
            .read_to_string(&mut path_content)
            .await?;

        let json: Value = serde_json::from_str(&path_content)?;
        let samples = json["trajectory"]["samples"].as_array().unwrap();
        let x = samples[index]["x"].as_f64().unwrap();
        let y = samples[index]["y"].as_f64().unwrap();
        let heading = samples[index]["heading"].as_f64().unwrap();
        let vx = samples[index]["vx"].as_f64().unwrap();
        let vy = samples[index]["vy"].as_f64().unwrap();

        Ok(if alliance_station().red() {
            Sample {
                x,
                y,
                heading,
                vx,
                vy,
            }
            .mirror()
        } else {
            Sample {
                x,
                y,
                heading,
                vx,
                vy,
            }
        })
    }

    pub async fn get_length(&self, name: &str) -> Result<usize, Box<dyn std::error::Error>> {
        let mut path_content = String::new();
        File::open(format!("/home/lvuser/deploy/choreo/{}.traj", name))
            .await?
            .read_to_string(&mut path_content)
            .await?;

        let json: Value = serde_json::from_str(&path_content)?;
        Ok(json["trajectory"]["samples"].as_array().unwrap().len())
    }

    pub async fn get_velocity(&self, name: &str, index: usize) -> f64 {
        let sample = self.get_sample(name, index).await.unwrap();
        Vector2::new(sample.x, sample.y).norm()
    }

    pub async fn at_sample(
        &self,
        drivetrain: &mut Drivetrain,
        _sample: Sample,
        name: &str,
    ) -> bool {
        let pose = drivetrain.localization.get_state();
        let distance = if self.target_point.is_some() {
            let current = Vector2::new(pose.x, pose.y);
            self.target_point.unwrap() - current
        } else {
            Vector2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0))
        };
        //println!("err: {}", distance.x.get::<meter>().abs() + distance.y.get::<meter>().abs());
        distance.x.get::<meter>().abs() + distance.y.get::<meter>().abs()
            < self.get_velocity(name, self.current_sample).await * 0.015
    }

    pub async fn move_to_sample(&mut self, name: &str, drivetrain: &mut Drivetrain, index: usize) {
        loop {
            let sample = self.get_sample(name, index).await.unwrap();
            // if self.current_sample == 10 {
            //     panic!("made it to point");
            // }
            self.set_target(Vector2::new(
                Length::new::<meter>(sample.x),
                Length::new::<meter>(sample.y),
            ));
            self.move_to(
                drivetrain,
                self.get_velocity(name, index).await,
                name,
                index,
            )
            .await;

            let past = if index != 0 {
                let pose = drivetrain.localization.get_state();
                let current_sample = self.get_sample(name, index).await.unwrap();
                let prev_sample = self.get_sample(name, index - 1).await.unwrap();
                let current = Vector2::new(current_sample.x, current_sample.y);
                let prev = Vector2::new(prev_sample.x, prev_sample.y);
                let pose = Vector2::new(pose.x.get::<meter>(), pose.y.get::<meter>());

                let mut v = current - prev;
                let vm = v.norm();
                v = v / v.norm();

                let pose_from_start = pose - prev;
                let t = v.x * pose_from_start.x + v.y * pose_from_start.y;
                println!(" {}:  t: {} vm: {}", self.current_sample, t, vm);
                t > vm
            } else {
                false
            };

            // false normally past
            if (past || self.at_sample(drivetrain, sample, name).await)
                && self.current_sample == index
                && self.current_sample + 1 < self.get_length(name).await.unwrap()
            {
                self.current_sample += 1;
            } else {
                break;
            }
        }
    }
}

impl Sample {
    pub fn mirror(&self) -> Sample {
        Sample {
            x: HALF_FIELD_LENGTH_METERS * 2.0 - self.x,
            y: HALF_FIELD_WIDTH_METERS * 2.0 - self.y,

            heading: self.heading + std::f64::consts::PI,

            vx: self.vx,
            vy: self.vy,
        }
    }
}

// pub fn calculate_relative_target(current: f64, target: f64) -> f64 {
//     let target_relative = target + (current / 360.0).floor() * 360.0;

//     // Ensure the relative target is the closest possible to the current angle
//     if target_relative - current > 180.0 {
//         target_relative - 360.0
//     } else if target_relative - current < -180.0 {
//         target_relative + 360.0
//     } else {
//         target_relative
//     }
// }
//
pub fn mirror_vec(vec: Vector2<Length>) -> Vector2<Length> {
    let x = HALF_FIELD_LENGTH_METERS * 2.0 - vec.x.get::<meter>();
    let y = HALF_FIELD_WIDTH_METERS * 2.0 - vec.y.get::<meter>();
    Vector2::new(Length::new::<meter>(x), Length::new::<meter>(y))
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
