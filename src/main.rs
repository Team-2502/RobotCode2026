use RobotCode2026::control::auton::auton::Auton;
use RobotCode2026::control::teleop::Teleop;
use RobotCode2026::subsystems::turret::TurretMode;
use RobotCode2026::{Ferris, post_shift};
use frcrs::input::RobotMode;
use frcrs::input::RobotState;
use frcrs::match_time;
use frcrs::networktables::NetworkTable;
use frcrs::telemetry::Telemetry;
use frcrs::{init_hal, observe_user_program_starting, refresh_data};
use std::cell::RefCell;
use std::f64::consts::PI;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::SystemTime;
use tokio::task;
use tokio::task::spawn_local;
use tokio::time::sleep;
use tokio::time::{Duration, Instant};
use uom::si::angle::degree;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

fn main() {
    let runtime = tokio::runtime::Runtime::new().unwrap();
    let local = task::LocalSet::new();

    runtime.block_on(local.run_until(async {
        // we check to make sure hal is initialized if not we panic
        if !init_hal() {
            panic!("Failed to initialize HAL");
        }

        // this starts robot code and shows it on the driver station
        observe_user_program_starting();

        // this initializes our telemetry server on port 5807
        Telemetry::init(5807);

        // we create our ferris here
        let ferris = Rc::new(RefCell::new(Ferris::new()));
        let mut teleop = Teleop::new();
        let mut auton = Auton::new();

        // this initializes network tables on the default port
        NetworkTable::init();

        // puts the auto chooser up on the telemetry server
        // Telemetry::put_selector("auto chooser", Auto::names()).await;
        Telemetry::put_selector("justice for cam :)", TurretMode::names()).await;
        Telemetry::put_selector(
            "airstrike mode",
            vec!["bomb the field".to_string(), "nah we chill".to_string()],
        )
        .await;

        Telemetry::put_color("shift", frcrs::telemetry::TelemetryColor::Purple).await;

        // this line is used if we are using a usb camera and want to see its feed on shuffleboard
        //SmartDashboard::start_camera_server();

        // set last loop time to now
        let mut last_loop = Instant::now();

        // Watchdog setup
        let last_loop_time = Arc::new(AtomicU64::new(0));
        let watchdog_last_loop = Arc::clone(&last_loop_time);
        let _watchdog_ferris = ferris.clone();

        // Spawn watchdog task
        spawn_local(async move {
            loop {
                sleep(Duration::from_millis(20)).await;
                let last = watchdog_last_loop.load(Ordering::Relaxed);
                let now = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64;

                // if more than 150 ms has passed the loop has overun and watchdog triggers
                if last != 0 && now - last > 150 {
                    println!("[WATCHDOG]: Loop Overrun: {}ms", now - last);
                }
            }
        });
        println!("loop");
        loop {
            // refresh the data like robot state
            refresh_data();

            // get the current state of the robot
            let state = RobotState::get();
            let dt = last_loop.elapsed();

            // if robot is not enabled make sure motors are stopped
            if !state.enabled() {
                if let Ok(f) = ferris.try_borrow() {
                    f.stop();
                } else {
                    println!("Didnt borrow ferris");
                }
            }

            Telemetry::put_number("match time", (match_time() * 100.0).trunc() / 100.0).await;

            if state.enabled() && state.teleop() {
                // if enabled and in teleop run the teleop function
                if let Ok(mut robot) = ferris.try_borrow_mut() {
                    robot.dt = dt;
                    robot.update_state();
                    post_shift(match_time()).await;
                    teleop.update(&mut robot).await;
                    teleop.act(&mut robot);
                }
            }

            if state.enabled() && state.auto() {
                if let Ok(mut robot) = ferris.try_borrow_mut() {
                    if robot.state.mode() != RobotMode::Auto {
                        robot.auto_init();
                        auton.init();
                    }
                    robot.update_state();
                    auton.update(&mut robot).await;
                    auton.act(&mut robot);
                    //robot.auto_periodic().await;
                    // auton.run_auton_frame(&mut robot).await;
                }
            }

            if state.enabled() && state.test() {
                // if enabled and in teleop run the teleop function
                if let Ok(robot) = ferris.try_borrow_mut() {
                    let current_time = Instant::now()
                        .duration_since(robot.start_time)
                        .as_secs_f64();
                    let cos = f64::cos(current_time * PI / 5.0);

                    let v = 40.0 + 10.0 * cos;
                    let yaw = Angle::new::<degree>(60.0 * cos);
                    let hood = 2.2992 / 2.0 + 2.0 * cos;
                    let mag = Length::new::<meter>(1.0);
                    let theta = Angle::new::<degree>(0.0);
                    let omega = Angle::new::<degree>(0.0);

                    if let Ok(mut drivetrain) = robot.drivetrain.try_borrow_mut() {
                        drivetrain.control_drivetrain(theta, mag, omega);
                    }

                    if let Ok(mut shooter) = robot.shooter.try_borrow_mut() {
                        shooter.set_velocity(v);
                        shooter.set_hood(hood);
                        shooter.turret.set_angle(yaw);
                    }
                }
            }
            //     //drivetrain.move_towards(Angle::new::<degree>(0.0), 0.0, Angle::new::<degree>(45.0));
            //     // Update dt before using it in auto
            //     if let Ok(mut ferris_mut) = ferris.try_borrow_mut() {
            //         ferris_mut.dt = dt;

            //         // Now access drivetrain
            //         if let Ok(mut drivetrain) = ferris_mut.drivetrain.try_borrow_mut() {
            //             drivetrain.update_pose().await;
            //             let pose = drivetrain.localization.get_state();
            //             //update_telemetry_robot_pose(&pose).await;
            //         }
            //     }

            //     //let run = Auto::run_auto(ferris, Auto::Move);
            //     //auto = Some(local.spawn_local(run).abort_handle());

            //     // start auto
            //     if auto.is_none() {
            //         // create a ferris clone to use in auto
            //         let ferris_clone = Rc::clone(&ferris);

            //         // if an auto is chosen from telemetry run that
            //         if let Some(selected_auto) = Telemetry::get_selection("auto chooser").await {
            //             let chosen = Auto::from_dashboard(selected_auto.as_str());

            //             let run = Auto::run_auto(ferris_clone, chosen);
            //             auto = Some(local.spawn_local(run).abort_handle());
            //         } else {
            //             // if no auto is chosen run the default auto (nothing)
            //             eprintln!("Failed to get selected auto from telemetry, running default");

            //             let run = Auto::run_auto(ferris_clone, Auto::Nothing);
            //             auto = Some(local.spawn_local(run).abort_handle());
            //         }
            //     }
            // } else if let Some(auto) = auto.take() {
            //     // if auto is already running before it should abort it
            //     println!("Aborted");
            //     auto.abort();
            // }

            // post our loop rate to telemetry
            Telemetry::put_number("Loop Rate", 1. / dt.as_secs_f64()).await;

            // update watchdog
            let now_millis = SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;
            last_loop_time.store(now_millis, Ordering::Relaxed);

            // enforce 250 hz timing
            let elapsed = dt.as_secs_f64();
            let left = (1. / 250. - elapsed).max(0.);
            sleep(Duration::from_secs_f64(left)).await;

            last_loop = Instant::now();
        }
    }));
}
