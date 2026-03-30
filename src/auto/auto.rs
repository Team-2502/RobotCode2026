use crate::Ferris;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Serialize, Deserialize)]
pub enum Auto {
    Nothing,
    Test,
    DasAuto,
    Move,
}

impl Auto {
    pub fn from_dashboard(s: &str) -> Self {
        match s {
            "Nothing" => Auto::Nothing,
            "Test" => Auto::Test,
            "Das Auto" => Auto::DasAuto,
            "Move" => Auto::Move,
            _ => Auto::Nothing,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Auto::Nothing => "Nothing",
            Auto::Test => "Test",
            Auto::DasAuto => "Das Auto",
            Auto::Move => "Move", //_ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![Auto::Nothing, Auto::DasAuto, Auto::Move]
    }

    pub fn names() -> Vec<String> {
        Self::iterator()
            .iter()
            .map(|a| a.name().to_owned())
            .collect()
    }

    pub async fn run_auto<'a>(ferris: Rc<RefCell<Ferris>>, chosen: Auto) {
        match chosen {
            Auto::Test => {
                //est(Rc::clone(&ferris)).await.expect("Failed running auto");
            }
            Auto::Nothing => {
                println!("No auto was selected!");
            }
            Auto::DasAuto => {
                //das_auto(Rc::clone(&ferris)).await.expect("auto is geeked");
            }
            Auto::Move => {} //move_test(Rc::clone(&ferris)).await.expect("auto is geeked"),
        }
    }
}

// pub async fn test(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
//     let robot = robot.borrow_mut();
//     let mut drivetrain = robot.drivetrain.deref().borrow_mut();
//     let starting_pose = get_waypoint("test", 0).await?;

//     let _x: Length = Length::new::<meter>(starting_pose[0]);
//     let _y: Length = Length::new::<meter>(starting_pose[1]);
//     let _heading = Angle::new::<degree>(starting_pose[2]);

//     //drivetrain.set_pose_estimate(RobotPoseEstimate::new(1.0, x, y, heading));

//     drive("test", &mut drivetrain, 1).await?;

//     // let _ = timeout(Duration::from_secs_f64(0.5), async {
//     //     loop {
//     //         drivetrain.post_odo().await;
//     //     }
//     // });

//     drive("test", &mut drivetrain, 2).await?;
//     drive("test", &mut drivetrain, 3).await?;
//     drive("test", &mut drivetrain, 4).await?;
//     drive("test", &mut drivetrain, 5).await?;

//     Ok(())
// }

// pub async fn das_auto(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
//     println!("auto");
//     let robot = robot.borrow_mut();
//     let mut drivetrain = robot.drivetrain.deref().borrow_mut();
//     let shooter = robot.shooter.deref().borrow_mut();
//     let intake = robot.intake.deref().borrow_mut();

//     println!("borrowed");

//     let _hub = match alliance_station().red() {
//         true => Vector2::new(
//             Length::new::<meter>(HUB_RED.x),
//             Length::new::<meter>(HUB_RED.y),
//         ),
//         false => Vector2::new(
//             Length::new::<meter>(HUB_BLUE.x),
//             Length::new::<meter>(HUB_BLUE.y),
//         ),
//     };

//     println!("bouta drive");

//     drive("das_auto", &mut drivetrain, 1).await?;

//     println!("drove");

//     //shooter.shoot_to(pose, yaw, hub);

//     intake.set_handoff(1.0);

//     sleep(Duration::from_millis(5000)).await;

//     intake.set_intake_speed(0.5);

//     sleep(Duration::from_millis(8000)).await;

//     intake.stop();
//     shooter.turret.stop();
//     shooter.stop();
//     println!("done");

//     Ok(())
// }

// pub async fn move_test(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
//     let robot = robot.borrow_mut();
//     let mut drivetrain = robot.drivetrain.deref().borrow_mut();
//     //let shooter = robot.shooter.deref().borrow_mut();
//     //let intake = robot.intake.deref().borrow_mut();
//     drivetrain.move_towards(Angle::new::<degree>(0.0), 0.0, Angle::new::<degree>(45.0));
//     sleep(Duration::from_millis(5000)).await;
//     drivetrain.move_towards(Angle::new::<degree>(90.0), 0.5, Angle::new::<degree>(0.0));

//     Ok(())
// }
