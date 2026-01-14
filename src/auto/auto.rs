use crate::Ferris;
use crate::auto::path::{drive, get_waypoint};
use crate::subsystems::swerve::odometry::RobotPoseEstimate;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
use std::ops::Deref;
use std::rc::Rc;
use std::time::Duration;
use tokio::time::{sleep, timeout};
use uom::si::angle::degree;
use uom::si::f64::Angle;
use uom::si::f64::Length;
use uom::si::length::meter;

#[derive(Serialize, Deserialize)]
pub enum Auto {
    Nothing,
    Test,
}

impl Auto {
    pub fn from_dashboard(s: &str) -> Self {
        match s {
            "Nothing" => Auto::Nothing,
            "Test" => Auto::Test,
            _ => Auto::Nothing,
        }
    }

    pub fn name(&self) -> &'static str {
        match self {
            Auto::Nothing => "Nothing",
            Auto::Test => "Test",
            _ => "none",
        }
    }

    pub fn iterator() -> Vec<Self> {
        vec![Auto::Nothing]
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
                test(Rc::clone(&ferris)).await.expect("Failed running auto");
            }
            Auto::Nothing => {
                println!("No auto was selected!");
            }
        }
    }
}

pub async fn test(robot: Rc<RefCell<Ferris>>) -> Result<(), Box<dyn std::error::Error>> {
    let robot = robot.borrow_mut();
    let mut drivetrain = robot.drivetrain.deref().borrow_mut();
    let starting_pose = get_waypoint("test", 0).await?;

    let x: Length = Length::new::<meter>(starting_pose[0]);
    let y: Length = Length::new::<meter>(starting_pose[1]);
    let heading = Angle::new::<degree>(starting_pose[2]);

    drivetrain.set_pose_estimate(RobotPoseEstimate::new(1.0, x, y, heading));

    drive("test", &mut drivetrain, 1).await?;

    let _ = timeout(Duration::from_secs_f64(0.5), async {
        loop {
            drivetrain.post_odo().await;
        }
    });

    drive("test", &mut drivetrain, 2).await?;
    drive("test", &mut drivetrain, 3).await?;
    drive("test", &mut drivetrain, 4).await?;
    drive("test", &mut drivetrain, 5).await?;

    Ok(())
}
