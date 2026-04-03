use frcrs::alliance_station;
use std::f64::consts::PI;
use tokio::time::Instant;
use uom::si::angle::radian;
use uom::si::f64::Angle;
use uom::si::length::Length;
use uom::si::length::meter;

use crate::Ferris;
use crate::control::auton::fueler_auton::AutonFueler;
use crate::control::auton::swerve_auton::AutonSwerve;

pub struct Auton {
    auton_swerve: AutonSwerve,
    auton_fueler: AutonFueler,
    start_time: Instant,
}

impl Auton {
    pub fn new() -> Auton {
        Auton {
            auton_swerve: AutonSwerve::new(),
            auton_fueler: AutonFueler::new(),
            start_time: Instant::now(),
        }
    }

    pub fn init(&mut self) {
        self.start_time = Instant::now();
    }

    pub async fn update(&mut self, ferris: &mut Ferris) {
        self.auton_swerve.update(ferris).await;
        self.auton_fueler.update(ferris).await;
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        let current_time = Instant::now().duration_since(self.start_time).as_secs_f64();
        if current_time < 1.0 {
            if alliance_station().red() {
                self.auton_swerve.control_drivetrain(
                    ferris,
                    Angle::new::<radian>(0.0),
                    Length::new::<meter>(1.0),
                    Angle::new::<radian>(0.0),
                );
            } else {
                self.auton_swerve.control_drivetrain(
                    ferris,
                    Angle::new::<radian>(PI),
                    Length::new::<meter>(1.0),
                    Angle::new::<radian>(0.0),
                );
            }
        } else if current_time < 3.0 {
            if alliance_station().red() {
                self.auton_swerve.turn_to(
                    ferris,
                    Angle::new::<radian>(0.0),
                    Length::new::<meter>(0.0),
                    Angle::new::<radian>(PI),
                );
            } else {
                self.auton_swerve.turn_to(
                    ferris,
                    Angle::new::<radian>(0.0),
                    Length::new::<meter>(0.0),
                    Angle::new::<radian>(0.0),
                );
            }
        } else if current_time < 8.0 {
            self.auton_fueler.shoot(ferris);
        }
        /* else if current_time < 12.0 { */
        // self.auton_swerve.turn_to(
        //     ferris,
        //     Angle::new::<radian>(PI / 2.0),
        //     Length::new::<meter>(0.4953 / 2.0 / 4.0),
        //     Angle::new::<radian>(0.0),
        // );
        /* } */
        else {
            ferris.stop();
        }
    }
}
