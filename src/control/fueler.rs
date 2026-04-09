use crate::{
    Ferris, HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED,
    constants::{
        config::{
            ESTIMATED_FRAME_TIME_SECONDS, HOOD_ROTATE_RATE_ROTS_PER_SEC,
            TURRET_ROTATE_RATE_RADS_PER_SEC,
        },
        shooter::MAX_FLYWHEEL_SPEED,
    },
};
use frcrs::deadzone;
use uom::si::angle::{degree, radian};
use uom::si::f64::Angle;

struct Targeting {
    turret_angle: Angle,
}

impl Targeting {
    pub fn new() -> Targeting {
        Targeting {
            turret_angle: Angle::new::<degree>(0.0),
        }
    }

    fn act(&mut self, ferris: &mut Ferris) {
        if let Ok(mut intake) = ferris.intake.try_borrow_mut() {
            if ferris.controllers.right_drive.get(4) && ferris.controllers.operator.get(1) {
                intake.set_intake_speed(INTAKE_IN_SPEED);
                intake.set_handoff(HANDOFF_SPEED);
            } else if ferris.controllers.right_drive.get(1) {
                intake.set_intake_speed(INTAKE_IN_SPEED);
                intake.set_handoff(HANDOFF_SPEED);
            } else if ferris.controllers.right_drive.get(2) {
                intake.set_intake_speed(-INTAKE_IN_SPEED);
                intake.set_handoff(-HANDOFF_SPEED);
            } else {
                intake.set_intake_speed(0.0);
                intake.set_handoff(0.0);
            }
        }
    }
}
struct Launcher {}

impl Launcher {
    fn new() -> Launcher {
        Launcher {}
    }

    fn act(&mut self, ferris: &mut Ferris) {
        if ferris
            .turret_toggle_debouncer
            .debounce(ferris.controllers.right_drive.get(3))
        {
            ferris.shooter_enabled = !ferris.shooter_enabled;
        }

        if ferris.shooter_enabled {
            if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
                let mut percent = (ferris.controllers.left_drive.get_throttle() + 1.0) / 2.0;
                if percent < 0.05 {
                    percent = 0.0;
                }
                println!("percent: {}", percent);
                shooter.set_velocity(MAX_FLYWHEEL_SPEED * percent);
                shooter.set_hood(1.75);
                shooter.turret.set_angle(Angle::new::<degree>(0.0));
            }
        } else {
            if let Ok(shooter) = ferris.shooter.try_borrow_mut() {
                shooter.stop();
            }
        }

        // if let Ok(intake) = ferris.intake.try_borrow_mut() {
        //     if ferris.controllers.right_drive.get(4) && ferris.controllers.operator.get(1) {
        //         intake.set_intake_speed(INTAKE_IN_SPEED);
        //         intake.set_handoff(HANDOFF_SPEED);
        //     } else if ferris.controllers.right_drive.get(1) {
        //         intake.set_intake_speed(INTAKE_IN_SPEED);
        //         intake.set_handoff(HANDOFF_SPEED);
        //     }

        //     if ferris.controllers.right_drive.get(1) {
        //         intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
        //         intake.set_handoff(-HANDOFF_SPEED);
        //     }

        //     // if ferris.controllers.operator.get(1) {
        //     //     intake.set_intake_speed(INTAKE_IN_SPEED);
        //     //     intake.set_handoff(HANDOFF_SPEED);
        //     // } else if ferris.controllers.operator.get(2) {
        //     //     intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
        //     //     intake.set_handoff(-HANDOFF_SPEED);
        //     // } else {
        //     //     intake.stop();
        //     // }
        // }
    }
}

pub struct Fueler {
    targeting: Targeting,
    launcher: Launcher,
}

impl Fueler {
    pub fn new() -> Fueler {
        Fueler {
            targeting: Targeting::new(),
            launcher: Launcher::new(),
        }
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        self.targeting.act(ferris);
        self.launcher.act(ferris);
    }
}
