use crate::{Ferris, HANDOFF_SPEED, INTAKE_IN_SPEED, constants::shooter::MAX_FLYWHEEL_SPEED};
use frcrs::deadzone;
use uom::si::angle::degree;
use uom::si::f64::Angle;

struct Targeting {}

impl Targeting {
    pub fn new() -> Targeting {
        Targeting {}
    }

    fn act(&mut self, ferris: &mut Ferris) {
        if let Ok(intake) = ferris.intake.try_borrow_mut() {
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
                let mut percent_flywheel =
                    (ferris.controllers.left_drive.get_throttle() + 1.0) / 2.0;
                if percent_flywheel < 0.05 {
                    percent_flywheel = 0.0;
                }
                println!("percent: {}", percent_flywheel);
                shooter.set_velocity(MAX_FLYWHEEL_SPEED * percent_flywheel);
                shooter.set_hood(1.75);

                let deadzone_output = 0.0..1.0;
                let deadzone_input = 0.05..1.0;
                let deadzoned_z = deadzone(
                    ferris.controllers.operator.get_z(),
                    &deadzone_input,
                    &deadzone_output,
                );

                let yaw = -60.0 * deadzoned_z;

                shooter.turret.set_angle(Angle::new::<degree>(yaw));
            }
        } else {
            if let Ok(shooter) = ferris.shooter.try_borrow_mut() {
                shooter.stop();
            }
        }
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
