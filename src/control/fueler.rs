use crate::{
    Ferris, HANDOFF_SPEED, INTAKE_IN_SPEED, INTAKE_REVSERSE_SPEED,
    constants::config::{
        ESTIMATED_FRAME_TIME_SECONDS, HOOD_ROTATE_RATE_ROTS_PER_SEC,
        TURRET_ROTATE_RATE_RADS_PER_SEC,
    },
};
use frcrs::deadzone;
use uom::si::angle::{degree, radian, revolution};
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
        if let Ok(mut shooter) = ferris.shooter.try_borrow_mut() {
            let deadzone_output = 0.0..1.0;
            let deadzone_input = 0.05..1.0;
            let deadzoned_x = deadzone(
                -ferris.controllers.right_drive.get_y(),
                &deadzone_input,
                &deadzone_output,
            );
            let deadzoned_y = deadzone(
                -ferris.controllers.right_drive.get_x(),
                &deadzone_input,
                &deadzone_output,
            );
            let deadzoned_z = deadzone(
                ferris.controllers.right_drive.get_z(),
                &deadzone_input,
                &deadzone_output,
            );

            let mag = (deadzoned_x * deadzoned_x + deadzoned_y * deadzoned_y)
                .sqrt()
                .clamp(-1.0, 1.0);

            let hood = shooter.get_hood()
                + mag * HOOD_ROTATE_RATE_ROTS_PER_SEC * ESTIMATED_FRAME_TIME_SECONDS;

            self.turret_angle += Angle::new::<radian>(
                deadzoned_z * TURRET_ROTATE_RATE_RADS_PER_SEC * ESTIMATED_FRAME_TIME_SECONDS,
            );

            shooter.set_hood(hood);
            shooter.turret.set_angle(self.turret_angle);
        }
    }
}
struct Launcher {}

impl Launcher {
    fn new() -> Launcher {
        Launcher {}
    }

    fn act(&mut self, ferris: &mut Ferris) {
        if let Ok(shooter) = ferris.shooter.try_borrow_mut() {
            let percent = (ferris.controllers.right_drive.get_throttle() + 1.0) / 2.0;
            shooter.set_velocity(30.0 + 60.0 * percent);
        }

        if let Ok(intake) = ferris.intake.try_borrow_mut() {
            if ferris.controllers.right_drive.get(4) && ferris.controllers.operator.get(1) {
                intake.set_intake_speed(INTAKE_IN_SPEED);
                intake.set_handoff(HANDOFF_SPEED);
            } else if ferris.controllers.right_drive.get(1) {
                intake.set_intake_speed(INTAKE_IN_SPEED);
                intake.set_handoff(HANDOFF_SPEED);
            }

            if ferris.controllers.right_drive.get(3) {
                intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
                intake.set_handoff(-HANDOFF_SPEED);
            }

            // if ferris.controllers.operator.get(1) {
            //     intake.set_intake_speed(INTAKE_IN_SPEED);
            //     intake.set_handoff(HANDOFF_SPEED);
            // } else if ferris.controllers.operator.get(2) {
            //     intake.set_intake_speed(INTAKE_REVSERSE_SPEED);
            //     intake.set_handoff(-HANDOFF_SPEED);
            // } else {
            //     intake.stop();
            // }
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
