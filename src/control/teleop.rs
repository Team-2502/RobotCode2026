use crate::Ferris;
use crate::control::shooter::Shooter;
use crate::control::swerve::Swerve;

pub struct Teleop {
    swerve: Swerve,
    shooter: Shooter,
}

impl Teleop {
    pub fn new() -> Teleop {
        Teleop {
            swerve: Swerve::new(),
            shooter: Shooter::new(),
        }
    }

    pub async fn update(&mut self, ferris: &mut Ferris) {
        self.swerve.update(ferris).await;
        self.shooter.update(ferris).await;
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        self.swerve.act(ferris);
        self.shooter.act(ferris);
    }
}
