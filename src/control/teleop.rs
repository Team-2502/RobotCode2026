use crate::Ferris;
use crate::control::fueler::Fueler;
use crate::control::swerve::Swerve;

pub struct Teleop {
    swerve: Swerve,
    fueler: Fueler,
}

impl Teleop {
    pub fn new() -> Teleop {
        Teleop {
            swerve: Swerve::new(),
            fueler: Fueler::new(),
        }
    }

    pub async fn update(&mut self, ferris: &mut Ferris) {
        self.swerve.update(ferris).await;
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        self.swerve.act(ferris);
        self.fueler.act(ferris);
    }
}
