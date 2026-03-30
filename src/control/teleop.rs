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
        self.fueler.update(ferris).await;
    }

    pub fn act(&mut self, ferris: &mut Ferris) {
        let target = self.fueler.get_target();
        let mode = self.fueler.get_mode();
        self.swerve.act(ferris, target, mode);
        self.fueler.act(ferris);
    }
}
