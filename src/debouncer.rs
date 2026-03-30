use std::time::Instant;

pub struct Debouncer {
    last_input: Instant,
}

impl Debouncer {
    pub fn new() -> Debouncer {
        Debouncer {
            last_input: Instant::now(),
        }
    }

    pub fn debounce(&mut self, input: bool) -> bool {
        if input == false {
            return input;
        }
        let now = Instant::now();
        if now.duration_since(self.last_input).as_secs_f64() > 0.25 {
            self.last_input = now;
            return true;
        }
        false
    }
}
