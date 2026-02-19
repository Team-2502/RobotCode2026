use nalgebra::{Rotation2, Vector2};
use std::f64::consts::PI;
use uom::si::angle::radian;
use uom::si::f64::Angle;

/// ## Kinematics is a structure that stores vectors representing a swerve module's rotation unit vector.
/// The magnitude represents the speed of the module, and the direction of the vector represents the angle.
/// While the rotation vectors are constant, we don't want to calculate them each frame, and syntax like drivetrain.kinematics.calculate_targets is intuitive.
#[allow(unused)]
pub struct Kinematics {
    module_rotation_unit_vectors: Vec<Vector2<f64>>,
}

impl Kinematics {
    /// ## Calculates rotation unit vectors and returns a Kinematics.
    /// A rotation unit vector will rotate the robot on a dime when applied to the swerve modules.
    pub fn new() -> Kinematics {
        let half_width = crate::constants::config::WHEELBASE_WIDTH_INCHES / 2.0;
        let half_length = crate::constants::config::WHEELBASE_LENGTH_INCHES / 2.0;

        // vectors pointing to each module from center of robot.
        // convention is FL, BL, BR, FR
        let module_vectors: Vec<Vector2<f64>> = vec![
            Vector2::new(half_length, half_width),   //   FL
            Vector2::new(-half_length, half_width),  //  BL
            Vector2::new(-half_length, -half_width), // BR
            Vector2::new(half_length, -half_width),  //  FR
        ];

        // rotate each vector by 90 degrees and normalize. This will give us the rotation unit vectors.
        let mut final_vectors: Vec<Vector2<f64>> = Vec::new();
        let ninety_degree_rotation = Rotation2::new(PI / 2.0);
        for mut vector in module_vectors {
            // due to some underlying math, you have to do rotation * vector, not vector * rotation.
            // search up "non-communicative matrix multiplication" and "rotation matrix" (this is what Rotation2 actually is) for the underlying math.
            vector = ninety_degree_rotation * vector;
            vector = vector.normalize();
            final_vectors.push(vector);
        }

        Kinematics {
            module_rotation_unit_vectors: final_vectors,
        }
    }

    /// ## Given input from the driver station, return a Vec<(f64, Angle)> representing swerve module setpoints.
    /// The target_transformation vector is a vector comprised of the x and y input from the driver station.
    /// Returned vector follows swerve module order: Vec(1) = FL, 2 = BL, 3 = BR, 4 = FR.
    /// Returned vector f64 represents speed setpoint and the Angle represents swerve angle setpoint wrapped from -PI to PI.
    fn calculate_targets(
        &self,
        target_transformation: Vector2<f64>,
        input_rotation: f64,
    ) -> Vec<(f64, Angle)> {
        // println!("[DEBUG]: calculate_targets inputs: target_transform: {}, rot: {}", target_transformation, input_rotation);
        let mut module_setpoints: Vec<(f64, Angle)> = Vec::new();

        for rotation_unit_vector in &self.module_rotation_unit_vectors.clone() {
            // scale each rotation unit vector by the rotation amount.
            let rotation_vector = *rotation_unit_vector * input_rotation;

            // add the scaled rotation vector to the target transformation vector in order to get the final vector.
            let final_vector = target_transformation + rotation_vector;

            // do some trig to figure out angle of final vector in radians
            let final_angle = Angle::new::<radian>(f64::atan2(final_vector.y, final_vector.x));
            module_setpoints.push((final_vector.magnitude(), final_angle));
        }

        // println!("[DEBUG]: calculate_targets outputs: module_setpoints: {:?}", module_setpoints);
        module_setpoints
    }

    /// ## Scales the speed setpoints to be from -1 to 1.
    /// FYI: It is possible for speed to be >1 after calculate_targets.
    fn scale_targets(&self, targets: Vec<(f64, Angle)>) -> Vec<(f64, Angle)> {
        // println!("[DEBUG]: scale_targets inputs: targets: {:?}", targets);
        let mut scaled_targets: Vec<(f64, Angle)> = Vec::new();
        let mut max = 0.0;

        for target in targets.clone() {
            if target.0 > max {
                max = target.0;
            }
        }
        if max > 1.0 {
            for target in targets {
                let scaled = target.0 / max;
                scaled_targets.push((scaled, target.1));
            }
        } else {
            scaled_targets = targets;
        }
        // println!("[DEBUG]: scale_targets outputs: targets: {:?}", scaled_targets);
        scaled_targets
    }

    /// ## Returns swerve setpoints given driver station input.
    /// Positive rotation = clockwise.
    pub fn get_targets(
        &self,
        target_transformation: Vector2<f64>,
        rotation: f64,
    ) -> Vec<(f64, Angle)> {
        // println!("[DEBUG]: get_targets inputs: target_transform: {}, rot: {}", target_transformation, rotation);
        let mut targets = self.calculate_targets(target_transformation, rotation);
        targets = self.scale_targets(targets);
        // println!("[DEBUG]: get_targets outputs: target_transform: {:?}", targets);
        targets
    }
}

// run tests with
//      cargo test -- --nocapture
// to show prints even for successful tests.
// by default, rust will capture (delete) output (for our context, println!) from successful tests. --nocapture prevents that.
#[cfg(test)]
mod kinematics_tests {
    use super::*;
    use nalgebra::vector;

    #[test]
    fn kinematics_new_test() {
        let results = Kinematics::new();

        // these seemingly random numbers are coordinates on the unit circle, specifically sqrt2/2 = 0.707...
        let expected = vec![
            vector![-0.7071067811865475, 0.7071067811865475],
            vector![-0.7071067811865475, -0.7071067811865475],
            vector![0.7071067811865475, -0.7071067811865475],
            vector![0.7071067811865475, 0.7071067811865475],
        ];

        println!("expected: {:?}", expected);
        println!("results: {:?}", results.module_rotation_unit_vectors);
        assert_eq!(results.module_rotation_unit_vectors, expected);
    }

    #[test]
    fn calculate_targets_right_full_power_test() {
        println!("calculate_targets_right_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![1.0, 0.0], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (1.0, Angle::new::<radian>(0.0)),
            (1.0, Angle::new::<radian>(0.0)),
            (1.0, Angle::new::<radian>(0.0)),
            (1.0, Angle::new::<radian>(0.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_right_half_power_test() {
        println!("calculate_targets_right_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.5, 0.0], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (0.5, Angle::new::<radian>(0.0)),
            (0.5, Angle::new::<radian>(0.0)),
            (0.5, Angle::new::<radian>(0.0)),
            (0.5, Angle::new::<radian>(0.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_left_full_power_test() {
        println!("calculate_targets_left_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![-1.0, 0.0], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (1.0, Angle::new::<radian>(PI)),
            (1.0, Angle::new::<radian>(PI)),
            (1.0, Angle::new::<radian>(PI)),
            (1.0, Angle::new::<radian>(PI)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_left_half_power_test() {
        println!("calculate_targets_left_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![-0.5, 0.0], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (0.5, Angle::new::<radian>(PI)),
            (0.5, Angle::new::<radian>(PI)),
            (0.5, Angle::new::<radian>(PI)),
            (0.5, Angle::new::<radian>(PI)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_up_full_power_test() {
        println!("calculate_targets_up_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, 1.0], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (1.0, Angle::new::<radian>(PI / 2.0)),
            (1.0, Angle::new::<radian>(PI / 2.0)),
            (1.0, Angle::new::<radian>(PI / 2.0)),
            (1.0, Angle::new::<radian>(PI / 2.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_up_half_power_test() {
        println!("calculate_targets_up_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, 0.5], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (0.5, Angle::new::<radian>(PI / 2.0)),
            (0.5, Angle::new::<radian>(PI / 2.0)),
            (0.5, Angle::new::<radian>(PI / 2.0)),
            (0.5, Angle::new::<radian>(PI / 2.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_down_full_power_test() {
        println!("calculate_targets_down_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, -1.0], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (1.0, Angle::new::<radian>(PI / -2.0)),
            (1.0, Angle::new::<radian>(PI / -2.0)),
            (1.0, Angle::new::<radian>(PI / -2.0)),
            (1.0, Angle::new::<radian>(PI / -2.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_down_half_power_test() {
        println!("calculate_targets_down_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, -0.5], 0.0);
        let expected: Vec<(f64, Angle)> = vec![
            (0.5, Angle::new::<radian>(PI / -2.0)),
            (0.5, Angle::new::<radian>(PI / -2.0)),
            (0.5, Angle::new::<radian>(PI / -2.0)),
            (0.5, Angle::new::<radian>(PI / -2.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_clockwise_full_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, 0.0], 1.0);
        // floating point operations means 1.0 becomes 0.9999999999
        let expected: Vec<(f64, Angle)> = vec![
            (0.9999999999999999, Angle::new::<radian>(3.0 * PI / 4.0)),
            (0.9999999999999999, Angle::new::<radian>(-3.0 * PI / 4.0)),
            (0.9999999999999999, Angle::new::<radian>(-PI / 4.0)),
            (0.9999999999999999, Angle::new::<radian>(PI / 4.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_clockwise_half_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, 0.0], 0.5);
        // floating point operations means 0.5 becomes 0.49999999999999994
        let expected: Vec<(f64, Angle)> = vec![
            (0.49999999999999994, Angle::new::<radian>(3.0 * PI / 4.0)),
            (0.49999999999999994, Angle::new::<radian>(-3.0 * PI / 4.0)),
            (0.49999999999999994, Angle::new::<radian>(-PI / 4.0)),
            (0.49999999999999994, Angle::new::<radian>(PI / 4.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_counter_clockwise_full_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, 0.0], -1.0);
        // floating point operations means 1.0 becomes 0.9999999999
        let expected: Vec<(f64, Angle)> = vec![
            (0.9999999999999999, Angle::new::<radian>(-PI / 4.0)),
            (0.9999999999999999, Angle::new::<radian>(PI / 4.0)),
            (0.9999999999999999, Angle::new::<radian>(3.0 * PI / 4.0)),
            (0.9999999999999999, Angle::new::<radian>(-3.0 * PI / 4.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn calculate_targets_counter_clockwise_half_power_test() {
        println!("calculate_targets_clockwise_test:");
        let kinematics = Kinematics::new();

        let results = kinematics.calculate_targets(vector![0.0, 0.0], -0.5);
        // floating point operations means 0.5 becomes 0.49999999999999994
        let expected: Vec<(f64, Angle)> = vec![
            (0.49999999999999994, Angle::new::<radian>(-PI / 4.0)),
            (0.49999999999999994, Angle::new::<radian>(PI / 4.0)),
            (0.49999999999999994, Angle::new::<radian>(3.0 * PI / 4.0)),
            (0.49999999999999994, Angle::new::<radian>(-3.0 * PI / 4.0)),
        ];
        println!("expected: {:?}", expected);
        println!("results: {:?}", results);
        assert_eq!(expected, results);
    }

    #[test]
    fn scale_targets_test() {
        let kinematics = Kinematics::new();
        let target = kinematics.calculate_targets(vector![1.0, 1.0], 1.0);
        println!("targets: {:?}", target);
        let target = kinematics.scale_targets(target);
        println!("scaled targets: {:?}", target);

        assert_eq!(target[3].0, 1.0);
    }
}
