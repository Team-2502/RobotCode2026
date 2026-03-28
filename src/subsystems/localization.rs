use nalgebra::{SMatrix, SVector, Vector2, matrix};
use std::ops::{Div, Sub};
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

use crate::constants::localization::{
    CURRENT_STATE_ANGULAR_VELOCITY_TRUST, CURRENT_STATE_DRIVE_TRUST,
    CURRENT_STATE_LINEAR_VELOCITY_TRUST, CURRENT_STATE_YAW_TRUST,
    LIMELIGHT_ACCEPTABLE_OUTLIER_COUNT, MAX_LIMELIGHT_POSE_DIFFERENCE_METERS,
};

pub struct Localization {
    state: SMatrix<f64, 6, 1>,
    state_confidence: SMatrix<f64, 6, 6>,
    limelight_outlier_count: u32,
}

#[derive(Debug, Clone)]
pub struct RobotPose {
    pub x: Length,
    pub y: Length,
    pub yaw: Angle,
}

impl Sub for RobotPose {
    type Output = (Vector2<Length>, Angle);

    fn sub(self, other: Self) -> Self::Output {
        (
            Vector2::new(self.x - other.x, self.y - other.y),
            self.yaw - other.yaw,
        )
    }
}

impl Localization {
    pub fn new() -> Localization {
        let state = SMatrix::zeros();
        let state_confidence = SMatrix::<f64, 6, 6>::from_diagonal(&SVector::<f64, 6>::new(
            9999.9, 9999.9, 9999.9, 0.01, 0.01, 0.01,
        ));

        Localization {
            state,
            state_confidence,
            limelight_outlier_count: 0,
        }
    }

    fn set_state(&mut self, new: SMatrix<f64, 6, 1>) {
        self.state = new;
        self.state[(2, 0)] = wrap_angle(self.state[(2, 0)]);
    }

    fn update_measurement<const C: usize>(
        &mut self,
        measurement: SMatrix<f64, C, 1>,
        state_to_measurement: SMatrix<f64, C, 6>,
        measurement_confidence: SMatrix<f64, C, C>,
        angle_index: isize,
    ) {
        let mut innovation = measurement - (state_to_measurement * self.state);

        if angle_index >= 0 {
            innovation[(angle_index as usize, 0)] =
                wrap_angle(innovation[(angle_index as usize, 0)]);
        }

        let innovation_confidence =
            transform(state_to_measurement, self.state_confidence) + measurement_confidence;

        let gain: SMatrix<f64, 6, C> = innovation_confidence
            .cholesky()
            .unwrap()
            .solve(&(&self.state_confidence * state_to_measurement.transpose()).transpose())
            .transpose();

        self.set_state(self.state + gain * innovation);

        let eye = SMatrix::<f64, 6, 6>::identity();
        self.state_confidence = transform(eye - gain * state_to_measurement, self.state_confidence)
            + transform(gain, measurement_confidence);
        self.state_confidence = 0.5 * (self.state_confidence + self.state_confidence.transpose());
    }

    pub fn translation_from_odometry(
        &mut self,
        robot_translation: Vector2<f64>,
        yaw_change: Angle,
        robot_translation_error: Vector2<f64>,
        yaw_error: f64,
    ) {
        let pose_shift: SMatrix<f64, 3, 1> = matrix![
            robot_translation.x;
            robot_translation.y;
            yaw_change.get::<radian>();
        ];

        let field_yaw = self.state[(2, 0)];
        let field_sin = field_yaw.sin();
        let field_cos = field_yaw.cos();

        // ??? is this supposed to have sin things
        let yaw_rot_matrix = matrix![
            field_cos, -field_sin, 0.0;
            field_sin, field_cos, 0.0;
            0.0, 0.0, 1.0;
            0.0, 0.0, 0.0;
            0.0, 0.0, 0.0;
            0.0, 0.0, 0.0;
        ];

        // ??? also here
        let error_from_state = matrix![
            1.0, 0.0, (-field_sin * pose_shift[(0, 0)] - field_cos * pose_shift[(1, 0)]), 0.0, 0.0, 0.0;
            0.0, 1.0, (field_cos * pose_shift[(0, 0)] - field_sin * pose_shift[(1, 0)]), 0.0, 0.0, 0.0;
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        ];

        let error_from_prediction = yaw_rot_matrix;
        self.set_state(self.state + yaw_rot_matrix * pose_shift);

        let translation_x_error_squared = robot_translation_error.x * robot_translation_error.x;
        let translation_y_error_squared = robot_translation_error.y * robot_translation_error.y;
        let current_drive_trust_squared = CURRENT_STATE_DRIVE_TRUST * CURRENT_STATE_DRIVE_TRUST;

        let yaw_error_squared = yaw_error * yaw_error;
        let current_yaw_trust_squared = CURRENT_STATE_YAW_TRUST * CURRENT_STATE_YAW_TRUST;

        let prediction_confidence = matrix![
            translation_x_error_squared + current_drive_trust_squared, 0.0, 0.0;
            0.0, translation_y_error_squared + current_drive_trust_squared, 0.0;
            0.0, 0.0, yaw_error_squared + current_yaw_trust_squared;
        ];

        self.state_confidence = transform(error_from_state, self.state_confidence)
            + transform(error_from_prediction, prediction_confidence);

        self.state_confidence = 0.5 * (self.state_confidence + self.state_confidence.transpose());
    }

    pub fn update_yaw(&mut self, new_yaw: Angle, error: f64) {
        let measurement: SMatrix<f64, 1, 1> = matrix![new_yaw.get::<radian>();];
        let state_to_measurement = matrix![0.0, 0.0, 1.0, 0.0, 0.0, 0.0];
        let measurement_confidence = matrix![error * error;];
        self.update_measurement(measurement, state_to_measurement, measurement_confidence, 0);
    }

    pub fn update_velocities(
        &mut self,
        linear_velocity: Vector2<Length>,
        angular_velocity: Angle,
        linear_velocity_err: Vector2<Length>,
        angular_velocity_err: Angle,
    ) {
        let measurement: SMatrix<f64, 3, 1> = matrix![
            linear_velocity.x.get::<meter>();
            linear_velocity.y.get::<meter>();
            angular_velocity.get::<radian>();
        ];

        let state_to_measurement = matrix![
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0;
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0;
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        ];

        let measurement_confidence = matrix![
            linear_velocity_err.x.get::<meter>() * linear_velocity_err.x.get::<meter>(), 0.0, 0.0;
            0.0, linear_velocity_err.y.get::<meter>() * linear_velocity_err.y.get::<meter>(), 0.0;
            0.0, 0.0, angular_velocity_err.get::<radian>() * angular_velocity_err.get::<radian>();
        ];

        self.state_confidence[(3, 3)] += CURRENT_STATE_LINEAR_VELOCITY_TRUST;
        self.state_confidence[(4, 4)] += CURRENT_STATE_LINEAR_VELOCITY_TRUST;
        self.state_confidence[(5, 5)] += CURRENT_STATE_ANGULAR_VELOCITY_TRUST;

        self.update_measurement(
            measurement,
            state_to_measurement,
            measurement_confidence,
            -1,
        );
    }

    pub fn update_pose_from_limelight(
        &mut self,
        robot_pose: Vector2<Length>,
        new_yaw: Angle,
        pose_error: Vector2<Length>,
        yaw_error: Angle,
    ) -> bool {
        let current_state = self.get_state();
        let pose_diff_scalar = ((current_state.x - robot_pose.x)
            * (current_state.x - robot_pose.x)
            + (current_state.y - robot_pose.y) * (current_state.y - robot_pose.y))
            .sqrt();

        if pose_diff_scalar > Length::new::<meter>(MAX_LIMELIGHT_POSE_DIFFERENCE_METERS) {
            self.limelight_outlier_count += 1;

            if self.limelight_outlier_count < LIMELIGHT_ACCEPTABLE_OUTLIER_COUNT {
                return false;
            }
        }

        self.limelight_outlier_count = 0;

        let measurement: SMatrix<f64, 3, 1> = matrix![
            robot_pose.x.get::<meter>();
            robot_pose.y.get::<meter>();
            new_yaw.get::<radian>();
        ];

        let state_to_measurement = matrix![
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        ];

        let measurement_confidence = matrix![
            pose_error.x.get::<meter>() * pose_error.x.get::<meter>(), 0.0, 0.0;
            0.0, pose_error.y.get::<meter>() * pose_error.y.get::<meter>(), 0.0;
            0.0, 0.0, yaw_error.get::<radian>() * yaw_error.get::<radian>();
        ];

        self.update_measurement(measurement, state_to_measurement, measurement_confidence, 2);
        return true;
    }

    /// Returns: Pose, Yaw, Pose Error, Yaw Error
    pub fn get_state(&self) -> RobotPose {
        RobotPose {
            x: Length::new::<meter>(self.state[(0, 0)]),
            y: Length::new::<meter>(self.state[(1, 0)]),
            yaw: Angle::new::<radian>(self.state[(2, 0)]),
        }
    }

    pub fn get_velocities(&self) -> (Vector2<Length>, Angle) {
        (
            Vector2::new(
                Length::new::<meter>(self.state[(3, 0)]),
                Length::new::<meter>(self.state[(4, 0)]),
            ),
            Angle::new::<radian>(self.state[(5, 0)]),
        )
    }
}

fn wrap_angle(angle: f64) -> f64 {
    f64::atan2(angle.sin(), angle.cos())
}

fn transform<const R: usize, const C: usize>(
    transformation: SMatrix<f64, R, C>,
    mat: SMatrix<f64, C, C>,
) -> SMatrix<f64, R, R> {
    transformation * mat * transformation.transpose()
}

#[cfg(test)]
mod localization_tests {
    use super::*;
    use float_cmp::assert_approx_eq;
    use rand_distr::{Distribution, Normal};
    use uom::si::angle::degree;

    #[test]
    fn odometry_update_test() {
        // translations
        // translation x, translation y, yaw change, x acceleration, y acceleration, angular acceleration
        let inputs = vec![
            (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
            (-1.0, -1.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0, 0.0, 0.0, 0.0),
            (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ];

        let expected = vec![
            (1.0, 0.0, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0),
            (0.0, 0.0, 180.0),
            (0.0, 0.0, -90.0),
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 90.0),
            (0.0, -1.0, 90.0),
        ];

        let mut local = Localization::new();

        let mut results: Vec<RobotPose> = vec![];
        for input in inputs {
            local.translation_from_odometry(
                Vector2::new(input.0, input.1),
                Angle::new::<degree>(input.2),
                Vector2::new(0.0, 0.0),
                0.0,
            );
            results.push(local.get_state());
        }

        let mut i = 1.0;
        for result in results.clone() {
            println!(
                "result {}: x: {}, y: {}, yaw: {}",
                i,
                result.x.get::<meter>(),
                result.y.get::<meter>(),
                result.yaw.get::<degree>()
            );
            i += 1.0;
        }

        for tuple in expected.iter().zip(results.iter()) {
            assert_approx_eq!(f64, tuple.0.0, tuple.1.x.get::<meter>(), epsilon = 0.001);
            assert_approx_eq!(f64, tuple.0.1, tuple.1.y.get::<meter>(), epsilon = 0.001);
            assert_approx_eq!(f64, tuple.0.2, tuple.1.yaw.get::<degree>(), epsilon = 0.001);
        }
    }

    // #[test]
    // fn throwaway_velocity_test() {
    //     let mut local = Localization::new();
    //     println!("{:0.5}", local.state_confidence);
    //     local.update_velocities(
    //         Vector2::new(Length::new::<meter>(0.0), Length::new::<meter>(0.0)),
    //         Angle::new::<degree>(0.0),
    //         Vector2::new(Length::new::<meter>(1000.0), Length::new::<meter>(1000.0)),
    //         Angle::new::<degree>(0.01),
    //     );

    //     let err_smudge = 1.0 / 8.0;

    //     let mut v = 1.0;
    //     let noise = Normal::new(0.0, 0.25).unwrap();
    //     let normal1 = Normal::new(0.9, 0.5).unwrap();

    //     println!("v = {}", v);
    //     for i in 0..10 {
    //         let noise_sample = noise.sample(&mut rand::rng());
    //         let normal_sample = normal1.sample(&mut rand::rng());
    //         println!("noisy v: {}", v + noise_sample);
    //         local.update_velocities(
    //             Vector2::new(
    //                 Length::new::<meter>(v + noise_sample),
    //                 Length::new::<meter>(v + noise_sample),
    //             ),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(
    //                 Length::new::<meter>(normal_sample * err_smudge),
    //                 Length::new::<meter>(normal_sample * err_smudge),
    //             ),
    //             Angle::new::<degree>(normal1.sample(&mut rand::rng())),
    //         );
    //         println!("{}, {:?}", i, local.get_velocities());
    //         println!("{:0.5}", local.state_confidence);
    //     }

    //     v = 4.5;
    //     let normal2 = Normal::new(3.5, 1.5).unwrap();
    //     println!("");

    //     println!("v = {}", v);
    //     for i in 0..10 {
    //         let noise_sample = noise.sample(&mut rand::rng());
    //         let normal_sample = normal2.sample(&mut rand::rng());
    //         println!("noisy v: {}", v + noise_sample);
    //         local.update_velocities(
    //             Vector2::new(
    //                 Length::new::<meter>(v + noise_sample),
    //                 Length::new::<meter>(v + noise_sample),
    //             ),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(
    //                 Length::new::<meter>(normal_sample * err_smudge),
    //                 Length::new::<meter>(normal_sample * err_smudge),
    //             ),
    //             Angle::new::<degree>(normal2.sample(&mut rand::rng())),
    //         );
    //         println!("{}, {:?}", i + 6, local.get_velocities());
    //         println!("{:0.5}", local.state_confidence);
    //     }
    //     panic!()
    // }

    #[test]
    fn throwaway() {
        let mut local = Localization::new();
        local.update_velocities(
            Vector2::new(Length::new::<meter>(10.0), Length::new::<meter>(10.0)),
            Angle::new::<radian>(3.141592),
            Vector2::new(Length::new::<meter>(0.0001), Length::new::<meter>(0.0001)),
            Angle::new::<degree>(0.0001),
        );
        panic!();
    }

    // #[test]
    // fn limelight_update_test() {
    //     let mut local = Localization::new();
    //     local.translation_from_metry(
    //         Vector2::new(0.0, 0.0),
    //         Angle::new::<degree>(0.0),
    //         Vector2::new(0.0000001, 0.0000001),
    //         0.005,
    //         Vector2::new(0.0, 0.0),
    //         Angle::new::<degree>(0.0),
    //         Vector2::new(0.00001, 0.00001),
    //         0.005,
    //     );
    //     local.update_pose_from_limelight(
    //         Vector2::new(Length::new::<meter>(15.0), Length::new::<meter>(5.0)),
    //         Angle::new::<degree>(0.0),
    //         Vector2::new(Length::new::<meter>(0.001), Length::new::<meter>(0.001)),
    //         Angle::new::<degree>(0.05),
    //     );
    //     println!("1: {}: {:?}", 0.0, local.state);
    //     for i in 0..=15 {
    //         local.translation_from_odometry(
    //             Vector2::new(0.0, 0.0),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(0.0000001, 0.0000001),
    //             0.005,
    //             Vector2::new(10.0, 0.0),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(0.00001, 0.00001),
    //             0.005,
    //         );
    //         local.update_pose_from_limelight(
    //             Vector2::new(Length::new::<meter>(14.0), Length::new::<meter>(4.0)),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(Length::new::<meter>(0.001), Length::new::<meter>(0.001)),
    //             Angle::new::<degree>(0.05),
    //         );
    //         println!("2: {}: {:?}", i + 1, local.state);
    //     }
    //     for i in 0..=15 {
    //         local.translation_from_odometry(
    //             Vector2::new(0.0, 0.0),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(0.01, 0.01),
    //             0.005,
    //             Vector2::new(-10.0, 0.0),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(0.00001, 0.00001),
    //             0.005,
    //         );
    //         local.update_pose_from_limelight(
    //             Vector2::new(Length::new::<meter>(15.0), Length::new::<meter>(5.0)),
    //             Angle::new::<degree>(0.0),
    //             Vector2::new(Length::new::<meter>(0.001), Length::new::<meter>(0.001)),
    //             Angle::new::<degree>(0.05),
    //         );
    //         println!("3: {}: {:?}", i + 16, local.state);
    //     }
    //     local.update_yaw(Angle::new::<degree>(1000.00), 1000.0);
    //     panic!()
    // }
}
