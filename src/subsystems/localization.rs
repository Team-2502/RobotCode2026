use nalgebra::{SMatrix, SVector, Vector2, matrix};
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

use crate::constants::drivetrain::{
    CURRENT_STATE_TRUST_SCALAR_DRIVE, CURRENT_STATE_TRUST_SCALAR_YAW,
};
use crate::constants::vision::{
    LIMELIGHT_ACCEPTABLE_OUTLIER_COUNT, MAX_LIMELIGHT_POSE_DIFFERENCE_METERS,
};

pub struct Localization {
    state: SMatrix<f64, 3, 1>,
    state_confidence: SMatrix<f64, 3, 3>,
    limelight_outlier_count: u32,
}

#[derive(Debug, Clone)]
pub struct RobotPose {
    pub x: Length,
    pub y: Length,
    pub yaw: Angle,
}

impl Localization {
    pub fn new() -> Localization {
        let state = SMatrix::zeros();
        let state_confidence =
            SMatrix::<f64, 3, 3>::from_diagonal(&SVector::<f64, 3>::new(1000.0, 1000.0, 1000.0));

        Localization {
            state,
            state_confidence,
            limelight_outlier_count: 0,
        }
    }

    fn set_state(&mut self, new: SMatrix<f64, 3, 1>) {
        self.state = new;
        self.state[(2, 0)] = wrap_angle(self.state[(2, 0)])
    }

    fn update_measurement<const C: usize>(
        &mut self,
        measurement: SMatrix<f64, C, 1>,
        state_to_measurement: SMatrix<f64, C, 3>,
        measurement_confidence: SMatrix<f64, C, C>,
        angle_index: usize,
    ) {
        let mut innovation = measurement - (state_to_measurement * self.state);
        innovation[(angle_index, 0)] = wrap_angle(innovation[(angle_index, 0)]);

        let innovation_confidence =
            transform(state_to_measurement, self.state_confidence) + measurement_confidence;

        let gain: SMatrix<f64, 3, C> = innovation_confidence
            .cholesky()
            .unwrap()
            .solve(&(&self.state_confidence * state_to_measurement.transpose()).transpose())
            .transpose();

        self.set_state(self.state + gain * innovation);

        let eye = SMatrix::<f64, 3, 3>::identity();
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
        // velocity: Vector2<f64>,
        // angular_velocity: Angle,
    ) {
        let pose_shift: SMatrix<f64, 3, 1> = matrix![
            robot_translation.x;
            robot_translation.y;
            yaw_change.get::<radian>();
        ];

        let field_yaw = self.state[(2, 0)];
        let field_sin = field_yaw.sin();
        let field_cos = field_yaw.cos();

        let yaw_rot_matrix = matrix![
            field_cos, -field_sin, 0.0;
            field_sin, field_cos, 0.0;
            0.0, 0.0, 1.0;
        ];

        let error_from_state = matrix![
            1.0, 0.0, (-field_sin * pose_shift[(0, 0)] - field_cos * pose_shift[(1, 0)]);
            0.0, 1.0, (field_cos * pose_shift[(0, 0)] - field_sin * pose_shift[(1, 0)]);
            0.0, 0.0, 1.0;
        ];

        let error_from_prediction = yaw_rot_matrix;
        self.set_state(self.state + yaw_rot_matrix * pose_shift);

        let prediction_confidence = matrix![
            robot_translation_error.x * robot_translation_error.x + CURRENT_STATE_TRUST_SCALAR_DRIVE * CURRENT_STATE_TRUST_SCALAR_DRIVE, 0.0, 0.0;
            0.0, robot_translation_error.y * robot_translation_error.y + CURRENT_STATE_TRUST_SCALAR_DRIVE * CURRENT_STATE_TRUST_SCALAR_DRIVE, 0.0;
            0.0, 0.0, yaw_error * yaw_error + CURRENT_STATE_TRUST_SCALAR_YAW * CURRENT_STATE_TRUST_SCALAR_YAW;
        ];

        self.state_confidence = transform(error_from_state, self.state_confidence)
            + transform(error_from_prediction, prediction_confidence);

        self.state_confidence = 0.5 * (self.state_confidence + self.state_confidence.transpose());
    }

    pub fn update_yaw(&mut self, new_yaw: Angle, error: f64) {
        let measurement: SMatrix<f64, 1, 1> = matrix![new_yaw.get::<radian>();];
        let state_to_measurement = matrix![0.0, 0.0, 1.0];
        let measurement_confidence = matrix![error * error;];
        self.update_measurement(measurement, state_to_measurement, measurement_confidence, 0);
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

        let state_to_measurement = SMatrix::<f64, 3, 3>::identity();

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

    pub fn get_velocity_errors(&self, elapsed_time_secs: f64) -> (Vector2<f64>, Angle) {
        let x_error = self.state_confidence[(0, 0)];
        let y_error = self.state_confidence[(1, 1)];
        let yaw_error = self.state_confidence[(2, 2)];

        let x_velocity_error = x_error / elapsed_time_secs;
        let y_velocity_error = y_error / elapsed_time_secs;
        let angular_velocity_error = yaw_error / elapsed_time_secs;
        (
            Vector2::new(x_velocity_error, y_velocity_error),
            Angle::new::<radian>(angular_velocity_error),
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
    use uom::si::angle::degree;

    #[test]
    fn odometry_update_test() {
        // translations
        let inputs = vec![
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (-1.0, -1.0, 0.0),
            (0.0, 0.0, 90.0),
            (0.0, 0.0, 90.0),
            (0.0, 0.0, 90.0),
            (0.0, 0.0, 90.0),
            (0.0, 0.0, 90.0),
            (-1.0, 0.0, 0.0),
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

    #[test]
    fn limelight_update_test() {
        let mut local = Localization::new();
        local.translation_from_odometry(
            Vector2::new(0.0, 0.0),
            Angle::new::<degree>(0.0),
            Vector2::new(0.0000001, 0.0000001),
            0.005,
        );
        local.update_pose_from_limelight(
            Vector2::new(Length::new::<meter>(15.0), Length::new::<meter>(5.0)),
            Angle::new::<degree>(0.0),
            Vector2::new(Length::new::<meter>(0.001), Length::new::<meter>(0.001)),
            Angle::new::<degree>(0.05),
        );
        println!("{}: {:?}", 0.0, local.get_state());
        for i in 0..=15 {
            local.translation_from_odometry(
                Vector2::new(0.0, 0.0),
                Angle::new::<degree>(0.0),
                Vector2::new(0.0000001, 0.0000001),
                0.005,
            );
            local.update_pose_from_limelight(
                Vector2::new(Length::new::<meter>(14.0), Length::new::<meter>(4.0)),
                Angle::new::<degree>(0.0),
                Vector2::new(Length::new::<meter>(0.001), Length::new::<meter>(0.001)),
                Angle::new::<degree>(0.05),
            );
            println!("{}: {:?}", i + 1, local.get_state());
        }
        for i in 0..=15 {
            local.translation_from_odometry(
                Vector2::new(0.0, 0.0),
                Angle::new::<degree>(0.0),
                Vector2::new(0.01, 0.01),
                0.005,
            );
            local.update_pose_from_limelight(
                Vector2::new(Length::new::<meter>(15.0), Length::new::<meter>(5.0)),
                Angle::new::<degree>(0.0),
                Vector2::new(Length::new::<meter>(0.001), Length::new::<meter>(0.001)),
                Angle::new::<degree>(0.05),
            );
            println!("{}: {:?}", i + 16, local.get_state());
        }
        panic!()
    }
}
