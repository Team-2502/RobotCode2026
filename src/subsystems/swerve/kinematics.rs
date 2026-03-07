use crate::constants::config::{
    MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND, MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND,
    MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND, WHEELBASE_LENGTH_METERS, WHEELBASE_WIDTH_METERS,
};
use crate::constants::drivetrain::{SWERVE_WHEEL_CIRCUMFERENCE_METERS, WHEEL_ENCODER_STD_DEV};
use nalgebra::{SMatrix, Vector2, matrix};
use std::f64::consts::PI;
use std::time::Instant;
use uom::si::angle::revolution;
use uom::si::{angle::radian, f64::Angle, f64::Length};

/// Inverse Kinematics: Robot Transform/Rotation -> Wheel State
/// Forward Kinematics: Wheel State -> Robot Transform/Rotation
pub struct Kinematics {
    ik_matrix: SMatrix<f64, 8, 3>,
    fk_matrix: SMatrix<f64, 3, 8>,
    timer: Instant,
}

#[derive(Clone, Debug)]
pub struct RobotPoseEstimate {
    pub fom: f64,
    pub x: Length,
    pub y: Length,
    pub angle: Angle,
}

impl RobotPoseEstimate {
    pub fn new(fom: f64, x: Length, y: Length, angle: Angle) -> RobotPoseEstimate {
        Self { fom, x, y, angle }
    }
}

impl Kinematics {
    pub fn new() -> Kinematics {
        let half_width = WHEELBASE_WIDTH_METERS / 2.0;
        let half_length = WHEELBASE_LENGTH_METERS / 2.0;

        let module_positions = vec![
            (-half_width, half_length),
            (-half_width, -half_length),
            (half_width, -half_length),
            (half_width, half_length),
        ];

        let ik_matrix = matrix![
            1.0, 0.0, -module_positions[0].1;
            0.0, 1.0, module_positions[0].0;
            1.0, 0.0, -module_positions[1].1;
            0.0, 1.0, module_positions[1].0;
            1.0, 0.0, -module_positions[2].1;
            0.0, 1.0, module_positions[2].0;
            1.0, 0.0, -module_positions[3].1;
            0.0, 1.0, module_positions[3].0;
        ];

        let fk_matrix = matrix![
            1.0, 0.0, -module_positions[0].1;
            0.0, 1.0, module_positions[0].0;
            1.0, 0.0, -module_positions[1].1;
            0.0, 1.0, module_positions[1].0;
            1.0, 0.0, -module_positions[2].1;
            0.0, 1.0, module_positions[2].0;
            1.0, 0.0, -module_positions[3].1;
            0.0, 1.0, module_positions[3].0;
        ]
        .pseudo_inverse(1e-5)
        .expect("kinematics: failed to make fk_matrix: ");

        Kinematics {
            ik_matrix,
            fk_matrix,
            timer: Instant::now(),
        }
    }

    pub fn get_targets(&self, translation: Vector2<f64>, rot: f64) -> Vec<(f64, Angle)> {
        // input unit: m/s
        let input_matrix: SMatrix<f64, 3, 1> = matrix![
            translation.x * MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND;
            translation.y * MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND;
            rot * MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND;
        ];

        // in m/s
        let setpoint_matrix = self.ik_matrix.clone() * input_matrix;

        let mut setpoints = Vec::new();
        let mut max = 0.0;
        for i in 0..=3 {
            let x = setpoint_matrix[2 * i];
            let y = setpoint_matrix[2 * i + 1];
            let speed = (x * x + y * y).sqrt() / (SWERVE_WHEEL_CIRCUMFERENCE_METERS);

            if speed > max {
                max = speed;
            }

            let angle = Angle::new::<radian>(f64::atan2(y, x));
            setpoints.push((speed, angle));
        }

        let mut multiplier = 1.0;

        let mut scaled: Vec<(f64, Angle)> = Vec::new();
        if max > MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND {
            multiplier = MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND / max;
        }

        for mut setpoint in setpoints.clone() {
            setpoint.0 *= multiplier;
            scaled.push(setpoint);
        }

        scaled
    }

    /// Note: returned angle is a CHANGE
    /// Drive wheel rotation diff, turn wheel rotation diff
    pub fn odometry(
        &self,
        differences: Vec<(Angle, Angle)>,
        current_angles: Vec<Angle>,
    ) -> (Vector2<f64>, Angle, Vector2<f64>, f64) {
        let uncertainty_angle = WHEEL_ENCODER_STD_DEV * 2.0 * PI;
        let uncertainty_distance = WHEEL_ENCODER_STD_DEV;

        let wheel_uncertainties = matrix![
            uncertainty_distance * uncertainty_distance, uncertainty_angle * uncertainty_angle + (differences[0].1.get::<radian>() * differences[0].1.get::<radian>()) / 4.0;
            uncertainty_distance * uncertainty_distance, uncertainty_angle * uncertainty_angle + (differences[1].1.get::<radian>() * differences[1].1.get::<radian>()) / 4.0;
            uncertainty_distance * uncertainty_distance, uncertainty_angle * uncertainty_angle + (differences[2].1.get::<radian>() * differences[2].1.get::<radian>()) / 4.0;
            uncertainty_distance * uncertainty_distance, uncertainty_angle * uncertainty_angle + (differences[3].1.get::<radian>() * differences[3].1.get::<radian>()) / 4.0;
        ];

        let mut cov_setpoints_matrix: SMatrix<f64, 8, 8> = SMatrix::zeros();

        let mut setpoints_matrix: SMatrix<f64, 8, 1> = SMatrix::zeros();
        for i in 0..=3 {
            let distance = differences[i].0.get::<revolution>() * SWERVE_WHEEL_CIRCUMFERENCE_METERS;
            let angle = current_angles[i].get::<radian>();
            let angle_cos = angle.cos();
            let angle_sin = angle.sin();

            setpoints_matrix[(2 * i, 0)] = distance * angle_cos;
            setpoints_matrix[(2 * i + 1, 0)] = distance * angle_sin;

            let wheel_jacobian = matrix![
                SWERVE_WHEEL_CIRCUMFERENCE_METERS * angle_cos, -distance * angle_sin;
                SWERVE_WHEEL_CIRCUMFERENCE_METERS * angle_sin, distance * angle_cos;
            ];

            let wheel_std_dev = matrix![
                wheel_uncertainties[(i, 0)], 0.0;
                0.0, wheel_uncertainties[(i, 1)];
            ];

            cov_setpoints_matrix
                .view_mut((2 * i, 2 * i), (2, 2))
                .copy_from(&(wheel_jacobian * wheel_std_dev * wheel_jacobian.transpose()));
        }

        let translations_matrix = self.fk_matrix.clone() * setpoints_matrix;

        let model_error =
            (self.fk_matrix * cov_setpoints_matrix * self.fk_matrix.transpose()).diagonal();
        let model_error_x = translations_matrix[(0, 0)] * translations_matrix[(0, 0)] * 0.01 * 0.01;
        let model_error_y = translations_matrix[(1, 0)] * translations_matrix[(1, 0)] * 0.01 * 0.01;
        let model_error_angle =
            translations_matrix[(2, 0)] * translations_matrix[(2, 0)] * 0.05 * 0.05;

        let module_return_translation_error = Vector2::new(
            f64::sqrt(model_error[(0, 0)] + model_error_x),
            f64::sqrt(model_error[(1, 0)] + model_error_y),
        );
        let module_return_angle_error = f64::sqrt(model_error[(2, 0)] + model_error_angle);

        let translation_vector_meters =
            Vector2::new(translations_matrix[(0, 0)], translations_matrix[(1, 0)]);

        let yaw_change = Angle::new::<radian>(translations_matrix[(2, 0)]);

        (
            Vector2::new(translation_vector_meters.x, translation_vector_meters.y),
            yaw_change,
            module_return_translation_error,
            module_return_angle_error,
        )
    }
}
