use nalgebra::{ComplexField, SMatrix, SVector, Vector2, matrix};
use uom::si::angle::radian;
use uom::si::f64::{Angle, Length};
use uom::si::length::meter;

pub struct Localization {
    state: SMatrix<f64, 3, 1>,
    state_confidence: SMatrix<f64, 3, 3>,
}

impl Localization {
    pub fn new() -> Localization {
        let state = SMatrix::zeros();
        let state_confidence =
            SMatrix::<f64, 3, 3>::from_diagonal(&SVector::<f64, 3>::new(1000.0, 1000.0, 1000.0));

        Localization {
            state,
            state_confidence,
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
    ) {
        let mut pose_shift: SMatrix<f64, 3, 1> = matrix![
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

        let prediciton_confidence = matrix![
            robot_translation_error.x * robot_translation_error.x, 0.0, 0.0;
            0.0, robot_translation_error.y * robot_translation_error.y, 0.0;
            0.0, 0.0, yaw_error * yaw_error;
        ];

        self.state_confidence = transform(error_from_state, self.state_confidence)
            + transform(error_from_prediction, prediciton_confidence);

        self.state_confidence = 0.5 * (self.state_confidence + self.state_confidence.transpose());
    }

    pub fn update_yaw(&mut self, new_yaw: Angle, error: f64) {
        let measurement: SMatrix<f64, 1, 1> = matrix![new_yaw.get::<radian>();];
        let state_to_measurement = matrix![0.0, 0.0, 1.0];
        let measurement_confidence = matrix![error * error;];
        self.update_measurement(measurement, state_to_measurement, measurement_confidence, 0);
    }

    pub fn update_pose(
        &mut self,
        robot_pose: Vector2<Length>,
        new_yaw: Angle,
        pose_error: Vector2<Length>,
        yaw_error: Angle,
    ) {
        let measurement: SMatrix<f64, 3, 1> = matrix![
            robot_pose.x.get::<meter>();
            robot_pose.y.get::<meter>();
            yaw_error.get::<radian>();
        ];

        let state_to_measurement = SMatrix::<f64, 3, 3>::identity();
        let measurement_confidence = matrix![
            pose_error.x.get::<meter>() * pose_error.x.get::<meter>(), 0.0, 0.0;
            0.0, pose_error.y.get::<meter>() * pose_error.y.get::<meter>(), 0.0;
            0.0, 0.0, yaw_error.get::<radian>() * yaw_error.get::<radian>();
        ];
        self.update_measurement(measurement, state_to_measurement, measurement_confidence, 0);
    }

    /// Returns: Pose, Yaw, Pose Error, Yaw Error
    pub fn get_state(&self) -> (Vector2<Length>, Angle, Vector2<Length>, Angle) {
        (
            Vector2::new(
                Length::new::<meter>(self.state[(0, 0)]),
                Length::new::<meter>(self.state[(1, 0)]),
            ),
            Angle::new::<radian>(self.state[(2, 0)]),
            Vector2::new(
                Length::new::<meter>(self.state_confidence[(0, 0)]),
                Length::new::<meter>(self.state_confidence[(1, 1)]),
            ),
            Angle::new::<radian>(self.state_confidence[(2, 2)]),
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
