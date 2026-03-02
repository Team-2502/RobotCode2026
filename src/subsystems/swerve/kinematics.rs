use crate::constants::config::{
    MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND, MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND,
    WHEELBASE_LENGTH_METERS, WHEELBASE_WIDTH_METERS,
};
use crate::constants::{
    config::MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND, drivetrain::SWERVE_WHEEL_CIRCUMFERENCE_METERS,
};
use frcrs::networktables::SmartDashboard;
use nalgebra::{Matrix1x3, SMatrix, Vector2, dmatrix, matrix};
use std::f64::consts::PI;
use uom::si::angle::degree;
use uom::si::{angle::radian, f64::Angle, f64::Length, length::meter};

/// Inverse Kinematics: Robot Transform/Rotation -> Wheel State
/// Forward Kinematics: Wheel State -> Robot Transform/Rotation
pub struct Kinematics {
    ik_matrix: SMatrix<f64, 8, 3>,
    fk_matrix: SMatrix<f64, 3, 8>,
}

#[derive(Clone)]
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
        }
    }

    pub fn get_targets(&self, translation: Vector2<f64>, rot: f64) -> Vec<(f64, Angle)> {
        let input_matrix: SMatrix<f64, 3, 1> = matrix![
            translation.x * MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND;
            translation.y * MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND;
            rot * MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND;
        ];

        let setpoint_matrix = self.ik_matrix.clone() * input_matrix;

        let mut setpoints = Vec::new();
        let mut max = 0.0;
        for i in 0..=3 {
            let x = setpoint_matrix[(2 * i)];
            let y = setpoint_matrix[(2 * i + 1)];
            let speed = (x * x + y * y).sqrt() / (SWERVE_WHEEL_CIRCUMFERENCE_METERS);

            if speed > max {
                max = speed;
            }

            let angle = Angle::new::<radian>(f64::atan2(y, x));
            setpoints.push((speed, angle));
        }

        if max > MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND {
            for mut setpoint in setpoints.clone() {
                setpoint.0 *= MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND / max;
            }
        }

        setpoints
    }

    pub fn forward_kinematics(&self, differences: Vec<(f64, Angle)>) -> RobotPoseEstimate {
        let mut setpoints_matrix: SMatrix<f64, 8, 1> = SMatrix::zeros();
        for i in 0..=3 {
            let distance = differences[i].0 * SWERVE_WHEEL_CIRCUMFERENCE_METERS;
            let angle = differences[i].1.get::<radian>();

            setpoints_matrix[(2 * i, 0)] = distance * f64::cos(angle);
            setpoints_matrix[(2 * i + 1, 0)] = distance * f64::sin(angle);
        }

        let translations = self.fk_matrix.clone() * setpoints_matrix;

        println!("{:?}", translations);

        let fom = self.get_targets(
            Vector2::new(translations[(1, 1)], translations[(2, 1)]),
            translations[(3, 1)],
        );

        let mut rmse = 0.0;

        for i in 0..=3 {
            rmse += (fom[i].0 - differences[i].0) * (fom[i].0 - differences[i].0);
            rmse += (fom[i].1.get::<radian>() - differences[i].1.get::<radian>())
                * (fom[i].1.get::<radian>() - differences[i].1.get::<radian>());
        }

        rmse = (rmse / 8.0).sqrt();

        println!("rmse: {}", rmse);

        RobotPoseEstimate {
            fom: (rmse),
            x: (Length::new::<meter>(1.0)),
            y: (Length::new::<meter>(1.0)),
            angle: (Angle::new::<radian>(1.0)),
        }
    }
}

#[cfg(test)]
mod kinematics_tests {
    use uom::si::angle::degree;

    use super::*;
    use crate::subsystems::swerve::kinematics::Kinematics;

    #[test]
    fn throwaway() {
        let kinematics = Kinematics::new();
        let results = kinematics.get_targets(Vector2::new(0.0, 0.0), 1.0);
        for result in results.clone() {
            println!(
                "ik: setpoint f64: {}, angle: {}",
                result.0,
                result.1.get::<degree>()
            )
        }

        kinematics.forward_kinematics(results);
        panic!()
    }
}
