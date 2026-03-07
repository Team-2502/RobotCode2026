pub mod config {
    use std::f64::consts::PI;

    use nalgebra::Vector2;

    /// Wheel-Wheel width of robot.
    pub const WHEELBASE_WIDTH_METERS: f64 = 0.5715;
    /// Wheel-Wheel length of robot.
    pub const WHEELBASE_LENGTH_METERS: f64 = 0.4953;

    pub const FIELD_ORIENTED: bool = true;
    pub const MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND: f64 = 16.3;
    pub const MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND: f64 = 6.0;
    pub const MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND: f64 = 2.0 * PI;

    pub const HALF_FIELD_WIDTH_METERS: f64 = 17.55 / 2.;
    pub const HALF_FIELD_LENGTH_METERS: f64 = 8.05 / 2.;

    pub const HUB_RED: Vector2<f64> = Vector2::new(11.85, 4.02);
    pub const HUB_BLUE: Vector2<f64> = Vector2::new(4.0625, 4.02);
    pub const PASS_TOP_OFFSET: Vector2<f64> = Vector2::new(0.0, 2.01);
    pub const PASS_BOTTOM_OFFSET: Vector2<f64> = Vector2::new(0.0, -2.01);

    pub const MAX_ITER: i32 = 12;
}

pub mod robotmap {
    pub mod drivetrain_map {
        pub const GYRO_ID: i32 = 23;
        pub const DRIVETRAIN_CANBUS: Option<String> = None;

        pub const FL_ENCODER_ID: i32 = 1;
        pub const FL_DRIVE_ID: i32 = 2;
        pub const FL_TURN_ID: i32 = 3;

        pub const BL_ENCODER_ID: i32 = 4;
        pub const BL_DRIVE_ID: i32 = 5;
        pub const BL_TURN_ID: i32 = 6;

        pub const BR_ENCODER_ID: i32 = 7;
        pub const BR_DRIVE_ID: i32 = 8;
        pub const BR_TURN_ID: i32 = 9;

        pub const FR_ENCODER_ID: i32 = 10;
        pub const FR_DRIVE_ID: i32 = 11;
        pub const FR_TURN_ID: i32 = 12;
    }

    pub mod shooter {
        pub const SHOOTER_MOTOR_LEFT_ID: i32 = 13;
        pub const SHOOTER_MOTOR_RIGHT_ID: i32 = 14;
        pub const HOOD_MOTOR_ID: i32 = 15;

        pub const HOOD_MAX: f64 = 2.4;
        pub const SHOOTER_SPEED: f64 = 0.0;
    }

    pub mod turret {
        pub const SPIN_MOTOR_ID: i32 = 16;
    }

    pub mod intake {
        pub const INTAKE_TOP_MOTOR_ID: i32 = 17;
        pub const INTAKE_BOTTOM_MOTOR_ID: i32 = 18;
        pub const PIVOT_TOP_MOTOR_ID: i32 = 19;
        pub const PIVOT_BOTTOM_MOTOR_ID: i32 = 20;
        pub const INDEXER_MOTOR_ID: i32 = 21;
        pub const HANDOFF_MOTOR_ID: i32 = 22;

        pub const INTAKE_DOWN_POSITION: f64 = 0.0;
        pub const INTAKE_UP_POSITION: f64 = 0.0;

        pub const INTAKE_IN_SPEED: f64 = 0.5;
        pub const INTAKE_REVSERSE_SPEED: f64 = -0.5;
        pub const HANDOFF_SPEED: f64 = 1.0;
    }
}

pub mod shooter {
    pub const GEAR_RATIO_HOOD: f64 = 0.0;
    pub const MAX_FLYWHEEL_SPEED: f64 = 100.0;
    pub const SHOOTER_DISTANCE_ERROR_SMUDGE: f64 = 0.88;
}

pub mod turret {
    use nalgebra::Vector2;

    // 29.5
    // 34.5
    pub const GEAR_RATIO: f64 = 34.5;
    pub const TURRET_MAX: f64 = 180.0;
    pub const TURRET_MIN: f64 = -180.0;
    pub const TURRET_CLAMP: f64 = 4.0;

    // meters
    pub const OFFSET: Vector2<f64> = Vector2::new(0.0, 0.0);
    pub const TOLERANCE: f64 = 0.001;
}

pub mod vision {
    use nalgebra::Vector2;

    /// pitch of the limelight in degrees
    pub const LIMELIGHT_PITCH_DEGREES: f64 = 0.;
    /// the yaw of the limelight in degrees (counterclockwise positive)
    pub const LIMELIGHT_YAW_DEGREES: f64 = 90.;
    /// limelights height off the ground in inches
    pub const LIMELIGHT_HEIGHT_INCHES: f64 = 20.92;

    /// Distance from center of robot to limelight in inches as vector2 (x,y)
    pub const ROBOT_CENTER_TO_LIMELIGHT_INCHES: Vector2<f64> = Vector2::new(11.118, 10.352);

    /// # This compensates for underestimation caused by angled views of the target.
    /// Calculated with the formula:
    /// ```text
    /// TX_FUDGE_FACTOR = (real_distance - estimated_distance) / (estimated_distance * |tx|)
    /// ```
    /// Increase distance by 13.5% for every 20 degrees of absolute value of tx
    ///
    /// Set this to 0 for new robots
    pub const TX_FUDGE_FACTOR: f64 = 0.;

    /// the starting fom for the limelight in meters
    pub const LIMELIGHT_BASE_FOM: f64 = 0.001;
    /// meters of inaccuracy per degree of tx absolute value (if limelight is miscalibrated)
    pub const LIMELIGHT_INACCURACY_PER_DEGREE_TX: f64 = 0.015;
    /// meters of inaccuracy per (radian/second) of drivetrain angular velocity
    pub const LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY: f64 = 2.;
    ///  meters of inaccuracy per (meter/second) of drivetrain linear velocity
    pub const LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY: f64 = 2.;
}

pub mod drivetrain {
    pub const SWERVE_TURN_RATIO: f64 = 12.8;
    pub const SWERVE_DRIVE_RATIO: f64 = 6.12;
    pub const SWERVE_WHEEL_CIRCUMFERENCE_METERS: f64 = 0.364;

    pub const WHEEL_ENCODER_STD_DEV: f64 = 0.0005;

    pub const GYRO_OFFSET_UPDATE_RATIO: f64 = 0.25;

    pub const FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.418212890625;
    pub const BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.575927734375;
    pub const BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.811767578125;
    pub const FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.90576171875;
}

pub mod auto {
    pub const SWERVE_TURN_KP: f64 = 0.6;

    pub const SWERVE_DRIVE_KP: f64 = 0.7;
    pub const SWERVE_DRIVE_KI: f64 = 2.;
    pub const SWERVE_DRIVE_KD: f64 = 50.;
    pub const SWERVE_DRIVE_KF: f64 = 0.; // Velocity ff
    pub const SWERVE_DRIVE_KFA: f64 = 0.; // Acceleration ff

    pub const SWERVE_DRIVE_MAX_ERR: f64 = 0.15;
    pub const SWERVE_DRIVE_SUGGESTION_ERR: f64 = 0.35;
    pub const SWERVE_DRIVE_IE: f64 = 0.175; //0.175; // integral enable
}

pub mod joystick_map {
    // Joystick IDs (set in driver station)
    pub const RIGHT_DRIVE: i32 = 0;
    pub const LEFT_DRIVE: i32 = 1;
    pub const OPERATOR: i32 = 2;
}
