pub mod config {
    /// Wheel-Wheel width of robot.
    pub const WHEELBASE_WIDTH_INCHES: f64 = 10.0;
    /// Wheel-Wheel length of robot.
    pub const WHEELBASE_LENGTH_INCHES: f64 = 10.0;

    pub const FIELD_ORIENTED: bool = true;
    pub const HALF_FIELD_WIDTH_METERS: f64 = 17.55 / 2.;
    pub const HALF_FIELD_LENGTH_METERS: f64 = 8.05 / 2.;
}

pub mod robotmap {
    pub mod drivetrain_map {
        pub const GYRO_ID: i32 = 0;
        pub const DRIVETRAIN_CANBUS: Option<String> = None;

        pub const FL_ENCODER_ID: i32 = 0;
        pub const FL_DRIVE_ID: i32 = 0;
        pub const FL_TURN_ID: i32 = 0;

        pub const BL_ENCODER_ID: i32 = 0;
        pub const BL_DRIVE_ID: i32 = 0;
        pub const BL_TURN_ID: i32 = 0;

        pub const BR_ENCODER_ID: i32 = 0;
        pub const BR_DRIVE_ID: i32 = 0;
        pub const BR_TURN_ID: i32 = 0;

        pub const FR_ENCODER_ID: i32 = 0;
        pub const FR_DRIVE_ID: i32 = 0;
        pub const FR_TURN_ID: i32 = 0;
    }
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
    pub const SWERVE_TURN_RATIO: f64 = 1.0 / 12.8;
    pub const SWERVE_DRIVE_RATIO: f64 = 1.0 / 6.12;
    pub const SWERVE_WHEEL_DIAMETER_INCHES: f64 = 4.0;
    pub const ARC_ODOMETRY_MINIMUM_DELTA_ANGLE_RADIANS: f64 = 0.00001;
    pub const ARC_ODOMETRY_FOM_DAMPENING: f64 = 0.02; // How much to distrust higher values for arc radius.
    pub const DRIVETRAIN_ERROR_THRESHOLD: f64 = 0.5;
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
