pub mod config {
    use std::f64::consts::PI;

    use nalgebra::Vector2;

    /// Wheel-Wheel width of robot.
    pub const WHEELBASE_WIDTH_METERS: f64 = 0.4953;
    /// Wheel-Wheel length of robot.
    pub const WHEELBASE_LENGTH_METERS: f64 = 0.5715;

    pub const FIELD_ORIENTED: bool = true;
    pub const MAX_DRIVETRAIN_REVOLUTIONS_PER_SECOND: f64 = 16.3;
    pub const MAX_DRIVETRAIN_SPEED_METERS_PER_SECOND: f64 = 6.0; /* 6 b4 */
    pub const MAX_DRIVETRAIN_ROTATION_SPEED_RADIANS_PER_SECOND: f64 = 2.0 * PI;
    pub const MINIMUM_MODULE_VELOCITY_METERS_PER_SECOND: f64 = 0.05;
    pub const MANUAL_TURRET_MODE_DISTANCE_MAX_METERS: f64 = 7.62;
    pub const MANUAL_TURRET_YAW_CHANGE_SCALAR: f64 = 2.0;
    pub const SHOOTER_INITAL_DISTANCE_OFFSET_FEET: f64 = 1.0;

    pub const HALF_FIELD_WIDTH_METERS: f64 = 8.042656 / 2.;
    pub const HALF_FIELD_LENGTH_METERS: f64 = 16.513048 / 2.;
    pub const BLUE_HUB_X_INCHES: f64 = 182.11;
    pub const RED_HUB_X_INCHES: f64 = 469.11;

    pub const HUB_RED: Vector2<f64> = Vector2::new(11.915394, 4.02);
    pub const HUB_BLUE: Vector2<f64> = Vector2::new(4.0625, 4.02);

    pub const RED_PASS_TOP_OFFSET_METERS: Vector2<f64> = Vector2::new(3.0, 2.01);
    pub const RED_PASS_BOTTOM_OFFSET_METERS: Vector2<f64> = Vector2::new(3.0, -2.01);
    pub const BLUE_PASS_TOP_OFFSET_METERS: Vector2<f64> = Vector2::new(-3.0, 2.01);
    pub const BLUE_PASS_BOTTOM_OFFSET_METERS: Vector2<f64> = Vector2::new(-3.0, -2.01);

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
        pub static SHOOTER_CANBUS: &'static str = "can0";
        pub const SHOOTER_MOTOR_LEFT_ID: i32 = 13;
        pub const SHOOTER_MOTOR_RIGHT_ID: i32 = 14;
        pub const HOOD_MOTOR_ID: i32 = 15;

        pub const HOOD_MAX: f64 = 2.4;
        pub const SHOOTER_SPEED: f64 = 0.0;
    }

    pub mod turret {
        pub const SPIN_MOTOR_ID: i32 = 16;
        pub const ENCODER_ID: i32 = 1000;
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

        pub const INTAKE_IN_SPEED: f64 = 0.75;
        pub const INTAKE_REVSERSE_SPEED: f64 = -0.5;
        pub const HANDOFF_SPEED: f64 = 1.0;
    }
}

pub mod shooter {
    pub const GEAR_RATIO_HOOD: f64 = 0.0;
    pub const MAX_FLYWHEEL_SPEED: f64 = 100.0;
    pub const SHOOTER_DISTANCE_ERROR_SMUDGE: f64 = 1.0;
    // pub const SHOOTER_DISTANCE_ERROR_SMUDGE: f64 = 0.88; /* Original */
    // pub const SHOOTER_DISTANCE_ERROR_SMUDGE: f64 = 0.94; /* Northern Lights */
    // pub const SHOOTER_DISTANCE_ERROR_SMUDGE: f64 = 1.125;
}

pub mod turret {
    use std::f64::NAN;
    pub const GEAR_RATIO: f64 = 34.5;
    pub const TURRET_MAX: f64 = 180.0;
    pub const TURRET_MIN: f64 = -180.0;
    pub const TURRET_CLAMP: f64 = 2.5;
    // pub const ORIGIN_TO_TURRET_CENTER_X_INCHES: f64 = -4.0;
    // pub const ORIGIN_TO_TURRET_CENTER_Y_INCHES: f64 = 2.25;
    pub const ORIGIN_TO_TURRET_CENTER_X_INCHES: f64 = -20.0;
    pub const ORIGIN_TO_TURRET_CENTER_Y_INCHES: f64 = 2.25 * 5.0;

    // pub const HOOD_ZERO_POSE: f64 = -0.170177;
    pub const HOOD_ZERO_POSE: f64 = -0.2;
    // 0.2 margin
    pub const HOOD_MAX_SOFTSTOP: f64 = 1.9992675 - 0.2;
    pub const HOOD_MIN_SOFTSTOP: f64 = -0.370361 + 0.2;

    // meters
    pub const TURRET_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = NAN;
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
    use std::f64::consts::PI;

    pub const SWERVE_TURN_RATIO: f64 = 12.8;
    pub const SWERVE_DRIVE_RATIO: f64 = 6.12;
    pub const SWERVE_WHEEL_CIRCUMFERENCE_INCHES: f64 = PI * 4.0;

    pub const WHEEL_ENCODER_STD_DEV: f64 = 0.0005;
    pub const PIGEON_YAW_STD_DEV: f64 = 0.05;

    pub const GYRO_OFFSET_UPDATE_RATIO: f64 = 0.25;

    pub const FL_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.418212890625;
    pub const BL_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.575927734375;
    pub const BR_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.811767578125;
    pub const FR_ABSOLUTE_ENCODER_ZERO_ROTATIONS: f64 = 0.90576171875;
}

pub mod localization {
    // How much to distrust current localization state
    pub const CURRENT_STATE_DRIVE_TRUST: f64 = 0.003;
    pub const CURRENT_STATE_YAW_TRUST: f64 = 0.1;
    pub const CURRENT_STATE_LINEAR_VELOCITY_TRUST: f64 = 500000.0;
    pub const CURRENT_STATE_ANGULAR_VELOCITY_TRUST: f64 = 500000.0;

    pub const VELOCITY_MIN_CONF: f64 = 0.01;
    pub const LINEAR_VEL_CONF_SCALAR: f64 = 1.0;
    pub const ANGULAR_VEL_CONF_SCALAR: f64 = 1.0;

    pub const MAX_LIMELIGHT_POSE_DIFFERENCE_METERS: f64 = 1.0;
    pub const LIMELIGHT_ACCEPTABLE_OUTLIER_COUNT: u32 = 15;
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
