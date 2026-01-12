use crate::constants::vision;
use frcrs::limelight::{Limelight, LimelightResults};
use nalgebra::{Quaternion, Rotation2, Vector2, Vector3};
use serde_json::Value;
use std::fs::File;
use std::net::SocketAddr;
use uom::num::FromPrimitive;
use uom::si::length::inch;
use uom::si::{
    angle::{degree, radian},
    f64::{Angle, Length},
    length::meter,
};

use crate::constants::vision::{
    LIMELIGHT_BASE_FOM, LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY,
    LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY,
};
//not implemented yet
//use crate::swerve::Odometry::PoseEstimate;
use crate::constants::drivetrain::ARC_ODOMETRY_FOM_DAMPENING;
use tokio::time::Instant;

#[derive(Clone)]
/// The vision struct containing
/// - tag_map_values
/// - the limelight
/// - results and last results
/// - saved tag id
/// - drivetrain angle/robot position and last drivetrain angle/robot position
/// - last update time
pub struct Vision {
    tag_map_values: Value,
    limelight: Limelight,
    results: LimelightResults,
    last_results: LimelightResults,
    saved_id: i32,
    drivetrain_angle: Angle,
    last_drivetrain_angle: Angle,
    last_update_time: Instant,
    robot_position: Vector2<Length>,
    last_robot_position: Vector2<Length>,
}

/// the field position
/// - coordinates vec3
/// - quarternion f64
pub struct FieldPosition {
    pub coordinate: Option<Vector3<Length>>,
    pub quaternion: Option<Quaternion<f64>>,
}

impl Vision {
    pub fn new(addr: SocketAddr) -> Self {
        let tagmap = File::open("/home/lvuser/tagmap.json").unwrap();
        let tag_values: Value = serde_json::from_reader(tagmap).unwrap();
        let limelight = Limelight::new(addr);

        Self {
            tag_map_values: tag_values,
            limelight,
            results: LimelightResults::default(),
            last_results: LimelightResults::default(),
            saved_id: 0,
            drivetrain_angle: Angle::new::<degree>(0.),
            last_drivetrain_angle: Angle::new::<degree>(0.),
            last_update_time: Instant::now(),
            robot_position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
            last_robot_position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
        }
    }
    /// Updates the results from the limelight
    ///
    /// returns nothing, but updates vision struct values
    pub async fn update(&mut self, drivetrain_angle: Angle, robot_position: Vector2<Length>) {
        self.last_results = self.results.clone();
        self.last_robot_position = self.robot_position;
        self.robot_position = robot_position;

        let results = self.limelight.results().await;
        if let Ok(r) = results {
            self.results = r;
        } else {
            eprintln!("failed to fetch results from limelight")
        }

        self.last_drivetrain_angle = self.drivetrain_angle;
        self.drivetrain_angle = drivetrain_angle;
        self.last_update_time = Instant::now();

        if !self.results.Fiducial.is_empty()
            && self.results.Fiducial[0].fID != -1
            && self.results.Fiducial[0].fID != self.saved_id
        {
            self.saved_id = self.results.Fiducial[0].fID;
        }
    }

    /// gets the targeted tag's angle from the limelight's equator as of the last update
    ///
    /// will return 0 if no tag is targeted
    pub fn get_ty(&self) -> Angle {
        Angle::new::<degree>(self.results.ty)
    }

    /// gets the targeted tag's angle from the limelight's vertical centerline as of the last update
    ///
    /// will return 0 if no tag is targeted
    pub fn get_tx(&self) -> Angle {
        Angle::new::<degree>(self.results.tx)
    }

    /// gets the id of the targeted tag as of the last update
    ///
    /// will return -1 if no tag is seen
    pub fn get_id(&self) -> i32 {
        if !self.results.Fiducial.is_empty() {
            self.results.Fiducial[0].fID
        } else {
            -1
        }
    }

    /// gets you the last seen tag
    ///
    /// will be set to 0 if no tag has been seen
    pub fn get_last_results(&self) -> LimelightResults {
        self.last_results.clone()
    }

    /// Returns the horizontal distance from the targeted tag as of the last update to the limelight lens
    ///
    /// Returns Option::None if no tag is being targeted
    /// Returns a length
    ///
    /// based on 2D targeting math (known height difference, angle) described here:
    /// https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory
    pub fn get_dist(&self) -> Option<Length> {
        match self.get_tag_position(self.get_id()) {
            Some(..) => {
                let tag_height = self
                    .get_tag_position(self.get_id())?
                    .coordinate
                    .unwrap()
                    .z
                    .get::<inch>();
                let height_diff = tag_height - vision::LIMELIGHT_HEIGHT_INCHES;
                let pitch_to_tag: Angle = Angle::new::<degree>(
                    vision::LIMELIGHT_PITCH_DEGREES + self.get_ty().get::<degree>(),
                );
                let mut dist =
                    Length::new::<inch>(height_diff) / f64::tan(pitch_to_tag.get::<radian>());
                dist += dist * vision::TX_FUDGE_FACTOR * self.get_tx().get::<degree>().abs();
                Some(dist)
            }
            None => None,
        }
    }

    /// returns a tags position on the field using tagmap
    pub fn get_tag_position(&self, id: i32) -> Option<FieldPosition> {
        let id_json = usize::from_i32(id - 1);
        match id_json {
            Some(_usize) => {
                let coords = Vector3::new(
                    Length::new::<meter>(
                        self.tag_map_values["tags"][id_json?]["pose"]["translation"]["x"]
                            .as_f64()?,
                    ),
                    Length::new::<meter>(
                        self.tag_map_values["tags"][id_json?]["pose"]["translation"]["y"]
                            .as_f64()?,
                    ),
                    Length::new::<meter>(
                        self.tag_map_values["tags"][id_json?]["pose"]["translation"]["z"]
                            .as_f64()?,
                    ),
                );

                let quaternion = Quaternion::new(
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["W"]
                        .as_f64()
                        .unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["X"]
                        .as_f64()
                        .unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["Y"]
                        .as_f64()
                        .unwrap(),
                    self.tag_map_values["tags"][id_json?]["pose"]["rotation"]["quaternion"]["Z"]
                        .as_f64()
                        .unwrap(),
                );

                Some(FieldPosition {
                    coordinate: Some(coords),
                    quaternion: Some(quaternion),
                })
            }
            None => None,
        }
    }

    /// estimates robot position (always blue origin) given a drivetrain angle (CCW+) and last updates' limelight measurements
    /// uses 2D calculations: distance from tag center & angle to tag center
    ///
    /// returns Option::None if no tag is currently targeted
    pub fn get_position_from_tag_2d(&self) -> Option<Vector2<Length>> {
        let id = self.get_id();
        let dist = self.get_dist()?;

        let drivetrain_angle = self.drivetrain_angle;

        let tag_data = self.get_tag_position(id)?;
        let tag_xy = Vector2::new(tag_data.coordinate?.x, tag_data.coordinate?.y);

        let angle_to_tag: Angle = (drivetrain_angle)
            + Angle::new::<degree>(vision::LIMELIGHT_YAW_DEGREES)
            - self.get_tx();

        let limelight_to_tag: Vector2<Length> = Vector2::new(
            dist * f64::cos(angle_to_tag.get::<radian>()),
            dist * f64::sin(angle_to_tag.get::<radian>()),
        );

        let robot_center_to_limelight_unrotated_inches: Vector2<f64> = Vector2::new(
            vision::ROBOT_CENTER_TO_LIMELIGHT_INCHES.x,
            vision::ROBOT_CENTER_TO_LIMELIGHT_INCHES.y,
        );

        let drivetrain_angle_rotation = Rotation2::new(drivetrain_angle.get::<radian>());
        let robot_to_limelight_inches =
            drivetrain_angle_rotation * robot_center_to_limelight_unrotated_inches;
        let robot_to_limelight: Vector2<Length> = Vector2::new(
            Length::new::<inch>(robot_to_limelight_inches.x),
            Length::new::<inch>(robot_to_limelight_inches.y),
        );

        Some(tag_xy - limelight_to_tag - robot_to_limelight)
    }

    /// Returns the botpose: x, y
    pub fn get_botpose_orb(&self) -> Option<Vector2<Length>> {
        let pose: Vector2<Length> = Vector2::new(
            Length::new::<meter>(self.results.botpose_orb_wpiblue[0]),
            Length::new::<meter>(self.results.botpose_orb_wpiblue[1]),
        );
        if pose.x.get::<meter>() == 0. {
            None
        } else {
            Some(pose)
        }
    }

    /// gets the fom of the limelight returns it as a f64 the higher the more confident
    pub fn get_vision_fom(&self) -> f64 {
        let dt = Instant::now() - self.last_update_time;

        // get the angular drift as the drivetrain moves
        let angular_velocity_rad_per_sec = (self.drivetrain_angle.get::<radian>()
            - self.last_drivetrain_angle.get::<radian>())
            / dt.as_secs_f64();

        // get the movement of the robot since the last frame
        let robot_position_meters: Vector2<f64> = Vector2::new(
            self.robot_position.x.get::<meter>(),
            self.robot_position.y.get::<meter>(),
        );
        let last_robot_position_meters: Vector2<f64> = Vector2::new(
            self.last_robot_position.x.get::<meter>(),
            self.last_robot_position.y.get::<meter>(),
        );

        // get the linear velocity of the robot
        let linear_velocity_meters_per_sec =
            (robot_position_meters - last_robot_position_meters).magnitude() / dt.as_secs_f64();

        // total uncertainty the higher the more expected error
        let uncertainty = LIMELIGHT_INACCURACY_PER_ANGULAR_VELOCITY
            * angular_velocity_rad_per_sec.abs()
            + LIMELIGHT_INACCURACY_PER_LINEAR_VELOCITY * linear_velocity_meters_per_sec.abs()
            + LIMELIGHT_BASE_FOM;

        // steal dampening from odo fom and make higher is better to match them up and clamp it to be 0 - 1
        1.0 / (uncertainty + 1.0).clamp(0.0, 1.0)
    }

    /// returns the yaw in radians
    pub fn get_yaw(&self) -> Angle {
        Angle::new::<radian>(self.results.botpose_wpiblue[5])
    }
}
