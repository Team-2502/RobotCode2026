use crate::constants::vision;
use frcrs::limelight::{Limelight, LimelightResults};
use nalgebra::{Quaternion, Vector2, Vector3};
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
    pub limelight: Limelight,
    pub results: LimelightResults,
    last_results: LimelightResults,
    // pub status: LimelightStatus,
    saved_id: i32,
    // gyro_offset: Angle,
    // pub gyro_offset_set: bool,
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
        }
    }
    /// Updates the results from the limelight
    ///
    /// returns nothing, but updates vision struct values
    pub async fn update(&mut self) {
        self.last_results = self.results.clone();

        let results = self.limelight.results().await;
        if let Ok(r) = results {
            self.results = r;
            println!("it got results bro");
            println!(
                "vision line 66: ll pose: x: {} y: {}, yaw: {}",
                self.results.botpose_wpiblue[0],
                self.results.botpose_wpiblue[1],
                self.results.botpose_wpiblue[5]
            );
            //println!("{:?}", self.results.Fiducial);
        } else {
            println!("failed to fetch results from limelight");
            let response = self.limelight.response().await;
            println!("status: {:?}", response);
        }

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
        Angle::new::<degree>(self.results.Fiducial[0].ty)
    }

    /// gets the targeted tag's angle from the limelight's vertical centerline as of the last update
    ///
    /// will return 0 if no tag is targeted
    pub fn get_tx(&self) -> Angle {
        Angle::new::<degree>(self.results.Fiducial[0].tx)
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

    /// Returns the botpose: x, y
    pub fn get_botpose(&self) -> Option<Vector2<Length>> {
        let pose: Vector2<Length> = Vector2::new(
            Length::new::<meter>(self.results.botpose_wpiblue[0]),
            Length::new::<meter>(self.results.botpose_wpiblue[1]),
        );
        if pose.x.get::<meter>() == 0. {
            None
        } else {
            Some(pose)
        }
    }

    pub fn get_field_yaw(&self) -> Angle {
        Angle::new::<degree>(self.results.botpose_wpiblue[5])
    }

    pub fn get_location_error(&self) -> Vector2<Length> {
        let tag_area = self.results.Fiducial[0].ta;
        let distance_variation_modifier = 0.00000961227 * tag_area.powf(-1.25093);

        Vector2::new(
            Length::new::<meter>(self.results.stdev_mt1[0] + distance_variation_modifier),
            Length::new::<meter>(self.results.stdev_mt1[1] + distance_variation_modifier),
        )
    }

    pub fn has_tag(&self) -> bool {
        if self.results.botpose_tagcount > 0 && !self.results.Fiducial.is_empty() {
            true
        } else {
            false
        }
    }

    pub fn get_yaw_error(&self) -> Angle {
        let tag_area = self.results.Fiducial[0].ta;
        let yaw_variation_modifier = 0.000204176 * tag_area.powf(-1.37573);

        Angle::new::<degree>(self.results.stdev_mt1[5] + yaw_variation_modifier)
    }

    // {"cameraQuat":{"w":0.6321377276274888,"x":0.774783983401699,"y":-0.004118024448506402,"z":-0.009732124577505519},"cid":9281,"cpu":75.11737060546875,"finalYaw":-1.0707001893496237,"finalimu":[-1.0707001893496237,0.5657632629803511,-11.573459341930416,-1.0707001893496237,-0.38499999046325684,-0.17499999701976776,-0.2800000011920929,-0.20276400446891785,-0.010003999806940556,0.9882000088691711],"fps":60.90412521362305,"hailoCount":1,"hailoPower":3.75,"hailoTemp":74.0,"hwType":6,"ignoreNT":0,"interfaceNeedsRefresh":0,"name":"","pipeImgCount":2,"pipelineIndex":0,"pipelineType":"pipe_fiducial","ram":34.567813873291016,"snapshotMode":0,"temp":71.05000305175781}

    // //radians per second
    // pub fn get_angular_velocity(&self) -> f64 {
    //     self.results.imu.unwrap_or([0.0; 10])[6]
    // }

    pub fn get_limelight_data(&self) {}
}

pub fn distance(p1: Vector2<Length>, p2: Vector2<Length>) -> f64 {
    let dx = p2.x.get::<meter>() - p1.x.get::<meter>();
    let dy = p2.y.get::<meter>() - p1.y.get::<meter>();
    (dx * dx + dy * dy).sqrt()
}

// tagmap test to make sure i converted it all right
// ["tags"][id_json?]["pose"]
// test subsystems::vision::tests::test_tagmap ... ok
//
// gone for now because i changed tagmap will fix later

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use std::fs::File;
//     use std::net::SocketAddr;
//     use std::path::Path;
//     use uom::si::f64::Length;
//     use uom::si::length::meter;

//     fn test_vision() -> Vision {
//         let addr: SocketAddr = "127.0.0.1:5800".parse().unwrap();

//         // make new vision with hardwired tagmap path
//         let mut vision = Vision {
//             tag_map_values: serde_json::json!({"tags": []}),
//             limelight: Limelight::new(addr),
//             results: LimelightResults::default(),
//             last_results: LimelightResults::default(),
//             saved_id: 0,
//             drivetrain_angle: Angle::new::<degree>(0.),
//             last_drivetrain_angle: Angle::new::<degree>(0.),
//             last_update_time: Instant::now(),
//             robot_position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
//             last_robot_position: Vector2::new(Length::new::<meter>(0.), Length::new::<meter>(0.)),
//         };

//         // pass in local tagmap and make sure its actaully there
//         let tagmap_path = "tagmap.json";
//         assert!(
//             Path::new(tagmap_path).exists(),
//             "tagmap.json does not exist"
//         );

//         // open and parse
//         let tagmap_file = File::open(tagmap_path).expect("failed to open tagmap.json");
//         let tag_values: Value =
//             serde_json::from_reader(tagmap_file).expect("failed to parse tagmap.json");

//         // pass in the tagmap values to the fake vision
//         vision.tag_map_values = tag_values;

//         // return the new vision with local tagmap
//         vision
//     }

//     #[test]
//     fn test_tagmap() {
//         // create a test vision using the function above
//         let vision = test_vision();

//         // get the cords and quaternioms and assert equal
//         let tag1 = vision.get_tag_position(1).expect("tag 1 does not exist");
//         let coords1 = tag1.coordinate.unwrap();
//         let quat1 = tag1.quaternion.unwrap();

//         assert!((coords1.x.get::<meter>() - 3.6074798).abs() < 1e-6);
//         assert!((coords1.y.get::<meter>() - 3.3902756).abs() < 1e-6);
//         assert!((coords1.z.get::<meter>() - 0.889).abs() < 1e-6);

//         assert!((quat1.w - 6.123233995736766e-17).abs() < 1e-12);
//         assert!((quat1.i - 0.0).abs() < 1e-12);
//         assert!((quat1.j - 0.0).abs() < 1e-12);
//         assert!((quat1.k - 1.0).abs() < 1e-12);

//         // do the same with 32
//         let tag32 = vision.get_tag_position(32).expect("tag 32 does not exist");
//         let coords32 = tag32.coordinate.unwrap();
//         let quat32 = tag32.quaternion.unwrap();

//         assert!((coords32.x.get::<meter>() + 8.2624228).abs() < 1e-6);
//         assert!((coords32.y.get::<meter>() - 0.1430125999999996).abs() < 1e-6);
//         assert!((coords32.z.get::<meter>() - 0.55245).abs() < 1e-6);

//         assert!((quat32.w - 1.0).abs() < 1e-12);
//         assert!((quat32.i - 0.0).abs() < 1e-12);
//         assert!((quat32.j - 0.0).abs() < 1e-12);
//         assert!((quat32.k - 0.0).abs() < 1e-12);
//     }
// }
