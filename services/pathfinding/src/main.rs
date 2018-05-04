extern crate protobuf;
extern crate Obstacle_Path_Finder;
extern crate hyper;
extern crate futures;
extern crate tokio_core;

mod messages { pub mod interop; pub mod telemetry; }

use std::{env,str};
use std::cell::RefCell;
use std::rc::Rc;

use protobuf::*;
use messages::telemetry::*;
use messages::interop::*;
use Obstacle_Path_Finder::PathFinder;
use Obstacle_Path_Finder::Point;
use Obstacle_Path_Finder::Obstacle;
use Obstacle_Path_Finder::Plane;

use futures::future::Future;
use futures::stream::Stream;
use hyper::client::Client;
use hyper::server::{Http, Request, Response, Service};
use hyper::{Method, StatusCode, Uri};
use tokio_core::reactor::Core;

const PROTOCOL: &str = "http://";
const DEFAULT_TELEMETRY_URL: &str = "0.0.0.0:5000";
const DEFAULT_INTEROP_PROXY_URL: &str = "0.0.0.0:8000";

#[derive(Clone)]
struct Autopilot {
    telemetry_host: String,
    interop_proxy_host: String,
    pathfinder: Option<Rc<RefCell<PathFinder>>>
}

impl Autopilot {
    pub fn new() -> Autopilot {
        let mut autopilot = Autopilot {
            telemetry_host: PROTOCOL.to_string() + &env::var("TELEMETRY_URL").
                unwrap_or(String::from(DEFAULT_TELEMETRY_URL)),
            interop_proxy_host: PROTOCOL.to_string() + &env::var("INTEROP_PROXY_URL").
                unwrap_or(String::from(DEFAULT_INTEROP_PROXY_URL)),
            pathfinder: None
        };

        println!("Getting flyzone...");
        let mission = autopilot.get_mission().unwrap();
        let raw_flyzones = &mission.get_fly_zones();
        let mut flyzones = Vec::new();
        for i in 0..raw_flyzones.len() {
            let boundary = raw_flyzones[i].get_boundary();
            let mut flyzone = Vec::new();
            for j in 0..boundary.len() {
                flyzone.push(Point::from_degrees(boundary[j].lat, boundary[j].lon));
            }
            flyzones.push(flyzone);
        }
        let mut pathfinder = PathFinder::new(1.0, flyzones);

        let obstacle_list = autopilot.get_obstacles().unwrap();
        pathfinder.set_obstacle_list(obstacle_list);

        autopilot.pathfinder = Some(Rc::new(RefCell::new(pathfinder)));

        println!("Initialization complete.");
        autopilot
    }

    fn get_object<T: protobuf::MessageStatic>(&self, uri: Uri) -> Result<T, hyper::Error> {
        let mut core = Core::new()?;
        let client = Client::new(&core.handle());
        let request = client.get(uri).and_then(|res| {
            res.body().concat2()
        });
        let response = core.run(request)?;
        match core::parse_from_bytes::<T>(&response) {
            Ok(res) => Ok(res),
            Err(_e) => Err(hyper::Error::Incomplete)
        }
    }

    fn get_telemetry(&self) -> Result<InteropTelem, hyper::Error> {
        let uri = format!("{}/api/interop-telem", self.telemetry_host).parse()?;
        self.get_object(uri)
    }

    fn get_mission(&self) -> Result<InteropMission, hyper::Error> {
        let uri = format!("{}/api/mission", self.interop_proxy_host).parse()?;
        self.get_object(uri)
    }

    fn get_obstacles(&self) -> Result<Vec<Obstacle>, hyper::Error> {
        let uri = format!("{}/api/obstacles", self.interop_proxy_host).parse()?;
        let obstacles : Obstacles = self.get_object(uri)?;
        let mut obstacle_list = Vec::new();
        for obstacle in obstacles.get_stationary() {
            obstacle_list.push(
                Obstacle{
                    coords: Point::from_degrees(
                        obstacle.get_pos().lat, obstacle.get_pos().lon),
                    radius: obstacle.radius as f32,
                    height: obstacle.height as f32}
            );
        }

        Ok(obstacle_list)
    }
}

impl Service for Autopilot {
    type Request = Request;
    type Response = Response;
    type Error = hyper::Error;
    type Future = Box<Future<Item=Self::Response, Error=Self::Error>>;

    fn call(&self, req: Request) -> Self::Future {
        let mut response = Response::new();
        match (req.method(), req.path()) {
            (&Method::Get, "/api/alive") => {
                response.set_body("Alive and well!");
            }
            (&Method::Post, "/api/update_path") => {
                response.set_status(StatusCode::Ok);
                let telemetry = self.get_telemetry().unwrap();
                let pos = telemetry.get_pos();
                let pathfinder = self.pathfinder.clone().unwrap();
                let path = pathfinder.borrow_mut().adjust_path(
                    Plane::new(pos.lat, pos.lon, pos.alt_msl as f32));

                if let Some(path) = path {
                    println!("A* Result");
                    for node in path {
                        println!("{:.5}, {:.5}", node.location.lat_degree(), node.location.lon_degree());
                    }
                }
            }
            _ => {
                response.set_status(StatusCode::NotFound);
            }
        }
        Box::new(futures::future::ok(response))
    }
}

fn main() {
    println!("Initializing...");
    let autopilot = Autopilot::new();
    let addr = "0.0.0.0:7500".parse().unwrap();
    let server = Http::new().bind(&addr, move || Ok(autopilot.clone())).unwrap();
    server.run().unwrap();
}