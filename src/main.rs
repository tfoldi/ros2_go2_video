use anyhow::Result;
use dotenv::dotenv;
use std::collections::HashMap;
use std::env;
use tokio::task;
use tokio::time;
use video_rs::{self, Decoder, Locator};

use r2r;
use r2r::sensor_msgs::msg::Image;
use r2r::QosProfile;

fn get_decoder_options() -> video_rs::Options {
    let mut options: HashMap<String, String> = HashMap::new();
    options.insert(
        "protocol_whitelist".to_string(),
        "file,rtp,udp,https,tls,tcp".to_string(),
    );
    options.insert("flags".to_string(), "low_delay".to_string());
    options.insert("analyzeduration".to_string(), "4M".to_string());
    options.insert("probesize".to_string(), "4M".to_string());
    options.insert("vf".to_string(), "setpts=0".to_string());

    return video_rs::Options::from(options);
}

#[tokio::main]
async fn main() -> Result<()> {
    // Load environment variables from .env file
    dotenv().ok();
    // Initialize the logger for video_rs
    tracing_subscriber::fmt::init();

    // Create a new r2r context and ros2 node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "go2_video", "")?;
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;
    let nl = node.logger();
    r2r::log_info!(nl, "Starting {}", node.name()?);

    // Initialize the video_rs library
    video_rs::init().unwrap();

    // load parameters
    let robot_ip = env::var("GO2_IP").unwrap_or_else(|_| "".to_string());
    let robot_token = env::var("GO2_TOKEN").unwrap_or_else(|_| "".to_string());
    let video_port: u16 = 4002;
    let audio_port: u16 = 4000;
    let debug_webrtc: bool = true;

    r2r::log_info!(nl, "Connecting to the robot's video stream");
    let _webrtc_handle = task::spawn(async move {
        go2webrtc_rs::run(
            video_port,
            audio_port,
            robot_ip.as_str(),
            robot_token.as_str(),
            debug_webrtc,
        )
        .await
    });

    // TODO: wait for the webrtc connection to be established
    time::sleep(std::time::Duration::from_secs(3)).await;

    //let source = Locator::Url("file:///tmp/connection.sdp".parse().unwrap());
    let source = Locator::Url(
        "https://raw.githubusercontent.com/tfoldi/go2webrtc-rs/master/connection.sdp"
            .parse()
            .unwrap(),
    );

    r2r::log_debug!(nl, "Opening local stream");

    let decoder_options = get_decoder_options();
    let mut decoder =
        Decoder::new_with_options(&source, &decoder_options).expect("failed to create decoder");

    r2r::log_info!(nl, "Input stream size: {:?}", decoder.size());
    r2r::log_info!(nl, "Output stream size {:?}", decoder.size_out());

    // Create a publisher for the image topic
    let publisher = node.create_publisher::<Image>("/image_raw", QosProfile::default())?;

    // Create an Image message
    let mut image_msg = Image::default();
    let (width, height) = decoder.size_out();

    image_msg.header.frame_id = "front_camera".to_string();
    image_msg.width = width;
    image_msg.height = height;
    image_msg.step = image_msg.width * 3;
    image_msg.encoding = "rgb8".to_string();
    if cfg!(target_endian = "little") {
        image_msg.is_bigendian = 0;
    } else if cfg!(target_endian = "big") {
        image_msg.is_bigendian = 1;
    }

    // Process frames one by one
    for frame in decoder.decode_iter() {
        if let Ok((_, frame)) = frame {
            let now = clock.get_now()?;

            image_msg.header.stamp = r2r::Clock::to_builtin_time(&now);
            image_msg.data = frame.into_raw_vec();

            let _ = publisher.publish(&image_msg);
        } else {
            break;
        }
    }

    Ok(())
}
