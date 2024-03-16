use anyhow::Result;
use dotenv::dotenv;
use std::env;
use video_rs::{self, Decoder, Locator};

use r2r;
use r2r::sensor_msgs::msg::Image;
use r2r::QosProfile;

#[tokio::main]
async fn main() -> Result<()> {
    // Load environment variables from .env file
    dotenv().ok();

    // Create a new r2r context and ros2 node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "go2_video", "")?;
    let mut clock = r2r::Clock::create(r2r::ClockType::RosTime)?;
    let nl = node.logger();
    r2r::log_info!(nl, "Starting {}", node.name()?);

    // Initialize the video_rs library
    video_rs::init().unwrap();

    // load parameters
    let robot_ip_env = env::var("GO2_IP").unwrap_or_else(|_| "".to_string());
    let robot_token_default = env::var("GO2_TOKEN").unwrap_or_else(|_| "".to_string());

    // Create a publisher for the image topic
    let publisher = node.create_publisher::<Image>("/image_raw", QosProfile::default())?;

    let source = Locator::Url(
        "http://commondatastorage.googleapis.com/gtv-videos-bucket/sample/BigBuckBunny.mp4"
            .parse()
            .unwrap(),
    );

    let mut decoder = Decoder::new(&source).expect("failed to create decoder");

    // r2r::log_info!(nl, "Decoding video");

    println!("Input stream size: {:?}", decoder.size());
    println!("Output stream size {:?}", decoder.size_out());

    let (width, height) = decoder.size_out();

    let mut image_msg = Image::default();

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
