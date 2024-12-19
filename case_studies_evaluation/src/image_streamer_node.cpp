#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

std::string last_image_path_read;

cv::Mat image;
sensor_msgs::ImagePtr image_msg;

void read_image(const std::string& image_path) {
  ROS_INFO_STREAM("Reading image from " << image_path);
  if (image_path == last_image_path_read) {
    return;
  }

  image = cv::imread(image_path, CV_LOAD_IMAGE_COLOR);
  last_image_path_read = image_path;

  if (!image.empty()) {
    image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  }
  else {
    ROS_ERROR_STREAM("Failed to load the image from: " << image_path);
  }
}

void image_path_callback(const std_msgs::String::ConstPtr& image_path_msg) {
  ROS_INFO_STREAM("Image path received: " << image_path_msg->data);
  read_image(image_path_msg->data);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_streamer_node");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher projector_image_pub = it.advertise("/projector/image", 1);

  ros::Subscriber image_path_sub = nh.subscribe<std_msgs::String>("/image_streamer/image_path", 1, image_path_callback);

  if (argc > 1) {
    read_image(argv[1]);
  } else {
    ROS_FATAL("Please provide an initial image path to stream it.");
    exit(1);
  }

  ros::Rate r(30);
  while (ros::ok()) {
    projector_image_pub.publish(image_msg);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}