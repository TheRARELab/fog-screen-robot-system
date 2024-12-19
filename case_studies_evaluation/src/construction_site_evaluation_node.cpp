#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <ros/package.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_rescue_evaluation_node");
  ros::NodeHandle nh;

  ros::Publisher image_path_pub = nh.advertise<std_msgs::String>("/image_streamer/image_path", 1);

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
  ROS_INFO("Waiting for the move_base action server...");
  ac.waitForServer();

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 20.648;
  goal.target_pose.pose.position.y = -4.783;
  goal.target_pose.pose.orientation.z = 0.964;
  goal.target_pose.pose.orientation.w = -0.265;

  ac.sendGoal(goal);
  ROS_INFO("Moving to the first goal...");
  ac.waitForResult();

  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Failed to move to the first goal");
    exit(1);
  }
  ROS_INFO("Moved to the first goal");

  // Move to the second goal pose
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 20.10;
  goal.target_pose.pose.position.y = -7.766;
  goal.target_pose.pose.orientation.z = 0.618;
  goal.target_pose.pose.orientation.w = 0.786;

  ac.sendGoal(goal);
  ROS_INFO("Moving to the second goal...");
  ac.waitForResult();

  if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Failed to move to the second goal");
    exit(1);
  }

  // Stream lighting icon image
  std_msgs::String image_path_msg;
  image_path_msg.data = ros::package::getPath("case_studies_evaluation") + "/image/wall_plug.jpg";
  while (image_path_pub.getNumSubscribers() == 0) {
    ros::WallDuration(0.01).sleep();
    ROS_INFO_ONCE("No subscriber connections established for /image_streamer/image_path. Keep trying every 10ms...");
  }
  image_path_pub.publish(image_path_msg);
  ROS_INFO_STREAM("Published image path " << image_path_msg.data << " to /image_streamer/image_path");

  // Turn on the fog machine
  ros::ServiceClient client_on = nh.serviceClient<std_srvs::Empty>("/fog_machine/turn_on");
  std_srvs::Empty srv;
  if ( ! client_on.call(srv)) {
    ROS_ERROR("Failed to turn on fog machine");
    exit(1);
  }
  ROS_INFO("Fog machine turned on!");



  // Turn off fog machine after 30 seconds
  ros::Duration(30).sleep();
  ros::ServiceClient client_off = nh.serviceClient<std_srvs::Empty>("/fog_machine/turn_off");
  if (client_off.call(srv)) {
    ROS_INFO("Fog machine turned off after 30 seconds.");
  } else {
    ROS_ERROR("Failed to turn off fog machine.");
  }

  return 0;
}