# search_rescue_evaluation

ROS package for Fetch to go to the end of a hallway (a corner), turn right, rotate towards the corner, and turn on the fog machine (to project a human life icon).

A video demo can be found [here](https://github.com/TheRARELab/fog-screen-device-ros/issues/3#issuecomment-2292415721).

## Requirements

- Ubuntu 18.04 
- ROS Melodic
- A real Fetch robot (not in simulation)
- [fetch_ros](https://github.com/ZebraDevs/fetch_ros/)

## How to run

### Prepare projection

1. Run `roscd search_rescue_evaluation && rosrun search_rescue_evaluation image_streamer_node image/black_screen_image.jpg`
2. Run on the robot: `rosrun image_view image_view image:=/projector/image/` and make it full screen (Windows + F11)

### Run navigation

1. Run `roslaunch search_rescue_evaluation search_rescue_navigation.launch` to start the ROS navigation stack provided by `fetch_navigation`. The launch file uses the NEC north hallway map (See [maps](maps) directory).
2. Drive the robot to the east end of the north hallway and make it face west (see the video demo above).
3. Initialize robot pose: Launch Rviz `roscd fetch_navigation && rviz -d config/navigation.rviz` and use the "2D pose estimate" to set the initial pose of the robot.
4. Run `rosrun search_rescue_evaluation search_rescue_evaluation_node`.

## Resources

- Running Navigation in Gazebo Simulation: https://docs.fetchrobotics.com/navigation.html#running-navigation-in-gazebo-simulation
- Visualizing with Rviz: https://docs.fetchrobotics.com/gazebo.html#visualizing-with-rviz
  - The second code block for ROS navigation visualization, skip the first one
- Sending Waypoints in Rviz: https://docs.fetchrobotics.com/navigation.html#sending-waypoints
- Tutorial on Sending Goals to the Navigation Stack: https://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals
