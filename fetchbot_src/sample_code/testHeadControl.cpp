#include <ros/ros.h>
#include "robotHead.h"

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  //initialise robot head class
  RobotHead head;
  // look at (x,y,z) position in a given frame
  head.lookAt("base_link", 1, 0, 0.4);
}
