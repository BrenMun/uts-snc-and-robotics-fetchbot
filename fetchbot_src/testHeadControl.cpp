#include <ros/ros.h>
#include "robotHead.h"

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");

  RobotHead head;
  head.shakeHead(3);
}