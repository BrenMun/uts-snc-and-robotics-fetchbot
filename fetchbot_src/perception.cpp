#include <ros/ros.h>
#include "robotHead.h"
#include "headCamera.h"


int main(int argc, char **argv)
{
  // Look at table
  ros::init(argc, argv, "perception");
  RobotHead head;
  head.lookAt("base_link", 1, 0, 0.4); //frame and (x,y,z)
  ros::Duration(1);
  
  // Subscribe to rgb and depth data
  ros::init(argc, argv, "sub2camera");
  ros::Rate rate(1);
  // while(ros::ok()) {
    // ros::spinOnce();
    HeadCamera camera;
    // rate.sleep();
  // }
  ros::spin();
}