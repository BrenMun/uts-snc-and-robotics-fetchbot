#include <ros/ros.h>
#include "ImageConverter.h"
#include "robotHead.h"
#include <fstream>
#include <string>

void captureImage(std::string location){
  //subscribe to head camera topic and save an image in data folder
  ImageConverter ic;
  //trys to open image and returns true if the file exists
  std::ifstream ifile;
  ifile.open(location);
  //while the image isn't saved
  while(!ifile){
    //spin once
    ros::spinOnce();
    //try to open image
    ifile.open(location);
  }
  ros::shutdown();
  cv::Mat image = cv::imread(location);
  cv::imshow("image", image);
}

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  //initialise robot head class
  RobotHead head;
  // look at (x,y,z) position in a given frame
  head.lookAt("base_link", 1, 0, 0.4);
  ros::Duration(1);
  //initialise ros
  ros::init(argc, argv, "image_converter");
  //capture image
  captureImage("../data/my_image0.png");
  return 0;
} 
