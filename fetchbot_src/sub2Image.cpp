#include <ros/ros.h>
#include "ImageConverter.h"
#include "robotHead.h"
#include <fstream>
#include <string>

cv::Mat captureImage(std::string location){
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
  return image;
}

int main(int argc, char** argv)
{
  ///////////////////
  // LOOK AT TABLE //
  ///////////////////
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  //initialise robot head class
  RobotHead head;
  // look at (x,y,z) position in a given frame
  head.lookAt("base_link", 1, 0, 0.4);
  ros::Duration(1);

  ///////////////////
  // CAPTURE IMAGE //
  ///////////////////
  //initialise ros
  ros::init(argc, argv, "image_converter");
  //capture image
  cv::Mat image = captureImage("../data/my_image0.png");

  ////////////////////////
  // CONVERT BGR TO HSV //
  ////////////////////////
  cv::Mat hsv;
  cv::cvtColor(image, hsv, CV_BGR2HSV);
  cv::imwrite("../data/hsv_image.png", hsv);

  ///////////////////////
  // FILTER OUT OBJECT //
  ///////////////////////
  cv::Mat threshold;
  cv::inRange(
    hsv,
    cv::Scalar(222, 86, 22), //hsv min
    cv::Scalar(240, 100, 50), //hsv max
    threshold
  );
  cv::imwrite("../data/bin_image.png", threshold);
  return 0;
} 
