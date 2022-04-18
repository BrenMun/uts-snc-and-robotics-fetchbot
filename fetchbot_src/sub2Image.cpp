#include <ros/ros.h>
#include "ImageConverter.h"
#include "robotHead.h"
#include <fstream>
#include <string>
#include "hsvTrackbar.h"

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
}

int main(int argc, char** argv)
{
  ///////////////////
  // LOOK AT TABLE //
  ///////////////////
  ros::init(argc, argv, "robot_driver");
  RobotHead head;
  head.lookAt("base_link", 1, 0, 0.4); //frame and (x,y,z)
  ros::Duration(1);

  ///////////////////
  // CAPTURE IMAGE //
  ///////////////////
  ros::init(argc, argv, "image_converter");
  captureImage("../data/my_image0.png");
  cv::Mat image = cv::imread("../data/my_image0.png");

  ////////////////////////
  // CONVERT BGR TO HSV //
  ////////////////////////
  cv::Mat hsv;
  cv::cvtColor(image, hsv, CV_BGR2HSV);
  cv::imwrite("../data/hsv_image.png", hsv);

  ////////////////////
  // ISOLATE OBJECT //
  ////////////////////
  createTrackbars();
  //values for gazebo cube: object = min(91,15,63) and object edges = max(256,245,256)
  cv::Mat threshold = getIsolatedObject(hsv); 
  imshow("Isolated Object", threshold);
  cv::waitKey(0);
  return 0;
} 
