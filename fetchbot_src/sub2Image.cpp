#include <ros/ros.h>
#include "ImageConverter.h"
#include <fstream>

int main(int argc, char** argv)
{
  std::ifstream ifile;
  ifile.open("../data/my_image0.png");
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  while(!ifile){
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
} 
