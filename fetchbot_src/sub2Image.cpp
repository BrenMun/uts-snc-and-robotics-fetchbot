#include <ros/ros.h>
#include "ImageConverter.h"
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  
  std::ifstream ifile;
  ifile.open("../data/my_image0.png");
  while(!ifile){
    ImageConverter ic;
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
} 
