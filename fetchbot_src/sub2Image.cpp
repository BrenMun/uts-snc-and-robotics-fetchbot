#include <ros/ros.h>
#include "ImageConverter.h"
#include <fstream>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  
  std::ifstream ifile;
  ifile.open("../data/my_image0.png");
  
  while(!ifile){
    ros::spinOnce();std::cout << "file not found\n";
  }
  std::cout << "file found\n";
  ros::shutdown();
  return 0;
} 
