#include <ros/ros.h>
#include "ImageConverter.h"
#include <fstream>

int main(int argc, char** argv)
{
  //initialise ros
  ros::init(argc, argv, "image_converter");
  //subscribe to head camera topic and save an image in data folder
  ImageConverter ic;
  
  //trys to open image and returns true if the file exists
  std::ifstream ifile;
  ifile.open("../data/my_image0.png");
  
  //while the image isn't saved
  while(!ifile){
    //spin once
    ros::spinOnce();
    //try to open image
    ifile.open("../data/my_image0.png");
  }
  ros::shutdown();
  return 0;
} 
