#include <ros/ros.h>
#include "ImageConverter.h"
#include <sys/stat.h>
#include <stdbool.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
    while(ros::ok()){
    ImageConverter ic;
    ros::spinOnce();
  }
  return 0;
} 
