#include <ros/ros.h>
#include "ImageConverter.h"
#include <experimental/filesystem>
#include <sys/stat.h>
#include <stdbool.h>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spinOnce();
  ros::shutdown();
  return 0;
}
