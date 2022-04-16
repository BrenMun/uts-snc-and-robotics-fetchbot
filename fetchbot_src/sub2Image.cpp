#include <ros/ros.h>
#include "ImageConverter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spinOnce();
  return 0;
}