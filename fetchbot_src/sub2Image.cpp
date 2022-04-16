#include <ros/ros.h>
#include "ImageConverter.h"
#include <experimental/filesystem>
#include <sys/stat.h>
#include <stdbool.h>
#include <string>

bool file_exists (std::string filename) {
  std::string s(filename);
  char p[s.length()]; 
  int i;
  for (i = 0; i < sizeof(p); i++) p[i] = s[i];
  struct stat   buffer;   
  return (stat (p, &buffer) == 0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  if (file_exists("../image0.png")) ros::shutdown();
  ros::spin();
  return 0;
}