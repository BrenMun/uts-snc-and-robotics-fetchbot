#include <ros/ros.h>
#include "ImageConverter.h"
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
    while(ros::ok()){
    ImageConverter ic;
    ros::spinOnce();
  }
  return 0;
} 
