#include <ros/ros.h>
#include "ImageConverter.h"
#include "../robotHead.h"
#include <fstream>
#include <string>
#include "../hsvTrackbar.h"

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
  // ros::init(argc, argv, "robot_driver");
  // RobotHead head;
  // head.lookAt("base_link", 1, 0, 0.4); //frame and (x,y,z)
  // ros::Duration(1);

  // ///////////////////
  // // CAPTURE IMAGE //
  // ///////////////////
  // ros::init(argc, argv, "image_converter");
  // captureImage("../data/my_image0.png");
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
  
  //create isolated HSV image
  cv::Mat isolated; bitwise_and(hsv, hsv, isolated, threshold);
  
  //find contours of the isolated image
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat contourOutput = threshold.clone();
  cv::findContours(contourOutput, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  //Draw the contours
  cv::Mat contourImage(image.size(), CV_8UC3, cv::Scalar(0,0,0));
  cv::Scalar colors[3] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255)};
  for (size_t idx = 0; idx < contours.size(); idx++) 
    cv::drawContours(image, contours, idx, colors[idx % 3]);

  //find moments in the contour image
  cv::Moments m = cv::moments(threshold, true);

  //find centroid of the contour
  cv::Point p(m.m10/m.m00, m.m01/m.m00);
  
  // coordinates of centroid
  std::cout<< "Point: " << p.x << ", " << p.y << std::endl;

  // show the image with a point mark at the centroid
  circle(image, p, 5, cv::Scalar(0,0,255), -1);
  cv::imshow("Input Image", image);
  cv::waitKey(0);
  return 0;
} 