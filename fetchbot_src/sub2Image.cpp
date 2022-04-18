#include <ros/ros.h>
#include "ImageConverter.h"
#include "robotHead.h"
#include <fstream>
#include <string>

cv::Mat captureImage(std::string location){
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
  return image;
}

void on_trackbar( int, void* ) {}
std::string trackbarWindowName = "Trackbar";
int hsvMinMax[6] = {0,256,0,256,0,256};

void createTrackbars(){
	//create window for trackbars
  cv::namedWindow(trackbarWindowName,0);

  //HSV value names shown on trackbar
  std::string hsv_string[6] = {"H_MIN", "H_MAX", "S_MIN", "S_MAX", "V_MIN", "V_MAX"};

  //create memory to store trackbar name on window
  for (int i=0; i<6; i++){
      std::stringstream trackbarName;
      trackbarName << hsv_string[i] << hsvMinMax[i];
  }
  //trackbar for min hsv values
  for (int i=0; i<5; i+=2)
    cv::createTrackbar(hsv_string[i], trackbarWindowName, &hsvMinMax[i], hsvMinMax[i+1], on_trackbar);
  //trackbar for max hsv values
  for (int i=1; i<6; i+=2)
    cv::createTrackbar(hsv_string[i], trackbarWindowName, &hsvMinMax[i], hsvMinMax[i], on_trackbar);
}

int main(int argc, char** argv)
{
  ///////////////////
  // LOOK AT TABLE //
  ///////////////////
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  //initialise robot head class
  RobotHead head;
  // look at (x,y,z) position in a given frame
  head.lookAt("base_link", 1, 0, 0.4);
  ros::Duration(1);

  ///////////////////
  // CAPTURE IMAGE //
  ///////////////////
  //initialise ros
  ros::init(argc, argv, "image_converter");
  //capture image
  cv::Mat image = captureImage("../data/my_image0.png");

  ////////////////////////
  // CONVERT BGR TO HSV //
  ////////////////////////
  cv::Mat hsv;
  cv::cvtColor(image, hsv, CV_BGR2HSV);
  cv::imwrite("../data/hsv_image.png", hsv);

  //////////////////////
  // MANUAL FILTERING //
  //////////////////////
  cv::Mat threshold; 
  createTrackbars();

  while(true){
    //filter HSV image between values and store filtered image to threshold matrix
		inRange(
      hsv,
      cv::Scalar(hsvMinMax[0],hsvMinMax[2],hsvMinMax[4]), //hsv min
      cv::Scalar(hsvMinMax[1],hsvMinMax[3],hsvMinMax[5]), //hsv max
      threshold
    );
    imshow("threshold image",threshold);
    if (cv::waitKey(30) >= 0) break;
  }
  return 0;
} 
