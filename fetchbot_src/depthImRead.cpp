#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <fstream>
#include "robotHead.h"

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/head_camera/depth_registered/image_raw", 1,
      &ImageConverter::imageCb, this);
  }

  ~ImageConverter()
  {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    static int image_count = 0;
    std::stringstream sstream;
    sstream << "../data/depth_image" << image_count << ".png" ;
    cv::imwrite( sstream.str(),  cv_ptr->image );
    image_count++;
  }
};

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
  ros::init(argc, argv, "robot_driver");
  RobotHead head;
  head.lookAt("base_link", 1, 0, 0.4); //frame and (x,y,z)
  ros::Duration(1);

  ///////////////////
  // CAPTURE IMAGE //
  ///////////////////
  ros::init(argc, argv, "depth_image_converter");
  captureImage("../data/depth_image0.png");
  cv::Mat image = cv::imread("../data/depth_image0.png");
  imshow("depth_image", image);
  cv::waitKey(0);
  return 0;
} 
