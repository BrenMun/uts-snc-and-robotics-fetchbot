#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "robotHead.h"

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image, hsv, mask;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      
      //store image in matrix
      image = cv_ptr->image;

      //convert to hsv
      cv::cvtColor(image, hsv, CV_BGR2HSV);

      //filter out hsv of object
      cv::inRange(hsv, cv::Scalar(28,0,107), cv::Scalar(256,256,256), mask);

      //find moments of the object
      cv::Moments m = cv::moments(mask, true);

      //find centroid of the object
      cv::Point p(m.m10/m.m00, m.m01/m.m00);
      std::cout<< "Point: " << p.x << ", " << p.y << std::endl;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
}

int main(int argc, char **argv)
{
  ///////////////////
  // LOOK AT TABLE //
  ///////////////////
  ros::init(argc, argv, "robot_driver");
  RobotHead head;
  head.lookAt("base_link", 1, 0, 0.4); //frame and (x,y,z)
  ros::Duration(1);
  
  ////////////////
  // PERCEPTION //
  ////////////////
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/head_camera/rgb/image_raw", 1, imageCallback);
  ros::spin();
}