#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sstream>
#include <fstream>

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
    image_sub_ = it_.subscribe("/head_camera/rgb/image_raw", 1,
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
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //file bool
    std::ifstream ifile;
    ifile.open("../data/my_image0.png");

    //While file doesn't exist, save png
    while(!ifile){
      static int image_count = 0;
      std::stringstream sstream;
      sstream << "../data/my_image" << image_count << ".png" ;
      cv::imwrite( sstream.str(),  cv_ptr->image );
      image_count++;
      ifile.open("../data/my_image0.png");
    }
  }
};
