#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/stat.h>
#include <stdbool.h>
#include <sstream>

bool file_exists (std::string filename) {
  std::string s(filename);
  char p[s.length()]; 
  int i;
  for (i = 0; i < sizeof(p); i++) p[i] = s[i];
  struct stat   buffer;   
  return (stat (p, &buffer) == 0);
}

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
    static int image_count = 0;
    std::stringstream sstream;
    sstream << "../data/my_image" << image_count << ".png" ;
    cv::imwrite( sstream.str(),  cv_ptr->image );
    image_count++;
    if (file_exists("../data/my_image0.png")) ros::shutdown();
  }
};
