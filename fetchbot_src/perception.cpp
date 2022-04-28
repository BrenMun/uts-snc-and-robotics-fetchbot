#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include "robotHead.h"

//#define EXACT
#define APPROXIMATE
#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif
using namespace message_filters;

void callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::PointCloud2ConstPtr& msg_depth)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image, hsv, mask;
  
  pcl::PointCloud<pcl::PointXYZ> depth;   
  
  try{
    cv_ptr = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    //store image in matrix
    image = cv_ptr->image;

    //convert to hsv
    cv::cvtColor(image, hsv, CV_BGR2HSV);

    //filter out hsv of object
    cv::inRange(hsv, cv::Scalar(28,0,107), cv::Scalar(256,256,256), mask);

    //find moments of the object
    cv::Moments m = cv::moments(mask, true);

    //find centroid of the object
    cv::Point c(m.m10/m.m00, m.m01/m.m00);
    std::cout << "Image Point: (" << c.x << ", " << c.y << "), ";

    //convert msg to xyz point cloud
    pcl::fromROSMsg(*msg_depth, depth);
    pcl::PointXYZ p1 = depth.at(c.x, c.y);
    std::cout << "3D Point: (" << p1.x << ", " << p1.y << ", " << p1.z << ")\n";
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
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
  ros::init(argc, argv, "sub2camera");
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb(nh, "/head_camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_depth(nh, "/head_camera/depth_registered/points", 1);

  //Synchronisation policy for images
  #ifdef EXACT
      typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  #endif
  #ifdef APPROXIMATE
      typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  #endif

  // Sync policy takes a queue size as its constructor argument
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&callback, _1, _2));
  ros::spin();
}