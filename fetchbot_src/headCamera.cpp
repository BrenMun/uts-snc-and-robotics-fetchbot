#include "headCamera.h"
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

void callback(const ImageConstPtr& msg_rgb, const PointCloud2ConstPtr& msg_depth){
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

HeadCamera::HeadCamera(){}

void HeadCamera::sub2Cam(ros::NodeHandle *nh){
    //subscribers
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb(*nh, "/head_camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> subscriber_depth(*nh, "/head_camera/depth_registered/points", 1);

    //Synchronisation policy for images
    #ifdef EXACT
        typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    #endif
    #ifdef APPROXIMATE
        typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    #endif

    //Sync policy takes a queue size as  its constructor argument
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth);
    sync.registerCallback(boost::bind(&callback, _1, _2));
}
