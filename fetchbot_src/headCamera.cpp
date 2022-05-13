#include "headCamera.h"
#include <iostream>

HeadCamera::HeadCamera(){
  sub_rgb_.subscribe(nh_, "/head_camera/rgb/image_raw", 1);
  sub_depth_.subscribe(nh_, "/head_camera/depth_registered/points", 1);
  sync_.reset(new Sync(MySyncPolicy(10), sub_rgb_, sub_depth_));
  sync_->registerCallback(boost::bind(&HeadCamera::callback, this, _1, _2));
  pubPoint_ = nh_.advertise<geometry_msgs::PointStamped> ("target_point", 1);
  objHSV_ = {{28,0,107},{256,256,256}};
}

void HeadCamera::callback(const ImageConstPtr& msg_rgb, const PointCloud2ConstPtr& msg_depth){
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image, hsv, mask;
  pcl::PointCloud<pcl::PointXYZ> depth;
  try{
    //convert msg to a CV image
    cv_ptr = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    //store image in matrix
    image = cv_ptr->image;
    //convert to hsv
    cv::cvtColor(image, hsv, CV_BGR2HSV);
    //filter out hsv of object
    cv::inRange(
      hsv, 
      cv::Scalar(objHSV_.min.h, objHSV_.min.s, objHSV_.min.v), //min hsv
      cv::Scalar(objHSV_.max.h, objHSV_.max.s, objHSV_.max.v), //max hsv
      mask
    );
    //find moments of the object
    cv::Moments m = cv::moments(mask, true);
    //find centroid of the object
    cv::Point c(m.m10/m.m00, m.m01/m.m00);
    //get head camera point cloud
    pcl::fromROSMsg(*msg_depth, depth);
    depth.points.resize (depth.width * depth.height);
    //get corresponding 3D point from 2D image position of centroid
    pcl::PointXYZ p = depth.at(c.x, c.y);
    //convert pcl::PointXYZ to geometry_msgs::Point for publishing
    geometry_msgs::PointStamped target; 
    target.point.x = p.x; target.point.y = p.y; target.point.z = p.z; 
    //set the frame of reference for the point
    target.header.frame_id = "head_camera_rgb_optical_frame";
    //set stamp to current time
    target.header.stamp = ros::Time::now();
    //publish geometry_msgs::Point of centroid
    ros::Rate rate(1);
    while(ros::ok()){
      ros::spinOnce();
      pubPoint_.publish(target);
      rate.sleep();
    }
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
}


