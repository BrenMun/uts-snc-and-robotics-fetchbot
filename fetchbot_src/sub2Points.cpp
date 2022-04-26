#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <iostream>

void cloudCb(const sensor_msgs::PointCloud2ConstPtr & cloud) {
  pcl::PointCloud<pcl::PointXYZ> depth;
  pcl::fromROSMsg(*cloud, depth);
  pcl::PointXYZ p1 = depth.at(233, 275);
  pcl::PointXYZ 
  std::cout << "Point: (" << p1.x << ", " << p1.y << ", " << p1.z << ")\n";
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/head_camera/depth_registered/points", 1, &cloudCb);
  ros::spin();
}
