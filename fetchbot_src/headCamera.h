#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

//#define EXACT
#define APPROXIMATE
#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif
using namespace message_filters;
using namespace sensor_msgs;

class HeadCamera
{
    private:
    message_filters::Subscriber<sensor_msgs::Image> sub_rgb_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth_;
    public:
    HeadCamera();
    void sub2Cam(ros::NodeHandle *nh);
    // void callback(const ImageConstPtr& msg_rgb, const PointCloud2ConstPtr& msg_depth);
};