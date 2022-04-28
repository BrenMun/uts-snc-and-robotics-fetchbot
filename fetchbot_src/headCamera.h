#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
using namespace sensor_msgs;

class HeadCamera
{
    private:
    ros::NodeHandle nh_;
    message_filters::Subscriber<Image> sub_rgb_;
    message_filters::Subscriber<PointCloud2> sub_depth_;

    typedef message_filters::sync_policies::ApproximateTime<Image, PointCloud2> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    public:
    HeadCamera();
    void callback(const ImageConstPtr& msg_rgb, const PointCloud2ConstPtr& msg_depth);
};