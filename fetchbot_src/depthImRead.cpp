////////////////////////////////////////////////////////////////////////////////////
// ROS node for saving images from the following topics:                          //
//   - "/head_camera/depth/image_raw": saved as 1 channel 16 bit unsigned int PNG //
//   - "/head_camera/rgb/image_raw ": saved as 3 channel 8 bit unsinged int PNG   //
////////////////////////////////////////////////////////////////////////////////////
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <vector>

//////////////////////////////////////////////////////////////////////////////////
// Synchronisation is used on the subscribers to yield images that are captured //
// roughly the same time                                                        //
//////////////////////////////////////////////////////////////////////////////////
//#define EXACT
#define APPROXIMATE
#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif
using namespace message_filters;

///////////////////////
// CALLBACK FUNCTION //
///////////////////////
unsigned int cnt = 1; // Counter for filenames.
void callback(const sensor_msgs::ImageConstPtr& msg_rgb, const sensor_msgs::ImageConstPtr& msg_depth)
{
    //Image pointers for depth and RGB
    cv_bridge::CvImagePtr img_ptr_rgb; cv_bridge::CvImagePtr img_ptr_depth;

    //Depth image
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    //RGB image
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception:  %s", e.what());
        return;
    }

    //RGB and Depth images stored
    cv::Mat& mat_depth = img_ptr_depth->image; cv::Mat& mat_rgb = img_ptr_rgb->image;

    //file names
    char file_rgb[100]; char file_depth[100];
    sprintf(file_rgb, "%04d_rgb.png", cnt);
    sprintf(file_depth, "%04d_depth.png", cnt);

    //Save with no compression for faster processing
    std::vector<int> png_params;
    png_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    png_params.push_back(9); 

    //write images to data folder
    cv::imwrite(file_rgb, mat_rgb, png_params); cv::imwrite(file_depth, mat_depth, png_params);

    ROS_INFO_STREAM(cnt << "\n");
    ROS_INFO_STREAM("Images saved\n");
    cnt++;
}

//////////
// MAIN //
//////////
int main(int argc, char** argv){
    // Initialize the ROS system and become a node.
    ros::init(argc, argv, "guardar_imagenes");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> subscriber_depth(
        nh, "/head_camera/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb(
        nh, "/head_camera/rgb/image_raw", 1);

    //Synchronisation policy for images
    #ifdef EXACT
        typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    #endif
    #ifdef APPROXIMATE
        typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    #endif

    // Sync policy takes a queue size as its constructor argument
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
    sync.registerCallback(boost::bind(&callback, _1, _2));

    //Spin ros node
    while(ros::ok()){
        char c;
        ROS_INFO_STREAM("\nEnter 'a' to save a pair of images or 'b' to automatically save 300 images\n");
        std::cin.get(c);
        std::cin.ignore();
        c = tolower(c);
        ROS_INFO_STREAM("You entered " << c << "\n");
        if( c == 'a' ) ros::spinOnce();
        else if( c == 'b' ) {
            unsigned int cnt_init = cnt;
            while( cnt - cnt_init < 300 ) ros::spinOnce();
        }
        else break;
    }
    ROS_INFO_STREAM("Closing node\n");
    return 0;
}