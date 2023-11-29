#include "ros/ros.h"
#include "acquisition/capture_image.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

// Global variables to store the latest image
cv::Mat g_image;

// Service function to save the images
bool save(acquisition::capture_image::Request &req, acquisition::capture_image::Response &res)
{   
    // Get the service request parameters
    int img_num = req.img_num;
    std::string file_name = req.filename;
    std::string img_topic = req.img_topic;
    
    // Get the image
    // ^^^^^^^^^^^^^
    // Image is published as ROS topic. The image received as ROS message and converted to OpenCV image.
    //
    // Get the image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {   
        auto img_msg = ros::topic::waitForMessage<sensor_msgs::Image>(img_topic, ros::Duration(5));
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        g_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }


    // Check if the image have been received
    if (g_image.empty())
    {
        ROS_ERROR("Images not available.");
        return false;
    }

    // save the image
    ROS_INFO("Attempting to save image. File name: %s",file_name.c_str());
    if (!cv::imwrite(file_name, g_image))
    {
        ROS_ERROR("Failed to save image.");
        return false;
    }  

    ROS_INFO("Image saved successfully.");
}
    

int main(int argc, char **argv)
{
    // Initializing the node
    ros::init(argc, argv, "capture_image_server");
    ros::NodeHandle node_handle;

    // Advertising the service and passing through the node handle
    ros::ServiceServer service = node_handle.advertiseService("capture_image", save);
    ROS_INFO("Ready to save images.");

    // Spinning the node
    ros::spin();

    return 0;
}