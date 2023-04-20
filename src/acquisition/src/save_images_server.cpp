#include "ros/ros.h"
#include "acquisition/save_images.h"
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

// Global variables to store the latest left and right images
cv::Mat g_left_image;
cv::Mat g_right_image;

// // Callback function subscribed to the left camera image topic
// void camLeftCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         g_left_image = cv_ptr->image;
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
// }

// // Callback function subscribed to the right camera
// void camRightCallback(const sensor_msgs::ImageConstPtr& msg)
// {
//     cv_bridge::CvImagePtr cv_ptr;
//     try
//     {
//         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//         g_right_image = cv_ptr->image;
//     }
//     catch (cv_bridge::Exception& e)
//     {
//         ROS_ERROR("cv_bridge exception: %s", e.what());
//         return;
//     }
// }

// Service function to save the images
bool save(acquisition::save_images::Request &req, acquisition::save_images::Response &res)
{   

    std::string left_file_name = req.left_file_name;
    std::string right_file_name = req.right_file_name;

    cv_bridge::CvImagePtr cv_ptr_left;
    try
    {
        auto left_img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camLeft/image_color", ros::Duration(5));
        cv_ptr_left = cv_bridge::toCvCopy(left_img_msg, sensor_msgs::image_encodings::BGR8);
        g_left_image = cv_ptr_left->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    cv_bridge::CvImagePtr cv_ptr_right;
    try
    {
        auto right_img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/camRight/image_color", ros::Duration(5));
        cv_ptr_right = cv_bridge::toCvCopy(right_img_msg, sensor_msgs::image_encodings::BGR8);
        g_right_image = cv_ptr_right->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }



    // Check if the left and right images have been received
    if (g_left_image.empty() || g_right_image.empty())
    {
        ROS_ERROR("Images not available.");
        return false;
    }

    // save the left image
    ROS_INFO("Attempting to save left image. File name: %s", left_file_name.c_str());
    if (!cv::imwrite(left_file_name, g_left_image))
    {
        ROS_ERROR("Failed to save left image.");
        return false;
    }
    ROS_INFO("Left image saved successfully.");

    // save the right image
    ROS_INFO("Attempting to save right image. File name: %s", right_file_name.c_str());
    if (!cv::imwrite(right_file_name, g_right_image))
    {
        ROS_ERROR("Failed to save right image.");
        return false;
    }
    ROS_INFO("Right image saved successfully.");

    ROS_INFO("Images saved successfully.");
    res.success = true;
    return true;
}
    

int main(int argc, char **argv)
{
    // Initializing the node
    ros::init(argc, argv, "save_images_server");
    ros::NodeHandle node_handle;

    // Advertising the service and passing through the node handle
    ros::ServiceServer service = node_handle.advertiseService("save_images", save);
    ROS_INFO("Ready to save images.");

    // // Subscribing to the topic and setting up the image transport for the left camera
    // image_transport::ImageTransport it_left(node_handle);
    // image_transport::Subscriber sub_camLeft = it_left.subscribe("camLeft/image_color", 1, camLeftCallback);

    // // Subscribing to the topic and setting up the image transport for the right camera
    // image_transport::ImageTransport it_right(node_handle);
    // image_transport::Subscriber sub_camRight = it_right.subscribe("camRight/image_color", 1, camRightCallback);

    // Spinning the node
    ros::spin();

    return 0;
}