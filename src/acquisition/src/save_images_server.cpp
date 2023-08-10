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
#include <geometry_msgs/PoseStamped.h>

// Global variables to store the latest left and right images
cv::Mat g_left_image;
cv::Mat g_right_image;


// Service function to save the images
bool save(acquisition::save_images::Request &req, acquisition::save_images::Response &res)
{   
    // Get the service request parameters
    int img_num = req.img_num;
    std::string left_file_name = req.left_file_name;
    std::string right_file_name = req.right_file_name;
    
    // Get the left and right images
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Images are published as ROS topics. The images are received as ROS messages and converted to OpenCV images.
    //
    // Get the left image
    cv_bridge::CvImagePtr cv_ptr_left;
    try
    {   
        auto left_img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/left_camera/image_color", ros::Duration(5));
        cv_ptr_left = cv_bridge::toCvCopy(left_img_msg, sensor_msgs::image_encodings::BGR8);
        g_left_image = cv_ptr_left->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }

    //Get the right images
    cv_bridge::CvImagePtr cv_ptr_right;
    try
    {   
        auto right_img_msg = ros::topic::waitForMessage<sensor_msgs::Image>("/right_camera/image_color", ros::Duration(5));
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

    // Open the eef_pose file
    geometry_msgs::PoseStamped eef_pose_msg = req.eef_pose;
    std::string eef_file_name = req.eef_file_name;
    std::ofstream eef_pose_file;
    eef_pose_file.open(eef_file_name, std::ios::app);

    // Save the eef_pose to file
    char buffer[25];
    sprintf(buffer, "%04d", img_num);
    eef_pose_file
        << eef_pose_msg.header.stamp.sec << "." << eef_pose_msg.header.stamp.nsec << ","
        << eef_pose_msg.header.frame_id << ","
        << buffer << ","
        << eef_pose_msg.pose.position.x << ","
        << eef_pose_msg.pose.position.y << ","
        << eef_pose_msg.pose.position.z << ","
        << eef_pose_msg.pose.orientation.x << ","
        << eef_pose_msg.pose.orientation.y << ","
        << eef_pose_msg.pose.orientation.z << ","
        << eef_pose_msg.pose.orientation.w 
        << "\n";
    eef_pose_file.close();

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

    // Spinning the node
    ros::spin();

    return 0;
}