// ROS node to capture images from camera published to /camera/image_color
// Display the images in a window with opencv and save to disk
//
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
// #include <acquisition/save_images.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sensor_msgs/CameraInfo.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Function to save images to disk
// Function inputs include the image, the path to save the image to, and the name of the image
// The function should save the image to the specified path with the specified name
void save_image(cv::Mat image, std::string path, std::string name)
{
    // Create the full path name
    std::string full_path = path + name;
    // Save the image
    cv::imwrite(full_path, image);
}

//ROS node display images from camera published to /camera/image_color. Another node publishes to this topic
//Display the images in a window with opencv and save to disk

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "focus_capture");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
        
        // Create a window to display the images
        cv::namedWindow("view");
        cv::startWindowThread();
        
        // Create a variable to hold the image
        cv::Mat g_image;
        cv::Mat shown_image;
        // Create a variable to hold the path to save the images to
        std::string path = "/home/alex/data/enacuity/focus/data_1/";
        // Create 3 lists of image names. 
        std::vector<std::string> relay_lens_type = {"triplet","singlet"};
        std::vector<std::string> relay_lens_position = {"close","middle","far"};
        std::vector<std::string> aperture_position = {"open", "middle", "min"};
        std::vector<double> distances = {150, 200, 250, 300};
        // Create a variable to hold the image name
        std::string image_name;
        
        // Create 3 dropdown menus using the lists of image names
        // Create a variable to hold the relay lens type
        std::string relay_lens_type_;
        // Create a variable to hold the relay lens position
        std::string relay_lens_position_;
        // Create a variable to hold the aperture position
        std::string aperture_position_;
        // Create a variable to hold distances
        double distance_value_;

        // Create the dropdown menu in the opencv window
        cv::createTrackbar("relay_lens_type", "view", 0, relay_lens_type.size()-1);
        cv::createTrackbar("relay_lens_position", "view", 0, relay_lens_position.size()-1);
        cv::createTrackbar("aperture_position", "view", 0, aperture_position.size()-1);
        cv::createTrackbar("distance", "view", 0, distances.size()-1);
        cv::createTrackbar("zoom", "view", 0, 99);

        // Put the trackbars to the right of the image

        // Create a variable to hold the current value of the dropdown menu
        int relay_lens_type_value = 0;
        int relay_lens_position_value = 0;
        int aperture_position_value = 0;
        int distance_value = 150;
        float zoom_value = 0;

        // Create cv pointer
        cv_bridge::CvImagePtr cv_ptr;

        // Create a variable to hold the image topic from the parameter server (parameter name is image_topic) with the default being /camera/image_color
        std::string img_topic;
        nh.param<std::string>("image_topic", img_topic, "/camera/image_color");

        
        while(ros::ok())
        {
            // Get the current value of the dropdown menu
            relay_lens_type_value = cv::getTrackbarPos("relay_lens_type", "view");
            relay_lens_position_value = cv::getTrackbarPos("relay_lens_position", "view");
            aperture_position_value = cv::getTrackbarPos("aperture_position", "view");
            distance_value = cv::getTrackbarPos("distance", "view");
            zoom_value = cv::getTrackbarPos("zoom", "view");

            // Get the current image name
            relay_lens_type_ = relay_lens_type[relay_lens_type_value];
            relay_lens_position_ = relay_lens_position[relay_lens_position_value];
            aperture_position_ = aperture_position[aperture_position_value];
            distance_value_ = distances[distance_value];
            // Create the image name
            image_name = relay_lens_type_ + "_" + relay_lens_position_ + "_" + aperture_position_ + std::to_string(distance_value_) +".png";
            // Get the image from the camera
            auto img_msg = ros::topic::waitForMessage<sensor_msgs::Image>(img_topic, ros::Duration(5));
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
            g_image = cv_ptr->image;
            // Crop the image according to the zoom value
            // cv::Rect myROI(zoom_value/2, zoom_value/2, g_image.cols *(100 - zoom_value/2)/100, g_image.rows *(100- zoom_value/2)/100);
            shown_image = g_image(cv::Range(g_image.rows * zoom_value/200, 
                                            g_image.rows * (100-zoom_value/2)/100),
                                  cv::Range(g_image.cols * zoom_value/200, 
                                            g_image.cols * (100-zoom_value/2)/100));
            //Resize the image 
            cv::resize(shown_image, shown_image, cv::Size(g_image.cols*0.625, g_image.rows*0.625));
            // Put the image name on the image
            cv::putText(shown_image, image_name, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            // Display the image in the opencv window
            cv::imshow("view", shown_image);
            // If the user presses the save button then save the image
            if(cv::waitKey(30) == 's')
            {
                save_image(g_image, path, image_name);
            }      
                        
        }


}
    


