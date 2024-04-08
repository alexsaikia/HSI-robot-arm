#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <acquisition/save_images.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <tf2_ros/buffer.h>
#include <vector>

Eigen::Quaterniond quatmean(const std::vector<Eigen::Quaterniond>& quaternions) {
    Eigen::Vector4d sum(0, 0, 0, 0);
    for (const auto& quat : quaternions) {
        sum += quat.coeffs();
    }
    sum /= quaternions.size();
    Eigen::Quaterniond qa(sum);
    qa.normalize(); // Ensure the result is a unit quaternion
    return qa;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking_support");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get and define the MoveIt planning group
  static const std::string PLANNING_GROUP = "arm";

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Print the name of the reference frame for this robot.
  ROS_INFO_NAMED("Tracking", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // Print the name of the end-effector link for this group.
  ROS_INFO_NAMED("Tracking", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // List of all the groups in the robot:
  ROS_INFO_NAMED("Tracking", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

  // Check to see if the marker12 reference frame is available using tf and obtain the pose
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;
  // Create a true flag for the while loop
  bool marker12_flag = false;
  // Create a vector to store transformStamped
  int window_size = 100;
  std::vector<geometry_msgs::TransformStamped> transformStamped(window_size);
  // Set while loop while ros ok
  while (ros::ok())
  {
    // Another while loop that is only active while the user is pressing t
    bool moving;
    nh.getParam("/moving", moving);
    // printf("moving %d\n", moving);
    if(!moving)
    {
      try
      {
        if (marker12_flag)
        {
          // Lookup the transform from the base_link to the marker_12 and copy to each element of the vector directly
          for (int i = 0; i < window_size; i++)
          {
            transformStamped[i] = tfBuffer.lookupTransform("base_link", "marker_12", ros::Time(0));
          }
          // Set the flag to false
          marker12_flag = false;
        }
        else
        {
            geometry_msgs::TransformStamped temp_transformStamped;
            temp_transformStamped = tfBuffer.lookupTransform("base_link", "marker_12", ros::Time(0));
            // Create an exponential moving average of the transform from the base_link to the marker_12 using transformStamped
            for (int i = 0; i < window_size - 1; i++)
            {
                transformStamped[i]=transformStamped[i+1];
            }
            transformStamped[window_size - 1] = temp_transformStamped;
            // Average the transformStamped and weight the more recent terms
            geometry_msgs::TransformStamped average_transformStamped;
            average_transformStamped.header.stamp = ros::Time::now();
            average_transformStamped.header.frame_id = "base_link";
            average_transformStamped.child_frame_id = "average_marker_12";
            float rotx=0, roty=0, rotz=0, rotw = 0;
            // Ra = Eigen::Matrix3d::Zero();
            std::vector<Eigen::Quaterniond> quaternions;
            for (int i = 0; i < window_size; i++)
            {   
              // Convert the quaternion to rotation matrix
              // Eigen::Quaterniond q(transformStamped[i].transform.rotation.w, 
              //                      transformStamped[i].transform.rotation.x, 
              //                      transformStamped[i].transform.rotation.y, 
              //                      transformStamped[i].transform.rotation.z);
              // Eigen::Matrix3d R = q.toRotationMatrix();
              
              // Get the quaternion as an Eigen quaternion
              Eigen::Quaterniond q(transformStamped[i].transform.rotation.w, 
                                   transformStamped[i].transform.rotation.x, 
                                   transformStamped[i].transform.rotation.y, 
                                   transformStamped[i].transform.rotation.z);
              quaternions.push_back(q);
            
              // rotx += transformStamped[i].transform.rotation.x / (float) window_size;
              // roty += transformStamped[i].transform.rotation.y / (float) window_size;
              // rotz += transformStamped[i].transform.rotation.z / (float) window_size;
              // rotw += transformStamped[i].transform.rotation.w / (float) window_size;
              average_transformStamped.transform.translation.x += transformStamped[i].transform.translation.x / (float) window_size;
              average_transformStamped.transform.translation.y += transformStamped[i].transform.translation.y / (float) window_size;
              average_transformStamped.transform.translation.z += transformStamped[i].transform.translation.z / (float) window_size;
            }
            // Calculate the average quaternion
            Eigen::Quaterniond mean_quaternion = quatmean(quaternions);

            // Normalize the quaternion
            // float norm = sqrt(rotx*rotx + roty*roty + rotz*rotz + rotw*rotw);
            // rotx = rotx/norm;
            // roty = roty/norm;
            // rotz = rotz/norm;
            // rotw = rotw/norm;

            // Set the rotation as the average
            // average_transformStamped.transform.rotation.x = rotx;
            // average_transformStamped.transform.rotation.y = roty;
            // average_transformStamped.transform.rotation.z = rotz;
            // average_transformStamped.transform.rotation.w = rotw;
            average_transformStamped.transform.rotation.x = mean_quaternion.x();
            average_transformStamped.transform.rotation.y = mean_quaternion.y();
            average_transformStamped.transform.rotation.z = mean_quaternion.z();
            average_transformStamped.transform.rotation.w = mean_quaternion.w();

            // printf("time %f, x %f, y %f, z %f\n", average_transformStamped.header.stamp.toSec(), average_transformStamped.transform.translation.x, average_transformStamped.transform.translation.y, average_transformStamped.transform.translation.z);
            // printf("time %f, x %f, y %f, z %f, qx %f, qy %f, qz %f, qw %f\n", average_transformStamped.header.stamp.toSec(), average_transformStamped.transform.translation.x, average_transformStamped.transform.translation.y, average_transformStamped.transform.translation.z, average_transformStamped.transform.rotation.x, average_transformStamped.transform.rotation.y, average_transformStamped.transform.rotation.z, average_transformStamped.transform.rotation.w);
            // Broadcast the average transformStamped
            tfBroadcaster.sendTransform(average_transformStamped);
        }
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
      }
      // Calculate the rotation matrix from the quaternion
      // Eigen::Quaterniond q(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);
      // Eigen::Matrix3d R = q.toRotationMatrix();
      // Get x rotation vector
      // Eigen::Vector3d x = R.col(0);
      // Create a new reference frame by taking the marker_12 position and adding 0.2 of the x vector
      try
      {
      geometry_msgs::TransformStamped new_transformStamped;
      new_transformStamped.header.stamp = ros::Time::now();
      new_transformStamped.header.frame_id = "average_marker_12";
      new_transformStamped.child_frame_id = "target_frame";
      new_transformStamped.transform.translation.x = 0.3; 
      new_transformStamped.transform.translation.y = 0.0; 
      new_transformStamped.transform.translation.z = 0.0; 
      // Set the rotation as the identity
      new_transformStamped.transform.rotation.x = -0.5;
      new_transformStamped.transform.rotation.y = -0.5;
      new_transformStamped.transform.rotation.z = 0.5;
      new_transformStamped.transform.rotation.w = 0.5;
      // Broadcast the new reference frame
      tfBroadcaster.sendTransform(new_transformStamped);
     
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
      }
      
    }
  }

  ros::shutdown();
  return 0;
}
