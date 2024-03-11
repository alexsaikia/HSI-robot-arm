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
#include <tf2_ros/buffer.h>



//Define save camera info function
void saveCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, std::string file_name)
{
  std::ofstream cam_info;
    cam_info.open(file_name);
    cam_info << "Timestamp:," << msg->header.stamp.sec <<"\n"
             << "Frame ID:," << msg->header.frame_id << "\n"
             << "Height:," << msg->height << "\n"
             << "Width:," << msg->width << "\n"
             << "Distortion model:," << msg->distortion_model << "\n"
             << "D:,";
    for (auto i : msg->D)
      cam_info << i << ",";
    cam_info << "\n"
             << "K:,";
    for (auto i : msg->K)
      cam_info << i << ",";
    cam_info << "\n"
             << "R:,";
    for (auto i : msg->R)
      cam_info << i << ",";
    cam_info << "\n"
             << "P:,";
    for (auto i : msg->P)
      cam_info << i << ",";
    cam_info << "\n" <<"Bin X:," << msg->binning_x << "\n"
             << "Bin Y:," << msg->binning_y << "\n";
    // // Close txt file
    cam_info.close();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Get and define the MoveIt planning group
  static const std::string PLANNING_GROUP = "arm";

  // Define number of capture points from ROS parameter server
  int N;
  int NN;
  nh.param("/N", NN, 10);
  N = NN*2;

  // Define sample radius from ROS parameter server
  double sample_rad;
  nh.param("/sample_rad", sample_rad, 0.2);

  // Define sample point from ROS parameter server
  std::vector<double> s_pos;  
  nh.param("/sample_point", s_pos, {0.50, 0, 0.00});

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Set maximum velocity scaling factor
  move_group_interface.setMaxVelocityScalingFactor(1.0);

  // Set maximum acceleration scaling factor
  move_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Define a collision object ROS message.
  auto const sphere_sample_collision_object = [frame_id = move_group_interface.getPlanningFrame(), s_pos,sample_rad]
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "sphere_sample";
    shape_msgs::SolidPrimitive primitive;

    // Define the size of the sphere in meters
    primitive.type = primitive.SPHERE;
    primitive.dimensions.resize(1);
    primitive.dimensions[primitive.SPHERE_RADIUS] = 0.9 * sample_rad;

    // Define the pose of the sphere (relative to the frame_id)
    geometry_msgs::Pose sphere_pose;
    sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = s_pos[0];
    sphere_pose.position.y = s_pos[1];
    sphere_pose.position.z = s_pos[2];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(sphere_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  // Add the collision object into the world  
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(sphere_sample_collision_object);
  // planning_scene_interface.addCollisionObjects(collision_objects);

  // Setup for data capture
  // ^^^^^^^^^^^^^^^^^^^^^^
  //
  // If rosparam capture is true then start the capture service
  bool capture_data = false;
  ros::param::get("/capture", capture_data);
  // Start the capture service
  ros::ServiceClient client = nh.serviceClient<acquisition::save_images>("/save_images");
  // If initializing the client fails then set capture_data to false
  if (!client.waitForExistence(ros::Duration(5.0)))
  {
    ROS_ERROR("Could not connect to save_images service");
    capture_data = false;
  }
  else 
  {
    ROS_INFO("Connected to save_images service");
  }

  // If capturing data then create dataset folder using filesystem
  if (capture_data)
  {
    // Create a folder for the dataset
    std::string dataset_folder = nh.param("/dataset_folder", std::string("/media/alex/Data/data"));
    std::filesystem::create_directory(dataset_folder);
    // Create a folder for the current dataset
    std::string current_dataset_folder = dataset_folder + "/dataset_tracking_" + std::to_string(std::time(0));
    std::filesystem::create_directory(current_dataset_folder);
    // Assign current dataset folder to parameter server
    ros::param::set("/current_dataset_folder", current_dataset_folder);

    // Create a folder for the left camera images
    std::string left_folder = current_dataset_folder + "/left";
    std::filesystem::create_directory(left_folder);
    // Assign left folder to parameter server
    ros::param::set("/left_folder", left_folder);
    // Save left camera parameters (topic /left_camera/camera_info) to file in left folder
    auto left_cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/left_camera/camera_info", ros::Duration(5.0));
    saveCameraInfo(left_cam_info, left_folder + "/camera_info.csv");

    // Create a folder for the right camera images
    std::string right_folder = current_dataset_folder + "/right";
    std::filesystem::create_directory(right_folder);
    // Assign right folder to parameter server
    ros::param::set("/right_folder", right_folder);
    // Save right camera parameters (topic /right_camera/camera_info) to file in right folder
    auto right_cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/right_camera/camera_info", ros::Duration(5.0));
    saveCameraInfo(right_cam_info, right_folder + "/camera_info.csv");

    //Create end effector pose csv file
    std::string eef_file_name = current_dataset_folder + "/eef_pose.csv";
    // Assign eef file name to parameter server
    ros::param::set("/eef_file_name", eef_file_name);
    std::ofstream eef_file;
    eef_file.open(eef_file_name);
    eef_file << "time,img,frame id,x,y,z,qx,qy,qz,qw\n";
    eef_file.close();
  }

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilities for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script.
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  //Create two text poses
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.0;
  visual_tools.publishText(text_pose, "Tracking", rvt::WHITE, rvt::XLARGE);
  Eigen::Isometry3d text_pose2 = Eigen::Isometry3d::Identity();
  text_pose2.translation().z() = 0.5;
  
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


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
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  while (ros::ok())
  {
    // Get the pose of target_frame in the base_link frame
    geometry_msgs::TransformStamped target_transformStamped;
    try
    {
      target_transformStamped = tfBuffer.lookupTransform("base_link", "target_frame", ros::Time(0));
      // Print the pose of the target_frame in the base_link frame
      // printf("time %f, x %f, y %f, z %f\n", target_transformStamped.header.stamp.toSec(),
                                            // target_transformStamped.transform.translation.x, 
                                            // target_transformStamped.transform.translation.y, 
                                            // target_transformStamped.transform.translation.z);
      // Plan a cartesian path to the target_frame
      std::vector<geometry_msgs::Pose> waypoints;
      geometry_msgs::Pose target_pose;
      target_pose.position.x = target_transformStamped.transform.translation.x;
      target_pose.position.y = target_transformStamped.transform.translation.y;
      target_pose.position.z = target_transformStamped.transform.translation.z;
      target_pose.orientation.x = target_transformStamped.transform.rotation.x;
      target_pose.orientation.y = target_transformStamped.transform.rotation.y;
      target_pose.orientation.z = target_transformStamped.transform.rotation.z;
      target_pose.orientation.w = target_transformStamped.transform.rotation.w;
      waypoints.push_back(target_pose);
      moveit_msgs::RobotTrajectory trajectory;
      // const double jump_threshold = 0.1;
      // const double eef_step = 0.001;
      // double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
      // plan the trajectory
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      move_group_interface.setPoseTarget(target_pose);
      bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      // Visualize the plan in RViz
      visual_tools.deleteAllMarkers();
      visual_tools.publishAxisLabeled(target_pose, "target_pose");
      visual_tools.publishText(text_pose2, "Pose Goal", rvt::WHITE, rvt::XLARGE);
      visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
      visual_tools.trigger();



      // Wait for the user to press next in the gui
      visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
      // Execute the trajectory 
      // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      // my_plan.trajectory_ = trajectory;
      move_group_interface.execute(my_plan);
    
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
    }
  }
  

  ros::shutdown();
  return 0;
}