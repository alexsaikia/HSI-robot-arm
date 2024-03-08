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
  planning_scene_interface.addCollisionObjects(collision_objects);

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

  // Check to see if the marker12 reference frame is available using tf and obtain the pose
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;
  // Create a true flag for the while loop
  bool marker12_flag = false;
  // Create a 10 long vector to store transformStamped
  std::vector<geometry_msgs::TransformStamped> transformStamped(10);
  // Set while loop while ros ok
  while (ros::ok())
  {
    // Another while loop that is only active while the user is pressing t
    if(1==1)
    {
      try
      {
        if (marker12_flag)
        {
          // Lookup the transform from the base_link to the marker_12 and copy to each element of the vector directly
          for (int i = 0; i < 10; i++)
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
        for (int i = 0; i < 9; i++)
        {
          transformStamped[i]=transformStamped[i+1];
        }
        transformStamped[9] = temp_transformStamped;
        // Average the transformStamped and weight the more recent terms
        geometry_msgs::TransformStamped average_transformStamped;
        average_transformStamped.header.stamp = ros::Time::now();
        average_transformStamped.header.frame_id = "base_link";
        average_transformStamped.child_frame_id = "average_marker_12";
        average_transformStamped.transform.translation.x = (transformStamped[0].transform.translation.x + 2*transformStamped[1].transform.translation.x + 3*transformStamped[2].transform.translation.x + 4*transformStamped[3].transform.translation.x + 5*transformStamped[4].transform.translation.x + 6*transformStamped[5].transform.translation.x + 7*transformStamped[6].transform.translation.x + 8*transformStamped[7].transform.translation.x + 9*transformStamped[8].transform.translation.x + 10*transformStamped[9].transform.translation.x)/55;
        average_transformStamped.transform.translation.y = (transformStamped[0].transform.translation.y + 2*transformStamped[1].transform.translation.y + 3*transformStamped[2].transform.translation.y + 4*transformStamped[3].transform.translation.y + 5*transformStamped[4].transform.translation.y + 6*transformStamped[5].transform.translation.y + 7*transformStamped[6].transform.translation.y + 8*transformStamped[7].transform.translation.y + 9*transformStamped[8].transform.translation.y + 10*transformStamped[9].transform.translation.y)/55;
        average_transformStamped.transform.translation.z = (transformStamped[0].transform.translation.z + 2*transformStamped[1].transform.translation.z + 3*transformStamped[2].transform.translation.z + 4*transformStamped[3].transform.translation.z + 5*transformStamped[4].transform.translation.z + 6*transformStamped[5].transform.translation.z + 7*transformStamped[6].transform.translation.z + 8*transformStamped[7].transform.translation.z + 9*transformStamped[8].transform.translation.z + 10*transformStamped[9].transform.translation.z)/55;
        average_transformStamped.transform.rotation.x = (transformStamped[0].transform.rotation.x + 2*transformStamped[1].transform.rotation.x + 3*transformStamped[2].transform.rotation.x + 4*transformStamped[3].transform.rotation.x + 5*transformStamped[4].transform.rotation.x + 6*transformStamped[5].transform.rotation.x + 7*transformStamped[6].transform.rotation.x + 8*transformStamped[7].transform.rotation.x + 9*transformStamped[8].transform.rotation.x + 10*transformStamped[9].transform.rotation.x)/55;
        average_transformStamped.transform.rotation.y = (transformStamped[0].transform.rotation.y + 2*transformStamped[1].transform.rotation.y + 3*transformStamped[2].transform.rotation.y + 4*transformStamped[3].transform.rotation.y + 5*transformStamped[4].transform.rotation.y + 6*transformStamped[5].transform.rotation.y + 7*transformStamped[6].transform.rotation.y + 8*transformStamped[7].transform.rotation.y + 9*transformStamped[8].transform.rotation.y + 10*transformStamped[9].transform.rotation.y)/55;
        average_transformStamped.transform.rotation.z = (transformStamped[0].transform.rotation.z + 2*transformStamped[1].transform.rotation.z + 3*transformStamped[2].transform.rotation.z + 4*transformStamped[3].transform.rotation.z + 5*transformStamped[4].transform.rotation.z + 6*transformStamped[5].transform.rotation.z + 7*transformStamped[6].transform.rotation.z + 8*transformStamped[7].transform.rotation.z + 9*transformStamped[8].transform.rotation.z + 10*transformStamped[9].transform.rotation.z)/55;
        average_transformStamped.transform.rotation.w = (transformStamped[0].transform.rotation.w + 2*transformStamped[1].transform.rotation.w + 3*transformStamped[2].transform.rotation.w + 4*transformStamped[3].transform.rotation.w + 5*transformStamped[4].transform.rotation.w + 6*transformStamped[5].transform.rotation.w + 7*transformStamped[6].transform.rotation.w + 8*transformStamped[7].transform.rotation.w + 9*transformStamped[8].transform.rotation.w + 10*transformStamped[9].transform.rotation.w)/55;
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
      geometry_msgs::TransformStamped new_transformStamped;
      new_transformStamped.header.stamp = ros::Time::now();
      new_transformStamped.header.frame_id = "average_marker_12";
      new_transformStamped.child_frame_id = "target_frame";
      new_transformStamped.transform.translation.x = 0.2; 
      new_transformStamped.transform.translation.y = 0.0; 
      new_transformStamped.transform.translation.z = 0.0; 
      // Set the rotation as the identity
      new_transformStamped.transform.rotation.x = -0.5;
      new_transformStamped.transform.rotation.y = -0.5;
      new_transformStamped.transform.rotation.z = 0.5;
      new_transformStamped.transform.rotation.w = 0.5;
      // Broadcast the new reference frame
      tfBroadcaster.sendTransform(new_transformStamped);
      // Sleep for 1 second
      // ros::Duration(1.0).sleep();
      
    }
  }

  ros::shutdown();
  return 0;
}