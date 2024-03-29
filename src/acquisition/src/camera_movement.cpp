#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>



//To run this node type in terminal:
//rosrun acquisition acquire_data_sphere __ns:="my_gen3"

//Parameters (defined in launch file):
//N: number of capture points
//sample_rad: radius of the sphere
//sample_point: center of the sphere



// The circle constant tau = 2*pi. One tau is one rotation in radians
const double tau = 2 * M_PI;
//The golden ratio constant:
const double GR = (1 + sqrt(5)) / 2;

int main(int argc, char** argv)
{
  // INIT ROS
  ros::init(argc, argv, "camera_movement");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // BEGIN
  //
  // Setup
  // ^^^^^
  //
  // Get and define the MoveIt planning group
  static const std::string PLANNING_GROUP = "arm";

  // Define number of capture points from ROS parameter server
  int N;
  int NN;
  node_handle.param("/N", NN, 10);
  N = NN*2;

  // Define sample radius from ROS parameter server
  double sample_rad;
  node_handle.param("/sample_rad", sample_rad, 0.1);

  // Define sample point from ROS parameter server
  std::vector<double> s_pos;  
  node_handle.param("/sample_point", s_pos, {0.5, 0, 0.0});

  // The :planning_interface:`MoveGroupInterface` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

  // We will use the :planning_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

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
  visual_tools.publishText(text_pose, "Acquire data sphere", rvt::WHITE, rvt::XLARGE);
  Eigen::Isometry3d text_pose2 = Eigen::Isometry3d::Identity();
  text_pose2.translation().z() = 0.5;
  
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();


  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Print the name of the reference frame for this robot.
  ROS_INFO_NAMED("data_sphere_collection", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

  // Print the name of the end-effector link for this group.
  ROS_INFO_NAMED("data_sphere_collection", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

  // List of all the groups in the robot:
  ROS_INFO_NAMED("data_sphere_collection", "Available Planning Groups:");
  std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));


  // Start the acquisition process
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the acquisition process");

  //Define a vector of capture points on a fibonacci sphere
  std::vector<std::vector<double>> capture_points;
  //Define a vector of capture orientations
  std::vector<std::vector<double>> capture_orientations;
  for (int i = 0; i < NN; i++)
  {
    //Define tf2 quaternion
    tf2::Quaternion q;
    //Define parameters theta and phi for the fibonacci sphere
    double theta = tau * i / GR;
    double phi = acos(1 - 2 * (i + 0.5) / N); //May need to add imaginary number check here
    std::vector<double> capture_point = {sample_rad * sin(phi) * cos(theta) + s_pos[0], 
                                         sample_rad * sin(phi) * sin(theta) + s_pos[1],
                                         sample_rad * cos(phi) + s_pos[2]};
    //Calculate the difference between the capture point and the sample point
    std::vector<double> v = {capture_point[0] - s_pos[0], capture_point[1] - s_pos[1], capture_point[2] - s_pos[2]};
    if (v[0] == 0 && v[1] == 0)
        {
          q.setRPY(M_PI, 0.0, 0.0);
        }
        else if (v[0] <= 0)
        {
          q.setRPY(M_PI - atan(sqrt(v[0] * v[0] + v[1] * v[1]) / v[2]), 0.0, acos(v[1] / (sqrt(v[0] * v[0] + v[1] * v[1]))));
        }
        else
        {
          q.setRPY(M_PI - atan(sqrt(v[0] * v[0] + v[1] * v[1]) / v[2]), 0.0, -acos(v[1] / (sqrt(v[0] * v[0] + v[1] * v[1]))));
        }
        q.normalize();
    std::vector<double> capture_orientation = {q[0], q[1], q[2], q[3]};
    capture_points.push_back(capture_point);
    capture_orientations.push_back(capture_orientation);
  }

  // Define a vector containing all the target poses
  std::vector<geometry_msgs::Pose> target_poses;
  for (int i = 0; i < NN; i++)
  {
    geometry_msgs::Pose target_pose;
    target_pose.position.x = capture_points[i][0];
    target_pose.position.y = capture_points[i][1];
    target_pose.position.z = capture_points[i][2];
    target_pose.orientation.x = capture_orientations[i][0];
    target_pose.orientation.y = capture_orientations[i][1];
    target_pose.orientation.z = capture_orientations[i][2];
    target_pose.orientation.w = capture_orientations[i][3];
    target_poses.push_back(target_pose);  
  }


  // Move the robot to the capture points
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  for (int i = 0; i < NN; i++)
  {
    
    // Set the target pose
    move_group_interface.setPoseTarget(target_poses[i]);
    //Wait for user input
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to plan for the robot to the capture point");

    // Plan the motion
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Visualize the plan in RViz
    visual_tools.publishAxisLabeled(target_poses[i], "pose" + std::to_string(i));
    visual_tools.publishText(text_pose, "Pose Goal" + std::to_string(i), rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    
    //Wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move the robot");
    visual_tools.trigger();
    // Execute the motion if the plan was successful
    if (success)
    {
      move_group_interface.execute(my_plan);
      
    }
    else
    {
      ROS_ERROR("Plan failed");
    }

    //Wait for user input
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to capture images. Remember to capture with the HSI camera also!");
    
  }


  
  
  // END_DATA_SPHERE_COLLECTION
  //Shut it down!

  ros::shutdown();
  return 0;
}