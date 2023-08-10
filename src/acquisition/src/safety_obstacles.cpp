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

//A node to add obstacles to the scene to constrain the robots movement and prevent collisions

// Define a function make_box() that creates a box in the scene and returns it
// Function inputs include the name of the box, the dimensions of the box (as an array), the pose of the box (as an array), and the frame_id
// The function should create a collision object, set its shape, pose, and name
// The function should return the collision object, this will be added to the scene later
moveit_msgs::CollisionObject make_box(std::string name, std::array<double, 3> dimensions, std::array<double, 3> pose, std::string frame_id)
{
    // Create a collision object
    moveit_msgs::CollisionObject collision_object;
    // Set the frame_id
    collision_object.header.frame_id = frame_id;
    // Set the id of the object
    collision_object.id = name;
    // Create a box
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    // Set the dimensions of the box
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = dimensions[0];
    primitive.dimensions[1] = dimensions[1];
    primitive.dimensions[2] = dimensions[2];
    // Set the pose of the box
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = pose[0];
    box_pose.position.y = pose[1];
    box_pose.position.z = pose[2];
    // Add the primitive to the collision object
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    // Set the operation to add the object
    collision_object.operation = collision_object.ADD;
    // Return the collision object
    return collision_object;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "safety_obstacles");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Safety Obstacles Node Started");
    // Get and define the planning group
    static const std::string PLANNING_GROUP = "arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    ROS_INFO("Reference frame: %s", move_group_interface.getPlanningFrame().c_str());
    // Define the planning scene interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ROS_INFO("Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
      // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    ROS_INFO("Reference frame: %s", move_group_interface.getPlanningFrame().c_str());
    
    // Create collision objects using make_box()
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    // Create boxes to constrain the movement of the robot and push them to the vector
    // The boxes will act as walls around the robot
    ROS_INFO("Creating collision objects");
    // Create a box to the left of the robot
    collision_objects.push_back(make_box("left_wall", {2.4, 0.01, 1.0}, {1.0, -0.35, 0.5}, move_group_interface.getPlanningFrame()));
    ROS_INFO("Collision object 1 created");
    // Create a box to the right of the robot
    collision_objects.push_back(make_box("right_wall", {2.4, 0.01, 1.0}, {1.0, 0.35, 0.5}, move_group_interface.getPlanningFrame()));
    ROS_INFO("Collision object 2 created");
    // Create a box in front of the robot
    collision_objects.push_back(make_box("front_wall", {0.01, 1.0, 1.0}, {0.8, 0.0, 0.5}, move_group_interface.getPlanningFrame()));
    ROS_INFO("Collision object 3 created");
    // Create a box behind the robot
    collision_objects.push_back(make_box("back_wall", {0.01, 1.0, 1.0}, {-0.15, 0.0, 0.5}, move_group_interface.getPlanningFrame()));
    ROS_INFO("Collision object 4 created");
    // Create a box above the robot
    collision_objects.push_back(make_box("top_wall", {2.0, 1.0, 0.01}, {0.5, 0.0, 1.00}, move_group_interface.getPlanningFrame()));
    ROS_INFO("Collision object 5 created");
    // Create a box below the robot
    collision_objects.push_back(make_box("bottom_wall", {2.0, 1.0, 0.01}, {1.0, 0.0, -0.069}, move_group_interface.getPlanningFrame()));
    ROS_INFO("Collision object 6 created");

    // Add the collision objects to the scene
    planning_scene_interface.addCollisionObjects(collision_objects);
    ROS_INFO("Collision objects added to the scene");



    
}

