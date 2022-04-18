#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>



int main(int argc, char **argv)
{
	//////////////
	// INIT ROS //
	//////////////
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;  
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
	//////////////////
    // SETUP MOVEIT //
    //////////////////
    moveit::planning_interface::MoveGroupInterface group("arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ///////////////////////////////
    // Getting Basic Information //
    ///////////////////////////////
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    /////////////////////////////
    // Planning to a Pose goal //
    /////////////////////////////
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(
        group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), 
        group_variable_values);

    group_variable_values[0] = -1.0;
    group.setJointValueTarget(group_variable_values);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::core::MoveItErrorCode success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 2 (joint space goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);

    //////////
    // Move //
    //////////
    group.move();

    ros::shutdown();
    return 0;
}
