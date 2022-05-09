/*
In MoveIt!, the primary user interface is through the MoveGroup class. It provides easy to use 
functionality for most operations that a user may want to carry out, specifically setting joint 
or pose goals, creating motion plans, moving the robot, adding objects into the environment and 
attaching/detaching objects from the robot.
*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>



int main(int argc, char **argv)
{   ////////////////
    // INPUT POSE //
    ////////////////
	geometry_msgs::Pose target_pose1;
    std::cout << "Input Position (x,y,z):\n";
    std::cout << "    x = "; std::cin >> target_pose1.position.x;
    std::cout << "    y = "; std::cin >> target_pose1.position.y;
    std::cout << "    z = "; std::cin >> target_pose1.position.z;
    target_pose1.orientation.w = 1.0;

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

    // /////////////////////////////
    // // Planning to a Pose goal //
    // /////////////////////////////
    // group.setPoseTarget(target_pose1);
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    // moveit::core::MoveItErrorCode success = group.plan(my_plan);

    // ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    // /* Sleep to give Rviz time to visualize the plan. */
    // sleep(5.0);

    ///////////////////////////////
    // Planning to a Joint-space //
    ///////////////////////////////
    // First get the current set of joint values for the group.
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
    
    // Now, let's modify one of the joints, plan to the new joint
    // space goal and visualize the plan.
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
