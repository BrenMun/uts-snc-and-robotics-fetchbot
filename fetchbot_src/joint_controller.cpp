#include "joint_controller.h"

JointController::JointController(ros::NodeHandle& nh):  nh_(nh),
                                                        trajectory_action_("arm_with_torso_controller/follow_joint_trajectory", true)
{
    while(!trajectory_action_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    traj_sub_ = nh_.subscribe("cartesian_controller/trajectory", 1, &JointController::trajReceived, this);

    goal_.trajectory.joint_names.push_back("torso_lift");
    goal_.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal_.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal_.trajectory.joint_names.push_back("upperarm_roll_joint");
    goal_.trajectory.joint_names.push_back("elbow_flex_joint");
    goal_.trajectory.joint_names.push_back("forearm_roll_joint");
    goal_.trajectory.joint_names.push_back("wrist_flex_joint");
    goal_.trajectory.joint_names.push_back("wrist_roll_joint");

    int size = goal_.trajectory.joint_names.size();
    point_.positions.resize(size);
    point_.velocities.resize(size);
    point_.accelerations.resize(size);
    point_.positions[0]= M_PI_2;
    point_.positions[1]= 0.0;
    point_.positions[2]= 0.0;
    point_.positions[3]= 0.0;
    point_.positions[4]= 0.0;
    point_.positions[5]= 0.0;
    point_.positions[6]= 0.0;
    for (int i = 0; i < size; i ++){
        point_.velocities[i] = 0.0;
        point_.accelerations[i] = 0.0;
    }
    traj_.points.push_back(point_);
}

JointController::JointController(ros::NodeHandle& nh, bool debug):  nh_(nh),
                                                                    trajectory_action_("arm_with_torso_controller/follow_joint_trajectory", true)
{

    while(!trajectory_action_.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    goal_.trajectory.joint_names.push_back("torso_lift");
    goal_.trajectory.joint_names.push_back("shoulder_pan_joint");
    goal_.trajectory.joint_names.push_back("shoulder_lift_joint");
    goal_.trajectory.joint_names.push_back("upperarm_roll_joint");
    goal_.trajectory.joint_names.push_back("elbow_flex_joint");
    goal_.trajectory.joint_names.push_back("forearm_roll_joint");
    goal_.trajectory.joint_names.push_back("wrist_flex_joint");
    goal_.trajectory.joint_names.push_back("wrist_roll_joint");

    if (debug == true){
        int size = goal_.trajectory.joint_names.size();
        point_.positions.resize(size);
        point_.velocities.resize(size);
        point_.accelerations.resize(size);
        point_.positions[0]= 0.0;
        point_.positions[1]= 0.0;
        point_.positions[2]= 0.0;
        point_.positions[3]= 0.0;
        point_.positions[4]= 0.0;
        point_.positions[5]= 0.0;
        point_.positions[6]= 0.0;
        for (int i = 0; i < size; i ++){
            point_.velocities[i] = 0.0;
            point_.accelerations[i] = 0.0;
        }
        traj_.points.push_back(point_);
    }
    else{   
        traj_sub_ = nh_.subscribe("/cartesian_controller/trajectory/", 1000, &JointController::trajReceived, this);
    }
}

JointController::~JointController(){
}

void JointController::updateTrajectory(){
    goal_.trajectory.points = traj_.points;
}

void JointController::sendCommand(){
    goal_.trajectory.header.stamp = ros::Time::now();
    this->updateTrajectory();
    trajectory_action_.sendGoal(goal_);
    // ROS_INFO("Sent goal");

}

void JointController::trajReceived(const trajectory_msgs::JointTrajectory::ConstPtr& traj){
    ROS_INFO_STREAM("Trajectory received");
    traj_ = *traj;
}

actionlib::SimpleClientGoalState JointController::getState(){
    return trajectory_action_.getState();
}

int main(int argc, char **argv){
    ros::init(argc, argv, "JointController");
    ros::NodeHandle nh;
    JointController jc(nh);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        jc.sendCommand();
        ros::spinOnce();
        while (!jc.getState().isDone()){loop_rate.sleep();}
        loop_rate.sleep();
    }

    return 0;
}
