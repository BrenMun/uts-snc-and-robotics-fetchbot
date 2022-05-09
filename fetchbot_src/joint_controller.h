#ifndef JOINT_CONTROLLER_H
#define JOINT_CONTROLLER_H

#include <ros/ros.h>
#include <math.h>


#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>



class JointController{
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_action_;
        ros::Subscriber traj_sub_;

        trajectory_msgs::JointTrajectory traj_;
        trajectory_msgs::JointTrajectoryPoint point_;

        control_msgs::FollowJointTrajectoryGoal goal_;
    public:
        JointController(ros::NodeHandle&);
        JointController(ros::NodeHandle&, bool);
        ~JointController();
        void trajReceived(const trajectory_msgs::JointTrajectory::ConstPtr&);

        actionlib::SimpleClientGoalState getState();

        void updateTrajectory();
        void sendCommand();
};

#endif
