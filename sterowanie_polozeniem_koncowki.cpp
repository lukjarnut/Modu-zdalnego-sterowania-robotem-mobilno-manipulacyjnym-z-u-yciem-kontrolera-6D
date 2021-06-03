#include <iostream>
#include <string>

#include <sensor_msgs/Joy.h>
#include <controller_ur/RG2.h>
#include <controller_ur/RG2_Grip.h>
#include<rg6_service/RG6_grip.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/WrenchStamped.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <control_msgs/JointTrajectoryControllerState.h> //read state of the arm
#include <control_msgs/FollowJointTrajectoryAction.h> //set goal positions
#include <control_msgs/FollowJointTrajectoryActionResult.h> // result
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include "ros/service_client.h"

//MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/ompl_interface/ompl_interface.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <controller_ur/JointsConfig.h>

// pluginlib
#include <pluginlib/class_loader.h>

// Boost
#include <boost/thread.hpp>

// standard
#include <mutex>
#include <iostream>
#include <thread>
#include <iomanip>
#include <fstream>
#include <iostream>

// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


    class TeleopControl{

    public:
      TeleopControl(ros::NodeHandle& nh_, std::string groupName);

    private:
    	void MoveArm(const sensor_msgs::Joy::ConstPtr& joy);
    	ros::Publisher arm_pub;
	  	ros::ServiceClient client;
		ros::Subscriber joy_sub_;
		tf::TransformListener listener;
		tf::StampedTransform endEfFrame;
		ros::NodeHandle& nodeHandle_;
		std::string jointGoalTopic_;
		rg6_service::RG6_grip gripper_width;
		std_msgs::Float64 width;

		std::string planningGroup;
		const robot_state::JointModelGroup* joint_model_group;
		robot_model_loader::RobotModelLoader robot_model_loader;
    	moveit::core::RobotStatePtr kinematic_state;
		robot_model::RobotModelPtr kinematic_model;
    	planning_scene::PlanningScenePtr planningScene;
    	moveit::planning_interface::MoveGroupInterface moveGroup;
    };

TeleopControl::TeleopControl(ros::NodeHandle& nh_, std::string groupName)
	:nodeHandle_(nh_), moveGroup(groupName), robot_model_loader("robot_description")
{
	kinematic_model = robot_model_loader.getModel();

	joy_sub_ = nodeHandle_.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &TeleopControl::MoveArm, this);
	client = nodeHandle_.serviceClient<rg6_service::RG6_grip>("/rg2_gripper/control_width");

}

void TeleopControl::MoveArm(const sensor_msgs::Joy::ConstPtr& joy)
{

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	nodeHandle_.param("/planning_group", planningGroup, std::string("right_arm"));

	joint_model_group = moveGroup.getCurrentState()->getJointModelGroup(planningGroup);

	try {
	listener.waitForTransform("base_link", "right_arm_ee_link", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("base_link", "right_arm_ee_link", ros::Time(0), endEfFrame);
	}
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

	if(joy->buttons[0] == 1){
		width.data = 0;
    	gripper_width.request.target_width = width;
		client.call(gripper_width);
    }
    else if(joy->buttons[1] == 1){
		width.data = 140;
    	gripper_width.request.target_width = width;
		client.call(gripper_width);
    }


	double prescaler_linear = 0.1;
    geometry_msgs::Pose target_pose;

	target_pose.position.x = endEfFrame.getOrigin().x() + prescaler_linear*joy->axes[0];
	target_pose.position.y = endEfFrame.getOrigin().y() + prescaler_linear*joy->axes[1];
	target_pose.position.z = endEfFrame.getOrigin().z() + prescaler_linear*joy->axes[2];


  	//read current joint position
  	double prescaler_angular = -4;
  	double roll, pitch, yaw;
  	tf::Quaternion q_original;
  	q_original = endEfFrame.getRotation();
  	tf::Matrix3x3 R(q_original);
 

  	//calculate new positions
  	tf::Quaternion q_rotation;

  	double roll_joy = prescaler_angular*joy->axes[3]*M_PI/180.0;
  	double pitch_joy = prescaler_angular*joy->axes[4]*M_PI/180.0;
  	double yaw_joy = prescaler_angular*joy->axes[5]*M_PI/180.0;

  	q_rotation.setRPY(roll_joy, pitch_joy, yaw_joy); 
  	tf::Matrix3x3 R1(q_rotation);
  	tf::Matrix3x3 R_new = R * R1;
  	R_new.getRPY(roll, pitch, yaw);
  	tf::Quaternion R_new_quat = tf::createQuaternionFromRPY(roll, pitch, yaw);

  	//set new orientation
  	target_pose.orientation.x = R_new_quat.x();
  	target_pose.orientation.y = R_new_quat.y();
  	target_pose.orientation.z = R_new_quat.z();
  	target_pose.orientation.w = R_new_quat.w();

	moveGroup.setStartStateToCurrentState();
	moveGroup.setMaxVelocityScalingFactor(0.25);
	moveGroup.setPoseReferenceFrame("base_link");
	moveGroup.setEndEffectorLink("right_arm_ee_link");

	moveGroup.clearPoseTargets();
	moveGroup.setPoseTarget(target_pose);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	moveGroup.setPlanningTime(1);

	bool success = (moveGroup.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

	moveGroup.plan(my_plan);

	//moveGroup.execute(my_plan);

	moveGroup.move();
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "teleop_control");

	ros::NodeHandle nh_;

	ros::AsyncSpinner spinner(2);
	spinner.start();

	std::string planningGroup;
	nh_.param("/planning_group", planningGroup, std::string("right_arm"));
	
	TeleopControl teleopControl(nh_, planningGroup);

	ros::waitForShutdown();

	return 0;
}
