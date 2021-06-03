#include "ros/ros.h"

#include <iostream>

#include "control_msgs/JointTrajectoryControllerState.h" //read state of the arm
#include "control_msgs/FollowJointTrajectoryAction.h" //set goal positions

#include "std_msgs/String.h"
#include <sensor_msgs/Joy.h>
#include <stdlib.h>


using namespace std;


class Control{
       
    public:
        Control(ros::NodeHandle& nh_);
            
    private:
        void MoveArm(const sensor_msgs::Joy::ConstPtr& joy);
        void getCurrState(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

        ros::Publisher arm_pub;
        ros::Subscriber subRobot;
        ros::Subscriber joy_sub_;
        ros::NodeHandle nHandle;
		
        std::vector<double> currentPos;
        int wJoint;
        double joint_1_new, joint_2_new, joint_3_new, joint_4_new, joint_5_new, joint_6_new;


};


Control::Control(ros::NodeHandle& nh_)
{
    nHandle = nh_;
    wJoint = 1;
    currentPos = {0,0,0,0,0,0};

    arm_pub = nHandle.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/scaled_pos_joint_traj_controller/follow_joint_trajectory/goal", 1000);
    subRobot = nHandle.subscribe("/arm_controller/scaled_pos_joint_traj_controller/state", 1, &Control::getCurrState, this);

}

/// read state of the robot
void Control::getCurrState(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg){

     for (size_t jointNo=0; jointNo<6;jointNo++){
        currentPos[jointNo] = msg->actual.positions[jointNo];
    }

    joy_sub_ = nHandle.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &Control::MoveArm, this);

}

void Control::MoveArm(const sensor_msgs::Joy::ConstPtr& joy) {

    //variables
    joint_1_new = 0.0;
    joint_2_new = 0.0;
    joint_3_new = 0.0;
    joint_4_new = 0.0;
    joint_5_new = 0.0;
    joint_6_new = 0.0;

    control_msgs::FollowJointTrajectoryActionGoal ctrlMsg;
    ctrlMsg.goal.trajectory.joint_names.push_back("right_arm_shoulder_pan_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("right_arm_shoulder_lift_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("right_arm_elbow_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("right_arm_wrist_1_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("right_arm_wrist_2_joint");
    ctrlMsg.goal.trajectory.joint_names.push_back("right_arm_wrist_3_joint");

    ctrlMsg.goal.trajectory.points.resize(1);

    ctrlMsg.goal.trajectory.points[0].positions.resize(6);

    //decide which joint is moving
    if(joy->buttons[0] == 1){
        sleep(0.3); 
        wJoint++;
        if(wJoint == 7){
            wJoint = 1;
        }
    }
    else if(joy->buttons[1] == 1){
        sleep(0.3);
        wJoint = wJoint - 1;
        if(wJoint == 0){
            wJoint = 6;
        }
    }

    //filling variables with data
    if(wJoint == 1){
        joint_1_new = joy->axes[0];
        joint_2_new = 0.0;
        joint_3_new = 0.0;
        joint_4_new = 0.0;
        joint_5_new = 0.0;
        joint_6_new = 0.0;
    }
    else if(wJoint == 2){
        joint_2_new = joy->axes[0];
        joint_1_new = 0.0;
        joint_3_new = 0.0;
        joint_4_new = 0.0;
        joint_5_new = 0.0;
        joint_6_new = 0.0;
    }
    else if(wJoint == 3){
        joint_3_new = joy->axes[0];
        joint_2_new = 0.0;
        joint_1_new = 0.0;
        joint_4_new = 0.0;
        joint_5_new = 0.0;
        joint_6_new = 0.0;
    }
    else if(wJoint == 4){
        joint_4_new = joy->axes[0];
        joint_2_new = 0.0;
        joint_3_new = 0.0;
        joint_1_new = 0.0;
        joint_5_new = 0.0;
        joint_6_new = 0.0;
    }
    else if(wJoint == 5){
        joint_5_new = joy->axes[0];
        joint_2_new = 0.0;
        joint_3_new = 0.0;
        joint_4_new = 0.0;
        joint_1_new = 0.0;
        joint_6_new = 0.0;
    }
    else if(wJoint == 6){
        joint_6_new = joy->axes[0];
        joint_2_new = 0.0;
        joint_3_new = 0.0;
        joint_4_new = 0.0;
        joint_5_new = 0.0;
        joint_1_new = 0.0;
    }
    else{
        std::cout << "Error: there's only 6 joints!" << std::endl;
    }
    
    // new positions for joints
    ctrlMsg.goal.trajectory.points[0].positions[0]= currentPos[0] + joint_1_new;
    ctrlMsg.goal.trajectory.points[0].positions[1]= currentPos[1] + joint_2_new;
    ctrlMsg.goal.trajectory.points[0].positions[2]= currentPos[2] + joint_3_new;
    ctrlMsg.goal.trajectory.points[0].positions[3]= currentPos[3] + joint_4_new;
    ctrlMsg.goal.trajectory.points[0].positions[4]= currentPos[4] + joint_5_new;
    ctrlMsg.goal.trajectory.points[0].positions[5]= currentPos[5] + joint_6_new;

    ctrlMsg.goal.trajectory.points[0].velocities.resize(6);
    for (size_t jointNo=0; jointNo<6;jointNo++)
        ctrlMsg.goal.trajectory.points[0].velocities[jointNo] = 0.0;

    ctrlMsg.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);

    arm_pub.publish(ctrlMsg);

}

int main(int argc, char **argv) {
    //initialize node
    ros::init(argc, argv, "control");
    // node handler
    ros::NodeHandle nh;
    
    Control control(nh);

    //ros loop
    ros::spin();
    return 0;
}
