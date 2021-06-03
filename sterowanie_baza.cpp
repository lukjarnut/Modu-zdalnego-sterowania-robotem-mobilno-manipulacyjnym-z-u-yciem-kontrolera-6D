#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

class TeleopSpacenav
{
public:
  TeleopSpacenav();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher cmd_vel_pub_;
  ros::Subscriber joy_sub_;

};


TeleopSpacenav::TeleopSpacenav()
{

  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy", 10, &TeleopSpacenav::joyCallback, this);

}

void TeleopSpacenav::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist cmd_vel_msg;
  double prescaler = 0.25;

  //linear x on joy to linear x on robot
  cmd_vel_msg.linear.x = prescaler*joy->axes[0]; 
  //angular z on joy to angular z on robot
  cmd_vel_msg.angular.z = prescaler*joy->axes[5]; 
  

  cmd_vel_pub_.publish(cmd_vel_msg);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_spacenav");
  TeleopSpacenav teleop_spacenav;

  ros::spin();
}
