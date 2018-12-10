#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh_;

  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize publishers
  ros::Publisher  l_r_vel_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/l_r_velocity_controller/command", 10);
  ros::Publisher  r_r_vel_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/r_r_velocity_controller/command", 10);
  ros::Publisher  l_f_vel_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/l_f_velocity_controller/command", 10);
  ros::Publisher  r_f_vel_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/r_f_velocity_controller/command", 10);

  ros::Publisher  l_r_pos_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/l_r_position_controller/command", 10);
  ros::Publisher  r_r_pos_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/r_r_position_controller/command", 10);
  ros::Publisher  l_f_pos_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/l_f_position_controller/command", 10);
  ros::Publisher  r_f_pos_pub_   = nh_.advertise<std_msgs::Float64>("/swerve_driver/r_f_position_controller/command", 10);


  ros::Rate loop_rate(125);

  while (ros::ok())
    {
      std_msgs::Float64 l_r_vel;
      std_msgs::Float64 r_r_vel;
      std_msgs::Float64 l_f_vel;
      std_msgs::Float64 r_f_vel;

      std_msgs::Float64 l_r_pos;
      std_msgs::Float64 r_r_pos;
      std_msgs::Float64 l_f_pos;
      std_msgs::Float64 r_f_pos;

      l_r_vel.data = -30;
      r_r_vel.data = 30;
      l_f_vel.data = -30;
      r_f_vel.data = 30;

      l_r_pos.data = 0;
      r_r_pos.data = 0;
      l_f_pos.data = 0;
      r_f_pos.data = 0;

      l_r_vel_pub_.publish(l_r_vel);
      r_r_vel_pub_.publish(r_r_vel);
      l_f_vel_pub_.publish(l_f_vel);
      r_f_vel_pub_.publish(r_f_vel);

      l_r_pos_pub_.publish(l_r_pos);
      r_r_pos_pub_.publish(r_r_pos);
      l_f_pos_pub_.publish(l_f_pos);
      r_f_pos_pub_.publish(r_f_pos);
    }
  return 0;
}
