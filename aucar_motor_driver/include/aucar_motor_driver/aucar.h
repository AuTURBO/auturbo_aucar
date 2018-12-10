#ifndef SWERVE_DRIVER_H_
#define SWERVE_DRIVER_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define LINEAR_VELOCITY  0.3
#define ANGULAR_VELOCITY 1.5

#define BODY_LENGTH       25.6
#define SPEED_ADD_ON      2
#define PI                3.14

class SwerveDriver
{
 public:
  SwerveDriver();
  ~SwerveDriver();
  bool init();
  bool controlLoop();

  double distance_from_center = 25.6;

  int wheel_l_r_vel, wheel_r_r_vel, wheel_l_f_vel, wheel_r_f_vel;
  int joint_l_r_angle, joint_r_r_angle, joint_l_f_angle, joint_r_f_angle;

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher l_r_vel_pub_;
  ros::Publisher r_r_vel_pub_;
  ros::Publisher l_f_vel_pub_;
  ros::Publisher r_f_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber odom_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  int velocity = 200;
  int wheel_vel[4] = {0, };
  int joint_angle[4] = {0, };

  enum MODE
  {
    NONE = 0
    , FORWARD, BACKWARD, RLEFT, RRIGHT, PLEFT, PRIGHT
    , PLEFT_FORWARD, PRIGHT_FORWARD, PLEFT_BACKWARD, PRIGHT_BACKWARD, RLEFT_FORWARD, RRIGHT_FORWARD, RLEFT_BACKWARD, RRIGHT_BACKWARD, RLEFT_PLEFT, RLEFT_PRIGHT, RRIGHT_PLEFT, RRIGHT_PRIGHT
    , RLEFT_PLEFT_FORWARD, RLEFT_PLEFT_BACKWARD, RRIGHT_PLEFT_FORWARD, RRIGHT_PLEFT_BACKWARD, RLEFT_PRIGHT_FORWARD, RLEFT_PRIGHT_BACKWARD, RRIGHT_PRIGHT_FORWARD, RRIGHT_PRIGHT_BACKWARD
  } mobile_mode;

  double scan_data_[3] = {0.0, 0.0, 0.0};

  double tb3_pose_;
  double prev_tb3_pose_;

  // Function prototypes
  void updatecommandVelocity(int linear[4], int angular[4]);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // TURTLEBOT3_DRIVE_H_
