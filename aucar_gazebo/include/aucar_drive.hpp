// #include <gazebo/gazebo.hh>
// #include <gazebo/common/Plugin.hh>
#include <gazebo-9/gazebo/gazebo.hh>
#include <gazebo-9/gazebo/common/Plugin.hh>
#include <gazebo-9/gazebo/physics/Model.hh>
#include <gazebo-9/gazebo/physics/physics.hh>
// #include <gazebo/common/common.hh>
// #include <gazebo/common/Time.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <thread>

// ROS
#include <ros/ros.h>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Messages
#include "std_msgs/Float32.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/JointState.h>

namespace gazebo
{
class AuCarDrive : public ModelPlugin
{
  enum
  {
    STEER_LEFT_FRONT,
    STEER_RIGHT_FRONT,
    STEER_LEFT_REAR,
    STEER_RIGHT_REAR,  
    WHEEL_LEFT_FRONT,
    WHEEL_RIGHT_FRONT,
    WHEEL_LEFT_REAR,
    WHEEL_RIGHT_REAR,
    NUM_JOINTS
  };

public:
  AuCarDrive();
  virtual ~AuCarDrive();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Update();
  virtual void Reset();

private:
  // void Update();
  void fnInitROSNode();
  void fnInitROSParam();
  void fnInitModelStatus();

  void fnControlJointPosition();
  void fnControlJointVelocity();

  void fnPubWheelJointState();
  void cbCmdVel(const geometry_msgs::TwistConstPtr &_msg);
  
  void fnQueueThread();
  
  physics::ModelPtr model_;
  sdf::ElementPtr sdf_;
  
  // Node Handler
  std::unique_ptr<ros::NodeHandle> nh_;
  
  // Publish
  ros::Publisher oPubOdom;
  ros::Publisher oPubJointState;  
  boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
  // tf::TransformBroadcaster *transform_broadcaster_;

  // Subscribe
  ros::Subscriber oSubCmdVel;

  // Message
  sensor_msgs::JointState joint_state_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  // Thread
  ros::CallbackQueue queue_;
  std::thread callback_queue_thread_;
  event::ConnectionPtr update_connection_;
  boost::mutex lock;

  // Simulation time of the last update
  double update_period_;
  common::Time current_time;
  common::Time last_update_time_;

  // Input Parameters
  std::string robot_namespace_;
  std::string command_velocity_topic_;
  std::string odometry_topic_;
  std::string odometry_frame_;
  std::string robot_base_frame_;
  bool does_publish_odometry_;
  bool does_publish_wheel_tf_;
  bool does_publish_wheel_joint_state_;
  double update_rate_;
  double wheel_separation_;
  double wheel_diameter_;
  double wheel_torque;
  double wheel_accel;

  // Model status
  double wheel_speed_[8];
  double wheel_speed_instr_[8];
  double x_;
  double rot_;
  bool alive_; 

  // Model variables
  // std::vector<physics::JointPtr> joints_;
  physics::JointPtr joints_[8];


};
}