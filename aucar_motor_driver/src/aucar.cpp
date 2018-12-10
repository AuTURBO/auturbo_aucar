#include "aucar_gazebo/aucar.h"

SwerveDriver::SwerveDriver()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("Swerve Driver Simulation Node Init");
  ROS_ASSERT(init());
}

SwerveDriver::~SwerveDriver()
{
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool SwerveDriver::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  l_r_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("l_r_vel", 10);
  r_r_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("r_r_vel", 10);
  l_f_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("l_f_vel", 10);
  r_f_vel_pub_   = nh_.advertise<geometry_msgs::Twist>("r_f_vel", 10);

  // initialize subscribers
  odom_sub_ = nh_.subscribe("odom", 10, &SwerveDriver::odomMsgCallBack, this);

  return true;
}

void SwerveDriver::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

	tb3_pose_ = atan2(siny, cosy);
}


void SwerveDriver::updatecommandVelocity(int linear[4], int angular[4])
{
  geometry_msgs::Twist l_r_vel;
  geometry_msgs::Twist r_r_vel;
  geometry_msgs::Twist l_f_vel;
  geometry_msgs::Twist r_f_vel;

  l_r_vel.linear.x  = linear[0];
  l_r_vel.angular.z = angular[0];

  r_r_vel.linear.x  = linear[1];
  r_r_vel.angular.z = angular[1];

  l_f_vel.linear.x  = linear[2];
  l_f_vel.angular.z = angular[2];

  r_f_vel.linear.x  = linear[3];
  r_f_vel.angular.z = angular[3];

  l_r_vel_pub_.publish(l_r_vel);
  r_r_vel_pub_.publish(r_r_vel);
  l_f_vel_pub_.publish(l_f_vel);
  r_f_vel_pub_.publish(r_f_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool SwerveDriver::controlLoop()
{
  static uint8_t turtlebot3_state_num = 1;

  switch(turtlebot3_state_num)
  {
    case FORWARD:                wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = 512; joint_r_r_angle = -512; joint_l_f_angle = -512; joint_r_f_angle = 512;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                // updatecommandVelocity(*wheel_vel[0], *joint_angle[0]);
                                 break;
    case BACKWARD:               wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = 512; joint_r_r_angle = -512; joint_l_f_angle = -512; joint_r_f_angle = 512;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT:                  wheel_l_r_vel = velocity; wheel_r_r_vel = velocity; wheel_l_f_vel = velocity; wheel_r_f_vel = velocity;
                                 joint_l_r_angle = 0; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT:                 wheel_l_r_vel = -velocity; wheel_r_r_vel = -velocity; wheel_l_f_vel = -velocity; wheel_r_f_vel = -velocity;
                                 joint_l_r_angle = 0; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case PLEFT:                  wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = -512; joint_r_r_angle = 512; joint_l_f_angle = 512; joint_r_f_angle = -512;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case PRIGHT:                 wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = -512; joint_r_r_angle = 512; joint_l_f_angle = 512; joint_r_f_angle = -512;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;

    case PLEFT_FORWARD:          wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = 0; joint_r_r_angle = 1024; joint_l_f_angle = 1024; joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case PRIGHT_FORWARD:         wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = -1024; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = -1024;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case PLEFT_BACKWARD:         wheel_l_r_vel = -velocity * SPEED_ADD_ON; wheel_r_r_vel = -velocity * SPEED_ADD_ON; wheel_l_f_vel = velocity * SPEED_ADD_ON; wheel_r_f_vel = velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = -1024; joint_r_r_angle = 0; joint_l_f_angle = 0; joint_r_f_angle = -1024;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case PRIGHT_BACKWARD:        wheel_l_r_vel = velocity * SPEED_ADD_ON; wheel_r_r_vel = velocity * SPEED_ADD_ON; wheel_l_f_vel = -velocity * SPEED_ADD_ON; wheel_r_f_vel = -velocity * SPEED_ADD_ON;
                                 joint_l_r_angle = 0; joint_r_r_angle = 1024; joint_l_f_angle = 1024; joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;

    case RLEFT_FORWARD:          wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_FORWARD:         wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT_BACKWARD:         wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_BACKWARD:        wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT_PLEFT:            wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT_PRIGHT:           wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_PLEFT:           wheel_l_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_PRIGHT:          wheel_l_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_r_r_vel = (int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan(distance_from_center / (BODY_LENGTH / 2.0)))));
                                 wheel_l_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 wheel_r_f_vel = -(int)((velocity * (BODY_LENGTH / 2.0)) / ((distance_from_center + (BODY_LENGTH / 2.0)) * cos(atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)))));
                                 joint_l_r_angle = -(1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_r_angle = (1536 - (int)(2048.0 * atan(distance_from_center / (BODY_LENGTH / 2.0)) / PI));
                                 joint_l_f_angle = -(512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 joint_r_f_angle = (512 - (int)(2048.0 * atan((distance_from_center + BODY_LENGTH) / (BODY_LENGTH / 2.0)) / PI));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;

    case RLEFT_PLEFT_FORWARD:    wheel_l_r_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_l_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_f_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 joint_l_r_angle = 0;
                                 joint_r_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_l_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT_PLEFT_BACKWARD:   wheel_l_r_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_l_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_f_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 joint_l_r_angle = 0;
                                 joint_r_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_l_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_PLEFT_FORWARD:   wheel_l_r_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_l_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_f_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 joint_l_r_angle = 0;
                                 joint_r_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_l_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_PLEFT_BACKWARD:  wheel_l_r_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_l_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_f_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 joint_l_r_angle = 0;
                                 joint_r_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_l_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT_PRIGHT_FORWARD:   wheel_l_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_r_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_l_f_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 joint_l_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_r_angle = 0;
                                 joint_l_f_angle = 0;
                                 joint_r_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RLEFT_PRIGHT_BACKWARD:  wheel_l_r_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_l_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_f_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 joint_l_r_angle = 0;
                                 joint_r_r_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_l_f_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_PRIGHT_FORWARD:  wheel_l_r_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_r_vel = (int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_l_f_vel = -(int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_f_vel = -(int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 joint_l_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_r_angle = 0;
                                 joint_l_f_angle = 0;
                                 joint_r_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;
    case RRIGHT_PRIGHT_BACKWARD: wheel_l_r_vel = (int)(velocity * (distance_from_center + BODY_LENGTH * sqrt(2.0)) / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 wheel_r_r_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_l_f_vel = (int)(velocity * (BODY_LENGTH / sqrt(2.0)) / (cos(atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0))) * (distance_from_center + (BODY_LENGTH / sqrt(2.0)))));
                                 wheel_r_f_vel = -(int)(velocity * distance_from_center / (distance_from_center + (BODY_LENGTH / sqrt(2.0))));
                                 joint_l_r_angle = 0;
                                 joint_r_r_angle = (2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_l_f_angle = -(2048 - (int)(2048.0 * atan((distance_from_center / (BODY_LENGTH / sqrt(2.0)) + 1.0) / PI)));
                                 joint_r_f_angle = 0;
                                 wheel_vel[0] = wheel_l_r_vel; wheel_vel[1] = wheel_r_r_vel; wheel_vel[2] = wheel_l_f_vel; wheel_vel[3] = wheel_r_f_vel;
                                 joint_angle[0] = joint_l_r_angle; joint_angle[1] = joint_r_r_angle; joint_angle[2] = joint_l_f_angle; joint_angle[3] = joint_r_f_angle;
                                 break;

    default:                     wheel_l_r_vel = 0; wheel_r_r_vel = 0; wheel_l_f_vel = 0; wheel_r_f_vel = 0;
                                 break;
  }
  return true;
}

/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "swerve_driver");
  SwerveDriver swerve_driver;

  ros::Rate loop_rate(125);

  while (ros::ok())
  {
    swerve_driver.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
