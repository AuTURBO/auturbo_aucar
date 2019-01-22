  void AuCarDrive::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // load parameters
    robot_namespace_ = _sdf->Get<std::string>("robotNamespace", "").first;
    command_velocity_topic_ = _sdf->Get<std::string>("commandVelocityTopic", "cmd_vel").first;
    odometry_topic_ = _sdf->Get<std::string>("odometryTopic", "odom").first;
    odometry_frame_ = _sdf->Get<std::string>("odometryFrame", "odom").first;
    robot_base_frame_ = _sdf->Get<std::string>("robotBaseFrame", "base_footprint").first;
    does_publish_odometry_ = _sdf->Get<bool>("doesPublishOdometry", false).first;
    does_publish_wheel_tf_ = _sdf->Get<bool>("doesPublishWheelTF", false).first;
    does_publish_wheel_joint_state_ = _sdf->Get<bool>("doesPublishWheelJointState", false).first;
    update_rate_ = _sdf->Get<double>("updateRate", 100.0).first;
    wheel_separation_ = _sdf->Get<double>("wheelSeparation", 0.34).first;
    wheel_diameter_ = _sdf->Get<double>("wheelDiameter", 0.15).first;
    wheel_accel = _sdf->Get<double>("wheelAcceleration", 0.0).first;
    wheel_torque = _sdf->Get<double>("wheelTorque", 5.0).first;

    // if (_sdf->HasElement("baseLinkName"))
    // link_name_ = _sdf->Get<std::string>("baseLinkName", "base_link").first;
    // link = _model->GetLink(link_name_);

    // std::map<std::string, OdomSource> odomOptions;
    // odomOptions["encoder"] = ENCODER;
    // odomOptions["world"] = WORLD;
    // gazebo_ros_->getParameter<OdomSource>(odom_source_, "odometrySource", odomOptions, WORLD);


    // update rate
    if (update_rate_ > 0.0)
    {
      update_period_ = 1.0 / update_rate_;
    }
    else
    {
      update_period_ = 0.0; 
    }
    world = _model->GetWorld();
    last_update_time_ = world->SimTime();    

    // Initialize velocity
    wheel_speed_[STEER_LEFT_FRONT] = 0;
    wheel_speed_[STEER_RIGHT_FRONT] = 0;
    wheel_speed_[STEER_LEFT_REAR] = 0;
    wheel_speed_[STEER_RIGHT_REAR] = 0;
    // wheel_speed_[WHEEL_LEFT_FRONT] = 0;
    // wheel_speed_[WHEEL_RIGHT_FRONT] = 0;
    // wheel_speed_[WHEEL_LEFT_FRONT] = 0;
    // wheel_speed_[WHEEL_RIGHT_FRONT] = 0;

    // Initialize velocity support
    wheel_speed_instr_[STEER_LEFT_FRONT] = 0;
    wheel_speed_instr_[STEER_RIGHT_FRONT] = 0;
    wheel_speed_instr_[STEER_LEFT_REAR] = 0;
    wheel_speed_instr_[STEER_RIGHT_REAR] = 0;
    // wheel_speed_instr_[WHEEL_LEFT_FRONT] = 0;
    // wheel_speed_instr_[WHEEL_RIGHT_FRONT] = 0;
    // wheel_speed_instr_[WHEEL_LEFT_FRONT] = 0;
    // wheel_speed_instr_[WHEEL_RIGHT_FRONT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;

    fnInitNode();
  }

  void AuCarDrive::fnInitNode()
  {
    rosnode_ = new ros::NodeHandle(robot_namespace_);
    rosnode_->setCallbackQueue(&queue_);
    tf_prefix_ = tf::getPrefixParam(*rosnode_);

    if (does_publish_wheel_joint_state_)
    {
      oPubJointState = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1000);
    }

    // transform_broadcaster_ = new tf::TransformBroadcaster();
    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    if (does_publish_odometry_)
    {
      oPubOdom = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    }

    oSubCmdVel = rosnode_->subscribe(so);

    callback_queue_thread_ = boost::thread(boost::bind(&AuCarDrive::fnQueueThread, this));

    // Reset();
    
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AuCarDrive::Update, this));
  }

  void AuCarDrive::Reset()
  {
    last_update_time_ = world->SimTime();
    // enableMotors = true;

    // for (size_t i = 0; i < 2; ++i){
    //   wheel_speed_[i] = 0;
    // }

    // last_update_time_ = world->SimTime();

    // x_ = 0;
    // rot_ = 0;
    // alive_ = true;

    // // Reset odometric pose
    // odomPose[0] = 0.0;
    // odomPose[1] = 0.0;
    // odomPose[2] = 0.0;

    // odomVel[0] = 0.0;
    // odomVel[1] = 0.0;
    // odomVel[2] = 0.0;
  }

  void AuCarDrive::Update()
  {
    for (int i = 0; i < NUM_JOINTS; i++)
    {
      if (fabs(wheel_torque - joints_[i]->GetForce(0)) > 1e-6)
        joints_[i]->SetForce(0, wheel_torque);
    }

    common::Time current_time = world->SimTime();
    double seconds_since_last_update = (current_time - last_update_time_).Double();

    if (seconds_since_last_update > update_period_)
    {
    // std::cout << "current_time  : " << current_time << std::endl;
    // std::cout << "last_update_time_  : " << last_update_time_ << std::endl;
    // std::cout << update_period_ << std::endl;
      // if (does_publish_odometry_)
      
      // if (does_publish_wheel_tf_)

      if (does_publish_wheel_joint_state_)
        fnPubWheelJointState();

      joints_[WHEEL_LEFT_FRONT]->SetVelocity(0, 40.0);
      joints_[WHEEL_RIGHT_FRONT]->SetVelocity(0, 40.0);

    std::cout << x_ << "  " << rot_ << std::endl;


      last_update_time_ += common::Time(update_period_);
    }
  }

  void AuCarDrive::fnPubWheelJointState()
  {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize(joints_.size());
    joint_state_.position.resize(joints_.size());

    for ( int i = 0; i < NUM_JOINTS; i++ )
    {
      // physics::JointPtr joint = joints_[i];
      auto angle = joints_[0]->Position(0);
      // std::cout << angle << std::endl;
      joint_state_.name[i] = joints_[i]->GetName();
      joint_state_.position[i] = angle; // .toRadian()
      // auto angle = joint->GetAngle(0);

    }
    oPubJointState.publish(joint_state_);
  }

  void AuCarDrive::GetPositionCmd()
  {
    lock.lock();

    double vr, va;

    vr = x_; //myIface->data->cmdVelocity.pos.x;
    va = -rot_; //myIface->data->cmdVelocity.yaw;

    //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

    // Changed motors to be always on, which is probably what we want anyway
    enableMotors = true; //myIface->data->cmdEnableMotors > 0;

    //std::cout << enableMotors << std::endl;

    wheel_speed_[0] = vr + va * wheel_separation_ / 2;
    wheel_speed_[1] = vr - va * wheel_separation_ / 2;

    lock.unlock();
  }

  void AuCarDrive::publish_odometry()
  {
    // get current time
  #if (GAZEBO_MAJOR_VERSION >= 8)
    ros::Time current_time_((world->SimTime()).sec, (world->SimTime()).nsec); 
  #else
    ros::Time current_time_((world->GetSimTime()).sec, (world->GetSimTime()).nsec); 
  #endif

    // getting data for base_footprint to odom transform
  #if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = link->WorldPose();
    ignition::math::Vector3d velocity = link->WorldLinearVel();
    ignition::math::Vector3d angular_velocity = link->WorldAngularVel();

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
  #else
    math::Pose pose = link->GetWorldPose();
    math::Vector3 velocity = link->GetWorldLinearVel();
    math::Vector3 angular_velocity = link->GetWorldAngularVel();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
  #endif
    tf::Transform base_footprint_to_odom(qt, vt);

    transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                              current_time_,
                                                              "odom",
                                                              "base_footprint"));

    // publish odom topic
  #if (GAZEBO_MAJOR_VERSION >= 8)
    odom_.pose.pose.position.x = pose.Pos().X();
    odom_.pose.pose.position.y = pose.Pos().Y();

    odom_.pose.pose.orientation.x = pose.Rot().X();
    odom_.pose.pose.orientation.y = pose.Rot().Y();
    odom_.pose.pose.orientation.z = pose.Rot().Z();
    odom_.pose.pose.orientation.w = pose.Rot().W();

    odom_.twist.twist.linear.x = velocity.X();
    odom_.twist.twist.linear.y = velocity.Y();
    odom_.twist.twist.angular.z = angular_velocity.Z();
  #else
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;

    odom_.twist.twist.linear.x = velocity.x;
    odom_.twist.twist.linear.y = velocity.y;
    odom_.twist.twist.angular.z = angular_velocity.z;
  #endif

    odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
    odom_.child_frame_id = "base_footprint";
    odom_.header.stamp = current_time_;

    oPubOdom.publish(odom_);
  }

  void AuCarDrive::fnQueueThread()
  {
    static const double timeout = 0.01;

    // while (this->rosNode->ok())
    // {
    //   std::cout << "CALLING STUFF\n";
    //   // this->rosQueue.callAvailable(ros::WallDuration(timeout));
    // }

    while (alive_ && rosnode_->ok())
    {
      std::cout << "CALLING STUFF\n";
      // queue_.callAvailable(ros::WallDuration(timeout));
      // queue_.
    }
  }

  void AuCarDrive::cbCmdVel(const geometry_msgs::Twist::ConstPtr& cmd_vel)
  {
    lock.lock();
    // boost::mutex::scoped_lock scoped_lock ( lock );
    x_ = cmd_vel->linear.x;
    rot_ = cmd_vel->angular.z;

    lock.unlock();
  }

  GZ_REGISTER_MODEL_PLUGIN(AuCarDrive);
}

// #include <algorithm>
// #include <assert.h>

// #include "aucar_drive.hpp"
// #include <gazebo/common/Events.hh>
// #include <gazebo/physics/physics.hh>

// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <boost/bind.hpp>

// #include <gazebo/gazebo_config.h>

// namespace gazebo {

// AuCarDrive::~AuCarDrive()
// {
// // #if (GAZEBO_MAJOR_VERSION >= 8)
// //   update_connection_.reset();
// // #else
// //   event::Events::DisconnectWorldUpdateBegin(update_connection_);
// // #endif
// //   delete transform_broadcaster_;
// //   rosnode_->shutdown();
// //   callback_queue_thread_.join();
// //   delete rosnode_;
// }

  
//   // std::map<std::string, OdomSource> odomOptions;
//   // odomOptions["encoder"] = ENCODER;
//   // odomOptions["world"] = WORLD;
//   // gazebo_ros_->getParameter<OdomSource>(odom_source_, "odometrySource", odomOptions, WORLD);









// }


// // Update the controller
// void AuCarDrive::Update()
// {
// //   // TODO: Step should be in a parameter of this function
// //   double d1, d2;
// //   double dr, da;
// //   common::Time stepTime;

// //   GetPositionCmd();

// //   //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
// // #if (GAZEBO_MAJOR_VERSION >= 8)
// //   stepTime = world->SimTime() - prevUpdateTime;
// //   prevUpdateTime = world->SimTime();
// // #else
// //   stepTime = world->GetSimTime() - prevUpdateTime;
// //   prevUpdateTime = world->GetSimTime();
// // #endif

// //   // Distance travelled by front wheels
// //   d1 = stepTime.Double() * wheelDiam / 2 * joints[WHEEL_LEFT_REAR]->GetVelocity(0);
// //   d2 = stepTime.Double() * wheelDiam / 2 * joints[WHEEL_RIGHT_REAR]->GetVelocity(0);

// //   dr = (d1 + d2) / 2;
// //   da = (d1 - d2) / wheelSep;

// //   // Compute odometric pose
// //   odomPose[0] += dr * cos(odomPose[2]);
// //   odomPose[1] += dr * sin(odomPose[2]);
// //   odomPose[2] += da;

// //   // Compute odometric instantaneous velocity
// //   odomVel[0] = dr / stepTime.Double();
// //   odomVel[1] = 0.0;
// //   odomVel[2] = da / stepTime.Double();

// //   if (enableMotors)
// //   {
// //     joints[WHEEL_LEFT_FRONT]->SetVelocity(0, wheelSpeed[0] / (wheelDiam / 2.0));
// //     joints[WHEEL_LEFT_REAR]->SetVelocity(0, wheelSpeed[0] / (wheelDiam / 2.0));
// //     joints[STEER_LEFT_FRONT]->SetVelocity(0, wheelSpeed[0] / (wheelDiam / 2.0));

// //     joints[WHEEL_RIGHT_FRONT]->SetVelocity(0, wheelSpeed[1] / (wheelDiam / 2.0));
// //     joints[WHEEL_RIGHT_REAR]->SetVelocity(0, wheelSpeed[1] / (wheelDiam / 2.0));
// //     joints[STEER_RIGHT_FRONT]->SetVelocity(0, wheelSpeed[1] / (wheelDiam / 2.0));

// // #if (GAZEBO_MAJOR_VERSION > 4)
// //     joints[WHEEL_LEFT_FRONT]->SetEffortLimit(0, torque);
// //     joints[WHEEL_LEFT_REAR]->SetEffortLimit(0, torque);
// //     joints[STEER_LEFT_FRONT]->SetEffortLimit(0, torque);

// //     joints[WHEEL_RIGHT_FRONT]->SetEffortLimit(0, torque);
// //     joints[WHEEL_RIGHT_REAR]->SetEffortLimit(0, torque);
// //     joints[STEER_RIGHT_FRONT]->SetEffortLimit(0, torque);
// // #else
// //     joints[WHEEL_LEFT_FRONT]->SetMaxForce(0, torque);
// //     joints[WHEEL_LEFT_REAR]->SetMaxForce(0, torque);
// //     joints[STEER_LEFT_FRONT]->SetMaxForce(0, torque);

// //     joints[WHEEL_RIGHT_FRONT]->SetMaxForce(0, torque);
// //     joints[WHEEL_RIGHT_REAR]->SetMaxForce(0, torque);
// //     joints[STEER_RIGHT_FRONT]->SetMaxForce(0, torque);
// // #endif
// //   }

// //   //publish_odometry();
// }

// // NEW: Now uses the target velocities from the ROS message, not the Iface 
// void AuCarDrive::GetPositionCmd()
// {
//   // lock.lock();

//   // double vr, va;

//   // vr = x_; //myIface->data->cmdVelocity.pos.x;
//   // va = -rot_; //myIface->data->cmdVelocity.yaw;

//   // //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

//   // // Changed motors to be always on, which is probably what we want anyway
//   // enableMotors = true; //myIface->data->cmdEnableMotors > 0;

//   // //std::cout << enableMotors << std::endl;

//   // wheelSpeed[0] = vr + va * wheelSep / 2;
//   // wheelSpeed[1] = vr - va * wheelSep / 2;

//   // lock.unlock();
// }


// // NEW: custom callback queue thread
// void AuCarDrive::QueueThread()
// {
//   // static const double timeout = 0.01;

//   // while (alive_ && rosnode_->ok())
//   // {
//   //   //    std::cout << "CALLING STUFF\n";
//   //   queue_.callAvailable(ros::WallDuration(timeout));
//   // }
// }

// // NEW: Update this to publish odometry topic
// void AuCarDrive::publish_odometry()
// {
// //   // get current time
// // #if (GAZEBO_MAJOR_VERSION >= 8)
// //   ros::Time current_time_((world->SimTime()).sec, (world->SimTime()).nsec); 
// // #else
// //   ros::Time current_time_((world->GetSimTime()).sec, (world->GetSimTime()).nsec); 
// // #endif

// //   // getting data for base_footprint to odom transform
// // #if (GAZEBO_MAJOR_VERSION >= 8)
// //   ignition::math::Pose3d pose = link->WorldPose();
// //   ignition::math::Vector3d velocity = link->WorldLinearVel();
// //   ignition::math::Vector3d angular_velocity = link->WorldAngularVel();

// //   tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
// //   tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
// // #else
// //   math::Pose pose = link->GetWorldPose();
// //   math::Vector3 velocity = link->GetWorldLinearVel();
// //   math::Vector3 angular_velocity = link->GetWorldAngularVel();

// //   tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
// //   tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
// // #endif
// //   tf::Transform base_footprint_to_odom(qt, vt);

// //   transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
// //                                                             current_time_,
// //                                                             "odom",
// //                                                             "base_footprint"));

// //   // publish odom topic
// // #if (GAZEBO_MAJOR_VERSION >= 8)
// //   odom_.pose.pose.position.x = pose.Pos().X();
// //   odom_.pose.pose.position.y = pose.Pos().Y();

// //   odom_.pose.pose.orientation.x = pose.Rot().X();
// //   odom_.pose.pose.orientation.y = pose.Rot().Y();
// //   odom_.pose.pose.orientation.z = pose.Rot().Z();
// //   odom_.pose.pose.orientation.w = pose.Rot().W();

// //   odom_.twist.twist.linear.x = velocity.X();
// //   odom_.twist.twist.linear.y = velocity.Y();
// //   odom_.twist.twist.angular.z = angular_velocity.Z();
// // #else
// //   odom_.pose.pose.position.x = pose.pos.x;
// //   odom_.pose.pose.position.y = pose.pos.y;

// //   odom_.pose.pose.orientation.x = pose.rot.x;
// //   odom_.pose.pose.orientation.y = pose.rot.y;
// //   odom_.pose.pose.orientation.z = pose.rot.z;
// //   odom_.pose.pose.orientation.w = pose.rot.w;

// //   odom_.twist.twist.linear.x = velocity.x;
// //   odom_.twist.twist.linear.y = velocity.y;
// //   odom_.twist.twist.angular.z = angular_velocity.z;
// // #endif

// //   odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
// //   odom_.child_frame_id = "base_footprint";
// //   odom_.header.stamp = current_time_;

// //   pub_.publish(odom_);
// }

// GZ_REGISTER_MODEL_PLUGIN(AuCarDrive)

// } // namespace gazebo
