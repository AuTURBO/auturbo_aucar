#include "aucar_drive.hpp"

namespace gazebo
{
  // Constructor
  AuCarDrive::AuCarDrive() {}

  // Destructor
  AuCarDrive::~AuCarDrive() {}

  void AuCarDrive::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    this->model_ = _model;
    this->sdf_ = _sdf;

    fnInitROSParam();

    fnInitModelStatus();

    // fnInitROSNode();
  }

  void AuCarDrive::fnInitROSParam()
  {
    robot_namespace_ = sdf_->Get<std::string>("robotNamespace", "").first;
    command_velocity_topic_ = sdf_->Get<std::string>("commandVelocityTopic", "cmd_vel").first;
    odometry_topic_ = sdf_->Get<std::string>("odometryTopic", "odom").first;
    odometry_frame_ = sdf_->Get<std::string>("odometryFrame", "odom").first;
    robot_base_frame_ = sdf_->Get<std::string>("robotBaseFrame", "base_footprint").first;
    does_publish_odometry_ = sdf_->Get<bool>("doesPublishOdometry", false).first;
    does_publish_wheel_tf_ = sdf_->Get<bool>("doesPublishWheelTF", false).first;
    does_publish_wheel_joint_state_ = sdf_->Get<bool>("doesPublishWheelJointState", false).first;
    update_rate_ = sdf_->Get<double>("updateRate", 100.0).first;
    wheel_separation_ = sdf_->Get<double>("wheelSeparation", 0.34).first;
    wheel_diameter_ = sdf_->Get<double>("wheelDiameter", 0.15).first;
    wheel_accel = sdf_->Get<double>("wheelAcceleration", 0.0).first;
    wheel_torque = sdf_->Get<double>("wheelTorque", 5.0).first;

    // joints_ = physics::JointPtr(new physics::Joint());
    // joints_.resize(NUM_JOINTS);

    // joints_[STEER_LEFT_FRONT] = 
    // joints_[STEER_RIGHT_FRONT]


    joints_[STEER_LEFT_FRONT] = model_->GetJoint(sdf_->Get<std::string>("steerLeftFrontJoint", "steer_left_front_joint").first);  
    joints_[STEER_RIGHT_FRONT] = model_->GetJoint(sdf_->Get<std::string>("steerRightFrontJoint", "steer_right_front_joint").first); 
    joints_[STEER_LEFT_REAR] = model_->GetJoint(sdf_->Get<std::string>("steerLeftRearJoint", "steer_left_rear_joint").first); 
    joints_[STEER_RIGHT_REAR] = model_->GetJoint(sdf_->Get<std::string>("steerRightRearJoint", "steer_right_rear_joint").first);   
    joints_[WHEEL_LEFT_FRONT] = model_->GetJoint(sdf_->Get<std::string>("wheelLeftFrontJoint", "wheel_left_front_joint").first);
    joints_[WHEEL_RIGHT_FRONT] = model_->GetJoint(sdf_->Get<std::string>("wheelRightFrontJoint", "wheel_right_front_joint").first);
    joints_[WHEEL_LEFT_REAR] = model_->GetJoint(sdf_->Get<std::string>("wheelLeftRearJoint", "wheel_left_rear_joint").first); 
    joints_[WHEEL_RIGHT_REAR] = model_->GetJoint(sdf_->Get<std::string>("wheelRightRearJoint", "wheel_right_rear_joint").first);  
  }

  void AuCarDrive::fnInitModelStatus()
  {
    joints_[0]->SetForce(0, 10);
    joints_[1]->SetForce(0, 10);
    // joints_[2]->SetForce(1, 10);
    // joints_[3]->SetForce(0, 10);
    // joints_[4]->SetForce(0, 10);
    // joints_[5]->SetForce(0, 10);
    // joints_[6]->SetForce(0, 10);
    // joints_[7]->SetForce(0, 10);

    // set joint torque
    for (int i = 0; i < NUM_JOINTS; i++)
    {

      // joints_[i]->SetForce(0, wheel_torque);
    }

    // filter update rate
    if (update_rate_ > 0.0)
    {
      update_period_ = 1.0 / update_rate_;
    }
    else
    {
      update_period_ = 0.0; 
    }

    // initialize last update time
    last_update_time_ = model_->GetWorld()->SimTime();

    // Initialize velocity
    // steer_speed_[STEER_LEFT_FRONT] = 0;
    // steer_speed_[STEER_RIGHT_FRONT] = 0;
    // steer_speed_[STEER_LEFT_REAR] = 0;
    // steer_speed_[STEER_RIGHT_REAR] = 0;
    wheel_speed_[WHEEL_LEFT_FRONT] = 0;
    wheel_speed_[WHEEL_RIGHT_FRONT] = 0;
    wheel_speed_[WHEEL_LEFT_FRONT] = 0;
    wheel_speed_[WHEEL_RIGHT_FRONT] = 0;

    // Initialize velocity support
    // steer_speed_instr_[STEER_LEFT_FRONT] = 0;
    // steer_speed_instr_[STEER_RIGHT_FRONT] = 0;
    // steer_speed_instr_[STEER_LEFT_REAR] = 0;
    // steer_speed_instr_[STEER_RIGHT_REAR] = 0;
    // wheel_speed_instr_[WHEEL_LEFT_FRONT] = 0;
    // wheel_speed_instr_[WHEEL_RIGHT_FRONT] = 0;
    // wheel_speed_instr_[WHEEL_LEFT_FRONT] = 0;
    // wheel_speed_instr_[WHEEL_RIGHT_FRONT] = 0;

    x_ = 0;
    rot_ = 0;
    alive_ = true;
  }

  void AuCarDrive::fnInitROSNode()
  {
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "",
          ros::init_options::NoSigintHandler);
    }

    this->nh_.reset(new ros::NodeHandle(""));

    // publisher
    if (does_publish_wheel_joint_state_)
    {
      oPubJointState = nh_->advertise<sensor_msgs::JointState>("joint_states", 1000);
    }

    // subscriber
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(
          "/cmd_vel",
          1,
          boost::bind(&AuCarDrive::cbCmdVel, this, _1),
          ros::VoidPtr(), &this->queue_);
    this->oSubCmdVel = this->nh_->subscribe(so);
    
    this->callback_queue_thread_ =
      std::thread(std::bind(&AuCarDrive::fnQueueThread, this));

    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&AuCarDrive::Update, this));
  }

  void AuCarDrive::Update()
  {    
    // for (int i = 0; i < NUM_JOINTS; i++)
    // {
    //   if (fabs(wheel_torque - joints_[i]->GetForce(0)) > 1e-6)
    //     joints_[i]->SetForce(0, wheel_torque);
    // }

    current_time = model_->GetWorld()->SimTime();
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

      fnControlJointPosition();

      fnControlJointVelocity();
      
      last_update_time_ += common::Time(update_period_);
    }
  }

  void AuCarDrive::Reset()
  {
    fnInitModelStatus();
  }

  void AuCarDrive::fnPubWheelJointState()
  {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    // joint_state_.name.resize(joints_.size());
    // joint_state_.position.resize(joints_.size());

    for (int i = 0; i < NUM_JOINTS; i++ )
    {
      // auto angle = joints_[i]->Position(0);
      // joint_state_.name[i] = joints_[i]->GetName();
      // joint_state_.position[i] = angle; // .toRadian()

    }
    oPubJointState.publish(joint_state_);
  }

  void AuCarDrive::fnControlJointPosition()
  {
  //   double joint_target_pos = 90; 
  //   common::PID jointPID(10, 0, 0, 10, -10);

  //   // compute the steptime for the PID
  //   common::Time currTime = model_->GetWorld()->SimTime();
  //   common::Time stepTime = currTime - last_update_time_;
  //   last_update_time_ = currTime;

  //  // set the current position of the joint, and the target position, 
  //   // and the maximum effort limit
  //   double pos_target = joint_target_pos;
  //   double pos_curr = joints_[STEER_RIGHT_FRONT]->Position(2);
  //   // GetAngle(2).Radian();
  //   double max_cmd = 30;//joint_max_effort;

  //   // calculate the error between the current position and the target one
  //   double pos_err = pos_curr - pos_target;

  //   // compute the effort via the PID, which you will apply on the joint
  //   double effort_cmd = jointPID.Update(pos_err, stepTime);

  //   // check if the effort is larger than the maximum permitted one
  //   effort_cmd = effort_cmd > max_cmd ? max_cmd :
  //               (effort_cmd < -max_cmd ? -max_cmd : effort_cmd);







    // apply the force on the joint
    // joints_[STEER_RIGHT_FRONT]->SetForce(2, effort_cmd);



    // // Get the default PIDs attached to your joints
    // physics::JointControllerPtr jointController = model_->GetJointController();  
    // // In a loop, on your joints
    // // Create a PID
    // // common::PID PID = gazebo::common::PID(p,i,d,imax,imin,cmdMax,cmdMin);
    // common::PID PID = common::PID(10000,0,0,123456,0,123456,0);
    // // Set the PID to your joint (pay attention to joint name and scopedName)
    // jointController->SetPositionPID("steer_left_front_joint",PID);
    // // Set target
    // jointController->SetPositionTarget("steer_left_front_joint", 100);
    // // jointController->SetPosition()

    // // Update the controller
    // jointController->Update();

    // auto m_axis_index = 0;
    // common::PID m_PositionPID;
    // common::Time currTime = this->model_->GetWorld()->GetSimTime();
    // common::Time stepTime = currTime - this->last_update_time_;
    // this->last_update_time_ = currTime;

    // if ( stepTime > 0 ) {
    //     auto m_cmd_force = this->m_PositionPID.Update(
    //                 this->m_JointPtr->GetAngle(m_axis_index).Radian() -
    //                 this->m_setpoint, stepTime);
    //     this->m_JointPtr->SetForce(m_axis_index, m_cmd_force);
    // }

    // joints_[STEER_LEFT_FRONT]->SetPosition(0, 30);
    // joints_[STEER_RIGHT_FRONT]->SetPosition(0, 30);
    // joints_[STEER_LEFT_REAR]->SetPosition(0, 30);
    // joints_[STEER_RIGHT_REAR]->SetPosition(0, 30);
    // joints_[STEER_LEFT_FRONT]->SetPosition(1, 30);
    // joints_[STEER_RIGHT_FRONT]->SetPosition(1, 30);
    // joints_[STEER_LEFT_REAR]->SetPosition(1, 30);
    // joints_[STEER_RIGHT_REAR]->SetPosition(1, 30);
    // joints_[STEER_LEFT_FRONT]->SetPosition(2, 30);
    // joints_[STEER_RIGHT_FRONT]->SetPosition(2, 30);
    // joints_[STEER_LEFT_REAR]->SetPosition(2, 30);
    // joints_[STEER_RIGHT_REAR]->SetPosition(2, 30);
  }

  void AuCarDrive::fnControlJointVelocity()
  {
    joints_[WHEEL_LEFT_FRONT]->SetVelocity(0, x_);
    joints_[WHEEL_RIGHT_FRONT]->SetVelocity(0, x_);
    joints_[WHEEL_LEFT_REAR]->SetVelocity(0, x_);
    joints_[WHEEL_RIGHT_REAR]->SetVelocity(0, x_);
    joints_[WHEEL_LEFT_FRONT]->SetVelocity(1, x_);
    joints_[WHEEL_RIGHT_FRONT]->SetVelocity(1, x_);
    joints_[WHEEL_LEFT_REAR]->SetVelocity(1, x_);
    joints_[WHEEL_RIGHT_REAR]->SetVelocity(1, x_);
    joints_[WHEEL_LEFT_FRONT]->SetVelocity(2, x_);
    joints_[WHEEL_RIGHT_FRONT]->SetVelocity(2, x_);
    joints_[WHEEL_LEFT_REAR]->SetVelocity(2, x_);
    joints_[WHEEL_RIGHT_REAR]->SetVelocity(2, x_);
  }

  void AuCarDrive::cbCmdVel(const geometry_msgs::TwistConstPtr &_msg)
  {
    x_ = _msg->linear.x * 100.0;
    rot_ = _msg->angular.z;
    // std::cout << _msg->linear.x << std::endl;
    // std::cout << _msg->data << std::endl;
    // this->SetVelocity(_msg->data);
  }

  void AuCarDrive::fnQueueThread()
  {
    static const double timeout = 0.01;
    while (this->nh_->ok())
    {
      this->queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

GZ_REGISTER_MODEL_PLUGIN(AuCarDrive)
}