#include <to_cmd_vel.hpp>

#include <cmath>

namespace pmc = prius::msg_converter;

pmc::ToCmdVel::ToCmdVel():
  wheel_base_(3.0),
  max_steer_angle_(0.6458),
  max_vel_(38.0),
  kp_throttle_(0.1),
  ki_throttle_(0.1),
  kp_brake_(0.1),
  ki_brake_(0.1),
  pub_rate_(25.0)
{
  nh_.param<double>(ros::this_node::getName() + "/wheel_base", wheel_base_, wheel_base_);
  nh_.param<double>(ros::this_node::getName() + "/max_steer_angle", max_steer_angle_, max_steer_angle_);
  nh_.param<double>(ros::this_node::getName() + "/max_velocity", max_vel_, max_vel_);
  nh_.param<double>(ros::this_node::getName() + "/kp_throttle", kp_throttle_, kp_throttle_);
  nh_.param<double>(ros::this_node::getName() + "/ki_throttle", ki_throttle_, ki_throttle_);
  nh_.param<double>(ros::this_node::getName() + "/kp_brake", kp_brake_, kp_brake_);
  nh_.param<double>(ros::this_node::getName() + "/ki_brake", ki_brake_, ki_brake_);
  nh_.param<double>(ros::this_node::getName() + "/publish_rate", pub_rate_);

  control_msg_pub_ = nh_.advertise<prius_msgs::Control>("/prius", 1);

  cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &pmc::ToCmdVel::CmdVelCallback, this);
  current_state_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &pmc::ToCmdVel::CurStateCallback, this);
}

pmc::ToCmdVel::~ToCmdVel(){ }

void pmc::ToCmdVel::CmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
  cmd_vel_mtx_.lock();
  cmd_vel_ = *msg;
  cmd_vel_mtx_.unlock();
}

void pmc::ToCmdVel::CurStateCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_state_mtx_.lock();
  current_state_ = *msg;
  current_state_mtx_.unlock();
}

void pmc::ToCmdVel::CtrlUpdate()
{  
  static ros::Rate r(pub_rate_);
  static prius_msgs::Control ctrl_msg;
  
  while(ros::ok()){
    ros::spinOnce();
    
    // get current state
    static double current_vel = 0;
    current_state_mtx_.lock();
    current_vel = sqrt(pow(current_state_.twist.twist.linear.x,2)+pow(current_state_.twist.twist.linear.y,2));
    current_state_mtx_.unlock();

    // get objective velocity
    static double obj_vel = 0;
    static double obj_ang_vel = 0;
    cmd_vel_mtx_.lock();
    obj_vel = cmd_vel_.linear.x;
    obj_ang_vel = cmd_vel_.angular.z;
    cmd_vel_mtx_.unlock();

    std::cout << "obj_vel : ";
    std::cout << obj_vel << std::endl;
    std::cout << "current_vel :";
    std::cout << current_vel << std::endl;
    
    // set time stamp for control input
    ctrl_msg.header.stamp = ros::Time::now();
    
    // calculate control steer angle
    static double ctrl_angle = 0;
    ctrl_angle = atan((wheel_base_*obj_ang_vel)/current_vel)/(2.0*max_steer_angle_);
    ctrl_angle = std::fabs(ctrl_angle) > 1.0 ? ctrl_angle/std::fabs(ctrl_angle) : ctrl_angle;

    ctrl_msg.steer = ctrl_angle;

    // calculate throttle & shift gear input
    if(obj_vel==0){
      ctrl_msg.throttle = 0.0;
      ctrl_msg.brake = 1.0;
      ctrl_msg.steer = 0.0;
      std::cout << "brake :";
      std::cout << ctrl_msg.brake << std::endl;
      std::cout << "throttle : ";
      std::cout << ctrl_msg.throttle << std::endl;
      control_msg_pub_.publish(ctrl_msg);
      r.sleep();
    }
    else if(obj_vel>0){
      ctrl_msg.throttle = 0.0;
      ctrl_msg.brake = 0.0;
      ctrl_msg.shift_gears = prius_msgs::Control::FORWARD;
    }
    else{
      ctrl_msg.throttle = 0.0;
      ctrl_msg.brake = 0.0;
      ctrl_msg.shift_gears = prius_msgs::Control::REVERSE;
    }
      
    static double err_vel = 0;
    static double err_int_vel = 0;
    static double ctrl_vel = 0;
    err_vel = std::fabs(obj_vel) - current_vel;

    std::cout << "err_vel : ";
    std::cout << err_vel << std::endl;
    std::cout << "err_int_vel : ";
    std::cout << err_int_vel << std::endl;
        
    switch(ctrl_msg.shift_gears){
    case prius_msgs::Control::FORWARD:
      if(0<err_vel){
        err_int_vel += err_vel;
        ctrl_vel = kp_throttle_*std::fabs(err_vel) + ki_throttle_*std::fabs(err_int_vel);
        ctrl_vel = std::fabs(ctrl_vel) >= 1.0 ? 1.0 : ctrl_vel;
        ctrl_msg.throttle = ctrl_vel;
      }
      else{
        err_int_vel -= err_vel;
        ctrl_vel = kp_brake_*std::fabs(err_vel) + ki_brake_*std::fabs(err_int_vel);
        ctrl_vel = std::fabs(ctrl_vel) >= 1.0 ? 1.0 : ctrl_vel;
        ctrl_msg.brake = ctrl_vel;   
      }
      break;
    case prius_msgs::Control::REVERSE:
      if(0<err_vel){
        err_int_vel -= err_vel;
        ctrl_vel = kp_throttle_*std::fabs(err_vel) + ki_throttle_*std::fabs(err_int_vel);
        ctrl_vel = std::fabs(ctrl_vel) >= 1.0 ? 1.0 : ctrl_vel;
        ctrl_msg.throttle = ctrl_vel;
      }
      else{
        err_int_vel += err_vel;
        ctrl_vel = kp_brake_*std::fabs(err_vel) + ki_brake_*std::fabs(err_int_vel);
        ctrl_vel = std::fabs(ctrl_vel) >= 1.0 ? 1.0 : ctrl_vel;
        ctrl_msg.brake = ctrl_vel;   
      }
      break;
    default:
      break;
    }
 
    std::cout << "brake :";
    std::cout << ctrl_msg.brake << std::endl;
    std::cout << "throttle : ";
    std::cout << ctrl_msg.throttle << std::endl;
  
    // publish the ctrl msg
    control_msg_pub_.publish(ctrl_msg);
    r.sleep();

  }
}




