#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.h>
#include <ros/param.h>

namespace hero_chassis_controller
{
HeroChassisController::HeroChassisController() = default;
HeroChassisController::~HeroChassisController() = default;

void HeroChassisController::reconfigureCallback(PIDConfig& config, uint32_t level)
{
  p_left_front = config.p_left_front;
  i_left_front = config.i_left_front;
  d_left_front = config.d_left_front;
  p_right_front = config.p_right_front;
  i_right_front = config.i_right_front;
  d_right_front = config.d_right_front;
  p_left_back = config.p_left_back;
  i_left_back = config.i_left_back;
  d_left_back = config.d_left_back;
  p_right_back = config.p_right_back;
  i_right_back = config.i_right_back;
  d_right_back = config.d_right_back;
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
{
  config_server_ = std::make_shared<dynamic_reconfigure::Server<PIDConfig>>(controller_nh);
  config_server_->setCallback(boost::bind(&HeroChassisController::reconfigureCallback, this, _1, _2));

  left_front_joint_ = effort_joint_interface->getHandle("left_front_wheel_joint");
  right_front_joint_ = effort_joint_interface->getHandle("right_front_wheel_joint");
  left_back_joint_ = effort_joint_interface->getHandle("left_back_wheel_joint");
  right_back_joint_ = effort_joint_interface->getHandle("right_back_wheel_joint");

  ros::param::get("/wheel_track", wheel_track);
  ros::param::get("/wheel_base", wheel_base);
  ros::param::get("/wheel_radius", wheel_radius);

  rx = wheel_track / 2;
  ry = wheel_base / 2;

  // Output the current parameter values
  ROS_INFO("wheel_track: %f", wheel_track);
  ROS_INFO("wheel_base: %f", wheel_base);
  ROS_INFO("wheel_radius: %f", wheel_radius);
  ROS_INFO("rx: %f", rx);
  ROS_INFO("ry: %f", ry);

  // 加载PID参数
  left_front_pid_.setGains(p_left_front, i_left_front, d_left_front, 1.0, -1.0);
  right_front_pid_.setGains(p_right_front, i_right_front, d_right_front, 1.0, -1.0);
  left_back_pid_.setGains(p_left_back, i_left_back, d_left_back, 1.0, -1.0);
  right_back_pid_.setGains(p_right_back, i_right_back, d_right_back, 1.0, -1.0);

  // 订阅cmd_vel话题获取目标速度
  cmd_vel_subscriber_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  return true;
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  // 为每个轮子计算误差
  // 角速度直接从hardware_interface::JointHandle获取，线速度需乘轮子半径
  //.getVelocity获取的是角速度，计算误差时需要转换成线速度，故需乘轮子半径
  double left_front_effort = left_front_pid_.computeCommand(
      desired_left_front_velocity_ - wheel_radius * left_front_joint_.getVelocity(), period);
  double right_front_effort = right_front_pid_.computeCommand(
      desired_right_front_velocity_ - wheel_radius * right_front_joint_.getVelocity(), period);
  double left_back_effort = left_back_pid_.computeCommand(
      desired_left_back_velocity_ - wheel_radius * left_back_joint_.getVelocity(), period);
  double right_back_effort = right_back_pid_.computeCommand(
      desired_right_back_velocity_ - wheel_radius * right_back_joint_.getVelocity(), period);

  // 设置关节力矩，由于力矩和速度成线性关系，可将误差作为力矩直接输出
  left_front_joint_.setCommand(left_front_effort);
  right_front_joint_.setCommand(right_front_effort);
  left_back_joint_.setCommand(left_back_effort);
  right_back_joint_.setCommand(right_back_effort);

  // 正运动学解算，计算实际速度
  // 这里的是线速度
  vx_real = wheel_radius *
            (left_front_joint_.getVelocity() + right_front_joint_.getVelocity() + left_back_joint_.getVelocity() +
             right_back_joint_.getVelocity()) /
            4;
  vy_real = wheel_radius *
            (-left_front_joint_.getVelocity() + right_front_joint_.getVelocity() + left_back_joint_.getVelocity() -
             right_back_joint_.getVelocity()) /
            4;
  omega_real = wheel_radius *
               (-left_front_joint_.getVelocity() + right_front_joint_.getVelocity() - left_back_joint_.getVelocity() +
                right_back_joint_.getVelocity()) /
               (4 * (rx + ry));
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  // 逆运动学解算，参考https://www.robotsfan.com/posts/b6e9d4e.html，cmd_vel.angular前系数由轮距和轮距决定
  // 这里的速度是线速度
  desired_left_front_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
  desired_right_front_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
  desired_left_back_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
  desired_right_back_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
}  // namespace hero_chassis_controller