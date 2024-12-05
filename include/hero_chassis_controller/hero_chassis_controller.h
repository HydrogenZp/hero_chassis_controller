#ifndef HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_H

#include <hero_chassis_controller/PIDConfig.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
namespace hero_chassis_controller
{
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController();
  ~HeroChassisController() override;

  bool init(hardware_interface::EffortJointInterface* effort_joint_interface, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

private:
  void reconfigureCallback(const PIDConfig& config, uint32_t level);
  void cmdVelCallback(const geometry_msgs::Twist& cmd_vel);
  void computeWheelEfforts(const ros::Time& time, const ros::Duration& period);
  void updateRobotVelocityAndPosition(const ros::Time& time, const ros::Duration& period);
  void publishOdometryAndTF(const ros::Time& time);
  std::shared_ptr<dynamic_reconfigure::Server<PIDConfig>> config_server_;

  double wheel_track{}, wheel_base{}, wheel_radius{};
  double rx{}, ry{};
  double vx_real{}, vy_real{}, omega_real{};

  double p_left_front, i_left_front, d_left_front;
  double p_right_front, i_right_front, d_right_front;
  double p_left_back, i_left_back, d_left_back;
  double p_right_back, i_right_back, d_right_back;

  hardware_interface::JointHandle left_front_joint_, right_front_joint_, left_back_joint_, right_back_joint_;
  control_toolbox::Pid left_front_pid_, right_front_pid_, left_back_pid_, right_back_pid_;
  ros::Subscriber cmd_vel_subscriber_;

  double desired_left_front_velocity_;
  double desired_right_front_velocity_;
  double desired_left_back_velocity_;
  double desired_right_back_velocity_;

  double x, y, th;

  ros::Publisher real_speed_publisher_;
  nav_msgs::Odometry odom_msg;
  bool odomMode;
};
}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_H