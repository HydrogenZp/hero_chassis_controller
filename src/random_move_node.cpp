/*
 * @file random_move_node.cpp
 * @brief 本程序实现了一个随机移动节点，通过发布/cmd_vel话题来控制底盘的速度，使其随机移动。
 * 本程序会在指定范围内随机生成线速度和角速度，并且逐步改变当前速度到目标速度，使得底盘的运动更加平滑，几乎不会出现因力矩突变导致的运动突变。
 * 本程序会以0.7Hz的频率发布速度消息。
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cstdlib>
#include <ctime>
#include <algorithm>

// 限制值在最小值和最大值之间
double clamp(double value, double min, double max)
{
  if (value < min)
    return min;
  if (value > max)
    return max;
  return value;
}

// 线性插值函数
double lerp(double start, double end, double t)
{
  return start + t * (end - start);
}

// 生成随机速度，确保在指定范围内，并且变化量不超过最大变化量
double generateRandomSpeed(double current_speed, double min_speed, double max_speed, double max_change)
{
  double new_speed = current_speed + static_cast<double>((std::rand() % 100 - 50)) / 100 * max_change;
  return clamp(new_speed, min_speed, max_speed);
}

int main(int argc, char** argv)
{
  // 初始化ROS节点
  ros::init(argc, argv, "random_move_node");
  ros::NodeHandle nh;
  // 创建速度发布者
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // 设置随机数种子
  std::srand(std::time(0));

  // 设置更新频率为0.7Hz
  ros::Rate rate(0.7);

  // 初始化当前速度
  double current_linear_x = 2.0;
  double current_linear_y = 2.0;
  double current_angular_z = 0.0;

  // 初始化目标速度
  double target_linear_x = current_linear_x;
  double target_linear_y = current_linear_y;
  double target_angular_z = current_angular_z;

  while (ros::ok())
  {
    geometry_msgs::Twist msg;

    // 偶尔改变目标速度
    if (std::rand() % 10 == 0)
    {
      target_linear_x = generateRandomSpeed(current_linear_x, 1.0, 3.0, 0.5);
      target_linear_y = generateRandomSpeed(current_linear_y, 1.0, 3.0, 0.5);
      target_angular_z = generateRandomSpeed(current_angular_z, 1.0, 3.0, 0.5);
    }

    // 使用线性插值逐步改变当前速度
    current_linear_x = lerp(current_linear_x, target_linear_x, 0.1);
    current_linear_y = lerp(current_linear_y, target_linear_y, 0.1);
    current_angular_z = lerp(current_angular_z, target_angular_z, 0.1);

    // 设置消息的速度值
    msg.linear.x = current_linear_x;
    msg.linear.y = current_linear_y;
    msg.linear.z = 0.0;
    msg.angular.z = current_angular_z;

    // 发布速度消��
    cmd_vel_pub.publish(msg);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}