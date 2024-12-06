# DynamicX final assignment

## 完成情况

所有必做与选做项目均已完成（特色功能已有两个，后续根据时间预算考虑添加功率约束算法）

## 功能核心代码片段

由于一些需求不便在文本展示，下面展示一些功能的代码及其思路与解释

### 4. 使用 PID 控制轮子的速度

```cpp
void HeroChassisController::computeWheelEfforts(const ros::Time& time, const ros::Duration& period)
{
  // 为每个轮子计算力矩
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

  left_front_joint_.setCommand(left_back_effort);
  right_front_joint_.setCommand(right_front_effort);
  left_back_joint_.setCommand(left_front_effort);
  right_back_joint_.setCommand(right_back_effort);
}
```

### 5.使用逆运动学计算各个轮子的期望速度

```cpp
// 逆运动学解算，参考https://www.robotsfan.com/posts/b6e9d4e.html，odom_velocity.angular前系数由轮距和轮距决定
// 这里的速度是线速度
desired_left_front_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
    desired_right_front_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
    desired_left_back_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
    desired_right_back_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
```

这个逆运动学解算写在了回调函数当中，并且与速度坐标系变换写在了一起，这里仅展示片段

### 6.使用正运动学实现里程计

```cpp
void HeroChassisController::updateRobotVelocityAndPosition(const ros::Time& time, const ros::Duration& period)
{
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

  // 通过速度对位置进行积分，计算实际位置
  // 这里对速度乘了个旋转矩阵，因为速度是机器人坐标系下的速度，需要对速度进行坐标变换

  double dt = period.toSec();
  double delta_x = (vx_real * cos(th) - vy_real * sin(th)) * dt;
  double delta_y = (vx_real * sin(th) + vy_real * cos(th)) * dt;
  double delta_th = omega_real * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;
  odom_msg.header.stamp = time;
}

void HeroChassisController::publishOdometryAndTF(const ros::Time& time)
{
  // 发布里程计消息
  odom_msg.header.stamp = time;
  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
  odom_msg.twist.twist.linear.x = vx_real;
  odom_msg.twist.twist.linear.y = vy_real;
  odom_msg.twist.twist.angular.z = omega_real;

  real_speed_publisher_.publish(odom_msg);

  // 发布tf变换，odom到base_link
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, 0.0));  // 设置平移
  tf::Quaternion q;
  q.setRPY(0, 0, th);  // 设置欧拉角，由于只考虑了平面运动，所以只有一个yaw角
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, time, "odom", "base_link"));
}
```

### 7.使用tf计算实现世界坐标下的速度控制

```cpp
void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& cmd_vel)
{
  geometry_msgs::Vector3Stamped chassis_velocity;
  chassis_velocity.vector = cmd_vel.linear;
  chassis_velocity.header.stamp = ros::Time::now();
  chassis_velocity.header.frame_id = "base_link";
  geometry_msgs::Twist odom_velocity;
  geometry_msgs::Vector3Stamped global_velocity;
  tf::TransformListener listener;
  if (odomMode)
  {
    try
    {
      listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(3.0));
      listener.transformVector("odom", ros::Time(0), chassis_velocity, "base_link", global_velocity);
      odom_velocity.linear = global_velocity.vector;
      odom_velocity.angular = cmd_vel.angular;
    }
    catch (tf::TransformException& ex)
    {
      ROS_INFO("%s", ex.what());
    }
    // 逆运动学解算，参考https://www.robotsfan.com/posts/b6e9d4e.html，odom_velocity.angular前系数由轮距和轮距决定
    // 这里的速度是线速度
    desired_left_front_velocity_ =
        odom_velocity.linear.x - odom_velocity.linear.y - (rx + ry) * odom_velocity.angular.z;
    desired_right_front_velocity_ =
        odom_velocity.linear.x + odom_velocity.linear.y + (rx + ry) * odom_velocity.angular.z;
    desired_left_back_velocity_ = odom_velocity.linear.x + odom_velocity.linear.y - (rx + ry) * odom_velocity.angular.z;
    desired_right_back_velocity_ =
        odom_velocity.linear.x - odom_velocity.linear.y + (rx + ry) * odom_velocity.angular.z;
  }
  else
  {
    desired_left_front_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
    desired_right_front_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
    desired_left_back_velocity_ = cmd_vel.linear.x + cmd_vel.linear.y - (rx + ry) * cmd_vel.angular.z;
    desired_right_back_velocity_ = cmd_vel.linear.x - cmd_vel.linear.y + (rx + ry) * cmd_vel.angular.z;
  }
}
```

这里在回调函数中实现了这个功能。在参数中设定了一个bool类型的参数odomMode，可以根据其bool判断是否对速度进行转换

### 8.其他特色功能

1.实现了使用 teleop_twist_keyboard 键盘操控底盘（见launch文件）

2.编写了一个random_move_node节点以随机发布/cmd_vel的速度指令，方便在编写控制器和PID调参时观察底盘的运动（解放双手！）

以上两个功能可以根据执行不同的launch文件使用

3.功率约束算法（todo）

