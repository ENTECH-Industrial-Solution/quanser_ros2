// Wheel + gyro odometry for the QCar2.
//
// qcar2_hardware publishes the motor encoder speed (qcar2_joint, velocity[0], counts/s) and
// the gyroscope (qcar2_imu, angular_velocity.z, rad/s, axes aligned with base_link), each at
// the hardware loop rate. This node integrates
//     v   = encoder speed converted to m/s (same formula as qcar2_hardware's speed controller)
//     yaw rate = gyro z minus its bias (learnt while the car stands still)
// and publishes nav_msgs/Odometry on /odom plus the odom -> base_link transform, which AMCL
// and Nav2 need (and cartographer uses as its published frame in SLAM mode).
//
// The yaw comes from the gyro, not from the steering, because the steering angle is only a
// command and the wheels slip; the gyro is what cartographer also uses.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class WheelOdom : public rclcpp::Node
{
public:
  WheelOdom() : Node("wheel_odom")
  {
    // m/s per encoder count/s: 1/(720*4) rev of the motor, gear 13*19/(70*30), wheel r 0.033 m
    speed_scale_ = declare_parameter(
      "speed_scale", (1.0 / (720.0 * 4.0)) * ((13.0 * 19.0) / (70.0 * 30.0)) * 2.0 * M_PI * 0.033);
    gyro_scale_ = declare_parameter("gyro_scale", 1.0);   // -1 if the IMU were mounted upside down
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    publish_tf_ = declare_parameter("publish_tf", true);
    stopped_speed_ = declare_parameter("stopped_speed", 0.01);   // m/s: below this = standing
    const double rate = declare_parameter("rate", 50.0);

    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      declare_parameter("joint_topic", std::string("/qcar2_joint")), qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        if (msg->velocity.empty()) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        speed_ = msg->velocity[0] * speed_scale_;
        joint_time_ = now();
      });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      declare_parameter("imu_topic", std::string("/qcar2_imu")), qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        gyro_z_ = msg->angular_velocity.z * gyro_scale_;
        imu_time_ = now();
      });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {step();});
    RCLCPP_INFO(get_logger(), "Publishing %s -> %s from %s and %s", odom_frame_.c_str(),
      base_frame_.c_str(), joint_sub_->get_topic_name(), imu_sub_->get_topic_name());
  }

private:
  void step()
  {
    const rclcpp::Time t = now();
    const double dt = (t - last_).seconds();
    last_ = t;
    double v, wz;
    bool fresh;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      v = speed_;
      wz = gyro_z_;
      fresh = joint_time_.nanoseconds() > 0 && (t - joint_time_).seconds() < 0.5 &&
        imu_time_.nanoseconds() > 0 && (t - imu_time_).seconds() < 0.5;
    }
    if (!fresh) {
      v = 0.0;   // hardware node not running (yet): hold the pose
      wz = 0.0;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "No fresh qcar2_joint / qcar2_imu data: odometry is standing still");
    } else if (std::fabs(v) < stopped_speed_) {
      // Standing still: learn the gyro bias and do not let it turn the car on the spot.
      stopped_time_ += dt;
      if (stopped_time_ > 0.5) {
        bias_ += 0.01 * (wz - bias_);
      }
      v = 0.0;
      wz = 0.0;
    } else {
      stopped_time_ = 0.0;
      wz -= bias_;
    }
    if (dt > 0.0 && dt < 0.5) {
      const double mid = yaw_ + 0.5 * wz * dt;   // midpoint integration
      x_ += v * std::cos(mid) * dt;
      y_ += v * std::sin(mid) * dt;
      yaw_ = std::atan2(std::sin(yaw_ + wz * dt), std::cos(yaw_ + wz * dt));
    }

    const double qz = std::sin(yaw_ / 2.0), qw = std::cos(yaw_ / 2.0);
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = t;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.rotation.z = qz;
      tf.transform.rotation.w = qw;
      tf_->sendTransform(tf);
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = t;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = wz;
    for (int i : {0, 7}) {odom.pose.covariance[i] = 0.01;}     // x, y
    odom.pose.covariance[35] = 0.05;                            // yaw
    for (int i : {14, 21, 28}) {odom.pose.covariance[i] = 1e6;}  // z, roll, pitch: unused
    odom.twist.covariance[0] = 0.01;
    odom.twist.covariance[35] = 0.02;
    odom_pub_->publish(odom);
  }

  double speed_scale_, gyro_scale_, stopped_speed_;
  std::string odom_frame_, base_frame_;
  bool publish_tf_;
  std::mutex mutex_;
  double speed_ = 0.0, gyro_z_ = 0.0;
  rclcpp::Time joint_time_{0, 0, RCL_ROS_TIME}, imu_time_{0, 0, RCL_ROS_TIME};
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, bias_ = 0.0, stopped_time_ = 0.0;
  rclcpp::Time last_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelOdom>());
  rclcpp::shutdown();
  return 0;
}
