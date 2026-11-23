#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_srvs/srv/empty.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using std::placeholders::_1;
using std::placeholders::_2;

class Odometer : public rclcpp::Node {
public:
  Odometer() : Node("odometer"), x_(0.0), y_(0.0), theta_(0.0),
               last_time_set_(false) {
    // CALIBRATED ROBOT PARAMETERS, obtained by observing the data from the bag ad calibrating the RPM to v and w
    // wheel_radius: R = v*60 / (2π * RPM_avg_sprocket) = 0.083m, obtained with the data from the bags
    // gear_ratio: 7.5 derived from info about the robot (pdf online)
    // track_separation: L = (vR-vL)/ω = 0.688m (effective, accounts for slip, the actual one is 635)->
    // the difference in L from the real one comes from ski steering: when the robot turns the effective baseline is larger
    // because the tracks slip laterally
    // the angular velocity is negative when vl>vr so when the robot
    // is turning right, therefore I derived from the bags that :
    // motor_id 0 → RIGHT track (derived from the sign of angular velocity in relation to the RPMs of the two motors)
    // motor_id 1 → LEFT  track
    wheel_radius_     = 0.083;
    gear_ratio_       = 7.5;
    track_separation_ = 0.688;
    // Publisher odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/project_odom", 10);

 // TF broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

// Subscriber /bunker_status
    sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&Odometer::statusCallback, this, _1));

// Servizio reset
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Odometer::resetCallback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Odometer node started.");
    RCLCPP_INFO(this->get_logger(),
      "R=%.3fm  gear=%.1f  L=%.3fm",
      wheel_radius_, gear_ratio_, track_separation_);
  }

private:
  // Converts motor RPM to track linear velocity [m/s]
  // v = RPM_motor * 2π * R / (60 * gear_ratio)
  double rpmToVelocity(double rpm_motor) {
    return rpm_motor * 2.0 * M_PI * wheel_radius_
           / (60.0 * gear_ratio_);
  }

   // Reads the real RPM values from the message.
  // The driver sends a duplicated motor_id=0 with rpm=0, so we skip zeros.
  // motor_id 0 = right track, motor_id 1 = left track.
  void extractRPM(
    const bunker_msgs::msg::BunkerStatus::SharedPtr & msg,
    double & rpm_right,
    double & rpm_left)
  {
    rpm_right = 0.0;
    rpm_left  = 0.0;
    bool found_right = false;
    bool found_left  = false;

    for (const auto & actuator : msg->actuator_states) {
      if (actuator.motor_id == 0 && !found_right) {
        if (actuator.rpm != 0) {
          rpm_right = actuator.rpm;
          found_right = true;
        }
      }
      if (actuator.motor_id == 1 && !found_left) {
        if (actuator.rpm != 0) {
          rpm_left = actuator.rpm;
          found_left = true;
        }
      }
    }
  }

  // Main callback: computes odometry from RPM data
  void statusCallback(
    const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    rclcpp::Time current_time(
      msg->header.stamp,
      this->get_clock()->get_clock_type());
    // First message: just store the timestamp.
    if (!last_time_set_) {
      last_time_ = current_time;
      last_time_set_ = true;
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) return;

    // 1) Extract real RPM values (filter buggy zeros)
    double rpm_right, rpm_left;
    extractRPM(msg, rpm_right, rpm_left);

    // 2) Convert RPM to track linear velocities
    double v_right = rpmToVelocity(rpm_right);
    double v_left  = rpmToVelocity(rpm_left);

    // 3) Differential drive kinematics
    // v     = (v_right + v_left) / 2
    // omega = (v_right - v_left) / L
    // Sign: v_right > v_left → omega > 0 → turning LEFT (CCW) ✓
    double v     = (v_right + v_left) / 2.0;
    double omega = (v_right - v_left) / track_separation_;

    RCLCPP_DEBUG(this->get_logger(),
      "RPM R=%.0f L=%.0f | v=%.3f(msg:%.3f) w=%.3f(msg:%.3f)",
      rpm_right, rpm_left,
      v, msg->linear_velocity,
      omega, msg->angular_velocity);

    // 4) Runge-Kutta 2nd order integration (midpoint method)
    // theta_mid = theta_k + omega * dt/2
    // More accurate than Euler because it uses midpoint orientation
    double theta_mid = theta_ + omega * (dt / 2.0);
    x_     += v * std::cos(theta_mid) * dt;
    y_     += v * std::sin(theta_mid) * dt;
    theta_ += omega * dt;

    // Normalize theta to [-pi, pi]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    publishOdometry(current_time);
    publishTF(current_time);
  }

  // Publishes nav_msgs/Odometry on /project_odom
  void publishOdometry(const rclcpp::Time & t) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = t;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link2";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom);
  }

  // Broadcasts TF transform odom -> base_link2
  void publishTF(const rclcpp::Time & t) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = t;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link2";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // Resets odometry to zero
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    last_time_set_ = false;
    RCLCPP_INFO(this->get_logger(), "Odometry reset.");
  }

  double wheel_radius_;
  double gear_ratio_;
  double track_separation_;
  double x_, y_, theta_;
  bool last_time_set_;
  rclcpp::Time last_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
