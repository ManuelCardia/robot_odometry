#include <chrono>
#include <cmath>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "first_project/msg/tf_error_msg.hpp"

using namespace std::chrono_literals;

class TfError : public rclcpp::Node {
public:
  TfError()
  : Node("tf_error"),
    travelled_distance_(0.0),
    last_x_(0.0), last_y_(0.0),
    first_pose_(true),
    start_time_set_(false)
  {
    // Publisher: sends our custom TfErrorMsg
    pub_ = this->create_publisher<first_project::msg::TfErrorMsg>(
      "/tf_error_msg", 10);

    // TF buffer: stores recent transforms from the TF tree
    // We pass the node's clock so the buffer knows about sim time
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    // TF listener: subscribes to /tf and /tf_static automatically
    // and fills the buffer. We just hold a reference to keep it alive.
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer: check transforms and publish error at 10 Hz. We use a timer instead of a subscriber because we need to
    // query TWO different transforms and combine them
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&TfError::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "TfError node started.");
  }

private:
   // Timer callback: runs at 10 Hz
  // Looks up both TFs and computes the error between them
  void timerCallback() {
    geometry_msgs::msg::TransformStamped tf_gt;// ground truth
    geometry_msgs::msg::TransformStamped tf_mine;// our odometry
  // Step 1: look up both transforms

    try {
      tf_gt = tf_buffer_->lookupTransform(
        "odom", "base_link", tf2::TimePointZero);
      tf_mine = tf_buffer_->lookupTransform(
        "odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG(this->get_logger(),
        "TF not yet available: %s", ex.what());
      return;
    }
     // Step 2: record start time
    rclcpp::Time now = this->now();
    if (!start_time_set_) {
      start_time_ = now;
      start_time_set_ = true;
    }

     // Step 3: compute travelled distance
    int32_t time_from_start =
      static_cast<int32_t>((now - start_time_).seconds());

    double mx = tf_mine.transform.translation.x;
    double my = tf_mine.transform.translation.y;

    if (!first_pose_) {
      double dx = mx - last_x_;
      double dy = my - last_y_;
      travelled_distance_ += std::sqrt(dx*dx + dy*dy);
    } else {
      first_pose_ = false;
    }
    last_x_ = mx;
    last_y_ = my;

     // Step 4: compute position error
    double gx = tf_gt.transform.translation.x;
    double gy = tf_gt.transform.translation.y;
    double error = std::sqrt(
      (gx - mx)*(gx - mx) +
      (gy - my)*(gy - my));

    // Step 5: publish message
    first_project::msg::TfErrorMsg out_msg;
    out_msg.header.stamp    = now;
    out_msg.header.frame_id = "odom";
    out_msg.tf_error           = static_cast<float>(error);
    out_msg.time_from_start    = time_from_start;
    out_msg.travelled_distance = static_cast<float>(travelled_distance_);

    pub_->publish(out_msg);
  }

  // Member variables
  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;

  double travelled_distance_;// total meters travelled so far
  double last_x_, last_y_;// position at last timer tick
  bool first_pose_;// true until first valid TF received
  bool start_time_set_; // true after first successful lookup
  rclcpp::Time start_time_; // time of first successful lookup
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfError>());
  rclcpp::shutdown();
  return 0;
}
