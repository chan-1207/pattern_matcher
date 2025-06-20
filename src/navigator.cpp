#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Navigator : public rclcpp::Node
{
public:
  Navigator() : Node("navigator")
  {
    // Parameters
    this->declare_parameter("target_frame", "front_of_station");
    this->declare_parameter("base_frame", "base_link");
    this->declare_parameter("map_frame", "map");
    
    target_frame_ = this->get_parameter("target_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();

    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Nav2 action client
    nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

    // Command subscriber
    command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "navigate_command", 10,
      std::bind(&Navigator::command_callback, this, std::placeholders::_1));

    // Timer for checking TF and auto-navigation
    timer_ = this->create_wall_timer(
      1s, std::bind(&Navigator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Navigator node started");
    RCLCPP_INFO(this->get_logger(), "Target frame: %s", target_frame_.c_str());
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "navigate" || msg->data == "go") {
      navigate_to_target();
    } else if (msg->data == "cancel") {
      cancel_navigation();
    }
  }

  void timer_callback()
  {
    // Check if target frame exists
    if (tf_buffer_->canTransform(map_frame_, target_frame_, tf2::TimePointZero)) {
      if (!navigation_requested_ && !navigation_active_) {
        RCLCPP_INFO(this->get_logger(), "Target frame %s detected, ready to navigate", target_frame_.c_str());
      }
    } else {
      if (navigation_requested_) {
        RCLCPP_WARN(this->get_logger(), "Target frame %s not found, canceling navigation", target_frame_.c_str());
        cancel_navigation();
      }
    }
  }

  void navigate_to_target()
  {
    if (navigation_requested_) {
      RCLCPP_WARN(this->get_logger(), "Navigation already requested");
      return;
    }

    if (!nav_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available");
      return;
    }

    // Get transform from map to target frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(map_frame_, target_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not get transform: %s", ex.what());
      return;
    }

    // Create goal pose
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = map_frame_;
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = transform.transform.translation.x;
    goal_msg.pose.pose.position.y = transform.transform.translation.y;
    goal_msg.pose.pose.position.z = transform.transform.translation.z;
    goal_msg.pose.pose.orientation = transform.transform.rotation;

    RCLCPP_INFO(this->get_logger(), "Sending navigation goal to %s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f, %.2f)", 
                goal_msg.pose.pose.position.x, 
                goal_msg.pose.pose.position.y, 
                goal_msg.pose.pose.position.z);
    
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Navigator::navigation_result_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&Navigator::navigation_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    navigation_requested_ = true;
    navigation_active_ = true;
  }

  void cancel_navigation()
  {
    if (navigation_requested_) {
      nav_action_client_->async_cancel_all_goals();
      navigation_requested_ = false;
      navigation_active_ = false;
      RCLCPP_INFO(this->get_logger(), "Navigation canceled");
    }
  }

  void navigation_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    navigation_active_ = false;
    
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Navigation to %s completed successfully", target_frame_.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Navigation to %s was aborted", target_frame_.c_str());
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Navigation to %s was canceled", target_frame_.c_str());
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        break;
    }
    navigation_requested_ = false;
  }

  void navigation_feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Navigation feedback: Distance remaining = %.2f", feedback->distance_remaining);
  }

  // Parameters
  std::string target_frame_;
  std::string base_frame_;
  std::string map_frame_;

  // ROS 2 specific members
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Nav2 action client
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
  bool navigation_requested_{false};
  bool navigation_active_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigator>());
  rclcpp::shutdown();
  return 0;
} 