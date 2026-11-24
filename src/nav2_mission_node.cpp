
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace std::chrono_literals;

volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int)
{
  stop_requested = 1;
}

class Nav2Mission : public rclcpp::Node
{
public:
  using Navigate = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Navigate>;

  Nav2Mission() : Node("nav2_mission_node")
  {
    client_ = rclcpp_action::create_client<Navigate>(
      this, "/navigate_to_pose");
  }

  void start()
  {
    RCLCPP_INFO(get_logger(), "Waiting for Nav2 action server...");

    if (!client_->wait_for_action_server(10s)) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server unavailable");
      finish(false);
      return;
    }

    send_next_goal();
  }

  void cancel()
  {
    if (done_ || cancel_requested_) {
      return;
    }

    cancel_requested_ = true;
    RCLCPP_WARN(get_logger(), "Mission cancellation requested");

    if (current_goal_) {
      client_->async_cancel_goal(current_goal_);
    }
  }

  bool done() const { return done_; }
  bool success() const { return success_; }

private:
  struct Waypoint
  {
    double x;
    double y;
  };

  // 三个室内导航目标，按顺序执行
  std::vector<Waypoint> waypoints_ = {
    {1.5, 1.2},
    {-1.5, -1.2},
    {1.5, 1.2}
  };

  void send_next_goal()
  {
    if (cancel_requested_) {
      finish(false);
      return;
    }

    if (index_ >= waypoints_.size()) {
      RCLCPP_INFO(get_logger(),
        "MISSION SUCCEEDED: all %zu goals completed",
        waypoints_.size());
      finish(true);
      return;
    }

    const auto & point = waypoints_[index_];

    Navigate::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = point.x;
    goal.pose.pose.position.y = point.y;
    goal.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(get_logger(),
      "Sending goal %zu/%zu: x=%.2f y=%.2f",
      index_ + 1, waypoints_.size(), point.x, point.y);

    rclcpp_action::Client<Navigate>::SendGoalOptions options;

    options.goal_response_callback =
      [this](GoalHandle::SharedPtr handle)
      {
        if (!handle) {
          RCLCPP_ERROR(get_logger(), "Goal rejected");
          finish(false);
          return;
        }

        current_goal_ = handle;
        RCLCPP_INFO(get_logger(), "Goal accepted");

        if (cancel_requested_) {
          client_->async_cancel_goal(handle);
        }
      };

    options.feedback_callback =
      [this](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const Navigate::Feedback> feedback)
      {
        auto t = std::chrono::steady_clock::now();

        if (t - last_feedback_ < 2s) {
          return;
        }

        last_feedback_ = t;

        RCLCPP_INFO(get_logger(),
          "Goal %zu: remaining=%.2fm recoveries=%d",
          index_ + 1,
          feedback->distance_remaining,
          static_cast<int>(feedback->number_of_recoveries));
      };

    options.result_callback =
      [this](const GoalHandle::WrappedResult & result)
      {
        current_goal_.reset();

        if (cancel_requested_) {
          RCLCPP_WARN(get_logger(), "Mission canceled");
          finish(false);
          return;
        }

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result && result.result->error_code == 0)
        {
          RCLCPP_INFO(get_logger(),
            "Goal %zu SUCCEEDED", index_ + 1);

          ++index_;
          send_next_goal();
        }
        else
        {
          RCLCPP_ERROR(get_logger(),
            "Goal %zu FAILED, action status=%d, error=%s",
            index_ + 1,
            static_cast<int>(result.code),
            result.result ? result.result->error_msg.c_str() : "unknown");

          finish(false);
        }
      };

    client_->async_send_goal(goal, options);
  }

  void finish(bool success)
  {
    success_ = success;
    done_ = true;
  }

  rclcpp_action::Client<Navigate>::SharedPtr client_;
  GoalHandle::SharedPtr current_goal_;

  std::size_t index_{0};
  bool done_{false};
  bool success_{false};
  bool cancel_requested_{false};

  std::chrono::steady_clock::time_point last_feedback_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(
    argc, argv,
    rclcpp::InitOptions(),
    rclcpp::SignalHandlerOptions::None);

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  auto node = std::make_shared<Nav2Mission>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  node->start();

  bool cancellation_started = false;
  auto cancel_time = std::chrono::steady_clock::now();

  while (rclcpp::ok() && !node->done()) {
    executor.spin_some();

    if (stop_requested) {
      if (!cancellation_started) {
        cancellation_started = true;
        cancel_time = std::chrono::steady_clock::now();
        node->cancel();
      }

      if (std::chrono::steady_clock::now() - cancel_time > 5s) {
        RCLCPP_ERROR(node->get_logger(),
          "Cancellation timed out");
        break;
      }
    }

    std::this_thread::sleep_for(50ms);
  }

  bool success = node->success();
  rclcpp::shutdown();
  return success ? 0 : 1;
}
