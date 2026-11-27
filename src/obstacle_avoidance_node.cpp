
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <csignal>
#include <thread>

using namespace std::chrono_literals;

class ObstacleAvoidanceNode : public rclcpp::Node
{
public:
    ObstacleAvoidanceNode()
        : Node("obstacle_avoidance_node")
    {
        // 订阅真实激光雷达数据
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_callback(msg);
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
           "/odom", 10,
           [this](nav_msgs::msg::Odometry::SharedPtr msg) {
               odom_callback(msg);
           });

       // 发布 TurtleBot3 所需的 TwistStamped
        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10);

        // 10Hz 控制循环
        timer_ = create_wall_timer(
            100ms, [this]() { control_loop(); });

        RCLCPP_INFO(get_logger(), "Obstacle avoidance node started");
        RCLCPP_INFO(get_logger(), "Waiting for real LiDAR data...");
    }

  // 正常退出前发送零速度，保持 ROS 2 通信仍然有效
  void stop_robot()
  {
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.frame_id = "base_link";

      for (int i = 0; i < 5; ++i)
      {
          cmd.header.stamp = now();
          cmd_pub_->publish(cmd);
          std::this_thread::sleep_for(50ms);
      }

      RCLCPP_INFO(get_logger(), "STOP command sent before shutdown");
  }

private:
    void odom_callback(
       const nav_msgs::msg::Odometry::SharedPtr msg)
   {
       const auto &p = msg->pose.pose.position;
       const auto &q = msg->pose.pose.orientation;

       double yaw = std::atan2(
           2.0 * (q.w * q.z + q.x * q.y),
           1.0 - 2.0 * (q.y * q.y + q.z * q.z));

       if (!odom_initialized_)
       {
           origin_x_ = p.x;
           origin_y_ = p.y;
           origin_yaw_ = yaw;
           odom_initialized_ = true;
           RCLCPP_INFO(get_logger(), "Odometry origin recorded");
       }

       double dx = p.x - origin_x_;
       double dy = p.y - origin_y_;

       // Translate and rotate into the starting robot frame
       local_x_ = std::cos(origin_yaw_) * dx
                + std::sin(origin_yaw_) * dy;
       local_y_ = -std::sin(origin_yaw_) * dx
                + std::cos(origin_yaw_) * dy;

       local_yaw_ = std::atan2(
           std::sin(yaw - origin_yaw_),
           std::cos(yaw - origin_yaw_));

       last_odom_time_ = std::chrono::steady_clock::now();
       auto t = std::chrono::steady_clock::now();
       if (t - last_odom_log_ >= 2s)
       {
           RCLCPP_INFO(get_logger(),
               "LOCAL POSE: x=%.2f y=%.2f yaw=%.1f deg",
               local_x_, local_y_,
               local_yaw_ * 180.0 / 3.141592653589793);
           last_odom_log_ = t;
       }
   }

   void scan_callback(
       const sensor_msgs::msg::LaserScan::SharedPtr msg)
   {
       double min_front = std::numeric_limits<double>::infinity();
       double left_sum = 0.0;
       double right_sum = 0.0;
       int front_count = 0;
       int left_count = 0;
       int right_count = 0;

       for (size_t i = 0; i < msg->ranges.size(); ++i)
       {
           double angle = msg->angle_min +
               static_cast<double>(i) * msg->angle_increment;

           angle = std::atan2(std::sin(angle), std::cos(angle));

           double distance = msg->ranges[i];
           double clearance = 0.0;

           if (std::isinf(distance) && distance > 0)
           {
               clearance = msg->range_max;
           }
           else if (std::isfinite(distance) &&
                    distance >= msg->range_min &&
                    distance <= msg->range_max)
           {
               clearance = distance;
           }
           else
           {
               continue;
           }

           // 前方约 ±26 度：使用最近障碍物距离
           if (std::abs(angle) <= 0.45)
           {
               ++front_count;
               if (clearance < min_front)
                   min_front = clearance;
           }

           // 左侧约 30~92 度：计算平均可通行距离
           if (angle >= 0.55 && angle <= 1.60)
           {
               left_sum += clearance;
               ++left_count;
           }

           // 右侧约 -92~-30 度
           if (angle <= -0.55 && angle >= -1.60)
           {
               right_sum += clearance;
               ++right_count;
           }
       }

       front_valid_ = front_count > 0;
       sides_valid_ = left_count > 0 && right_count > 0;

       front_distance_ = min_front;
       left_clearance_ = left_count > 0
           ? left_sum / left_count : 0.0;
       right_clearance_ = right_count > 0
           ? right_sum / right_count : 0.0;

       scan_received_ = true;
       last_scan_time_ = std::chrono::steady_clock::now();
   }

   void control_loop()
   {
       geometry_msgs::msg::TwistStamped cmd;
       cmd.header.stamp = now();
       cmd.header.frame_id = "base_link";

       int state = 0;
       const auto current = std::chrono::steady_clock::now();

       bool scan_fresh =
           scan_received_ &&
           (current - last_scan_time_ < 500ms);

       bool odom_fresh =
           odom_initialized_ &&
           (current - last_odom_time_ < 500ms);

       if (!scan_fresh || !odom_fresh ||
           !front_valid_ || !sides_valid_)
       {
           // 传感器异常时停车
           // 保留避障阶段，避免短暂丢帧后丢失绕障进度
           state = 0;
       }
       else
       {
           double dx = goal_x_ - local_x_;
           double dy = goal_y_ - local_y_;
           double distance_to_goal = std::hypot(dx, dy);

           if (distance_to_goal < 0.18)
               goal_reached_ = true;

           if (goal_reached_)
           {
               avoid_phase_ = 0;
               turn_direction_ = 0;
               state = 4;
           }
           else
           {
               // 阶段0：正常导航
               // 遇到障碍物时进入阶段1
               if (avoid_phase_ == 0 &&
                   front_distance_ < 0.60)
               {
                   turn_direction_ =
                       left_clearance_ >= right_clearance_
                           ? 1 : -1;

                   avoid_phase_ = 1;
               }

               // 阶段1：原地转向，直到前方安全
               if (avoid_phase_ == 1)
               {
                   if (front_distance_ >= 0.85)
                   {
                       avoid_phase_ = 2;
                       bypass_start_x_ = local_x_;
                       bypass_start_y_ = local_y_;
                       bypass_yaw_ = local_yaw_;
                   }
                   else
                   {
                       cmd.twist.angular.z =
                           0.5 * turn_direction_;

                       state = turn_direction_ == 1 ? 2 : 3;
                   }
               }

               // 阶段2：保持避障朝向向前绕行
               if (avoid_phase_ == 2)
               {
                   double traveled = std::hypot(
                       local_x_ - bypass_start_x_,
                       local_y_ - bypass_start_y_);

                   if (traveled >= 0.80)
                   {
                       // 已向侧面绕行足够距离
                       // 允许重新朝向目标点
                       avoid_phase_ = 0;
                       turn_direction_ = 0;
                   }
                   else if (front_distance_ < 0.60)
                   {
                       // 绕行途中再次发现障碍物
                       // 保持原来的避障方向，继续转向
                       avoid_phase_ = 1;

                       cmd.twist.angular.z =
                           0.5 * turn_direction_;

                       state = turn_direction_ == 1 ? 2 : 3;
                   }
                   else
                   {
                       // 保持开始绕行时的朝向
                       double yaw_error = std::atan2(
                           std::sin(bypass_yaw_ - local_yaw_),
                           std::cos(bypass_yaw_ - local_yaw_));

                       cmd.twist.linear.x = 0.12;

                       cmd.twist.angular.z = std::fmax(
                           -0.35,
                           std::fmin(0.35, 1.5 * yaw_error));

                       state = 6;
                   }
               }

               // 阶段0：重新朝目标点导航
               if (avoid_phase_ == 0)
               {
                   double target_yaw = std::atan2(dy, dx);

                   double error = std::atan2(
                       std::sin(target_yaw - local_yaw_),
                       std::cos(target_yaw - local_yaw_));

                   cmd.twist.angular.z = std::fmax(
                       -0.5,
                       std::fmin(0.5, 1.5 * error));

                   if (std::abs(error) > 0.35)
                   {
                       // 朝向偏差较大，先转向
                       state = 5;
                   }
                   else
                   {
                       // 朝目标前进
                       cmd.twist.linear.x = 0.12;
                       state = 1;
                   }
               }
           }
       }

       cmd_pub_->publish(cmd);

       if (state != last_state_)
       {
           if (state == 0)
               RCLCPP_INFO(get_logger(),
                   "STOP: sensor data unavailable");
           else if (state == 1)
               RCLCPP_INFO(get_logger(), "GO TO GOAL");
           else if (state == 2)
               RCLCPP_INFO(get_logger(), "AVOID: TURN LEFT");
           else if (state == 3)
               RCLCPP_INFO(get_logger(), "AVOID: TURN RIGHT");
           else if (state == 4)
               RCLCPP_INFO(get_logger(),
                   "GOAL REACHED: x=%.2f y=%.2f",
                   local_x_, local_y_);
           else if (state == 5)
               RCLCPP_INFO(get_logger(),
                   "ALIGN HEADING TO GOAL");
           else if (state == 6)
               RCLCPP_INFO(get_logger(),
                   "BYPASS: FORWARD 0.80m");

           last_state_ = state;
       }
   }

   rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool scan_received_ = false;
    bool front_valid_ = false;

    double front_distance_ =
        std::numeric_limits<double>::infinity();

    std::chrono::steady_clock::time_point last_scan_time_;

    bool sides_valid_ = false;
   double left_clearance_ = 0.0;
   double right_clearance_ = 0.0;
   int turn_direction_ = 0;

   rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

   bool odom_initialized_ = false;
   double origin_x_ = 0.0;
   double origin_y_ = 0.0;
   double origin_yaw_ = 0.0;

   double local_x_ = 0.0;
   double local_y_ = 0.0;
   double local_yaw_ = 0.0;

   std::chrono::steady_clock::time_point last_odom_log_{};

   // Target in the robot's starting coordinate frame
   const double goal_x_ = 2.0;
   const double goal_y_ = 0.0;
   bool goal_reached_ = false;

   std::chrono::steady_clock::time_point last_odom_time_{};

   // Avoidance state machine:
   // 0 = GOAL, 1 = TURN, 2 = BYPASS FORWARD
   int avoid_phase_ = 0;

   double bypass_start_x_ = 0.0;
   double bypass_start_y_ = 0.0;
   double bypass_yaw_ = 0.0;

   int last_state_ = -1;
};

volatile std::sig_atomic_t stop_requested = 0;

void handle_stop_signal(int)
{
    stop_requested = 1;
}

int main(int argc, char **argv)
{
    rclcpp::init(
        argc, argv,
        rclcpp::InitOptions(),
        rclcpp::SignalHandlerOptions::None);

    std::signal(SIGINT, handle_stop_signal);
    std::signal(SIGTERM, handle_stop_signal);

    auto node = std::make_shared<ObstacleAvoidanceNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    while (rclcpp::ok() && !stop_requested)
    {
        executor.spin_some();
        std::this_thread::sleep_for(20ms);
    }

    node->stop_robot();

    executor.remove_node(node);
    rclcpp::shutdown();

    return 0;
}
