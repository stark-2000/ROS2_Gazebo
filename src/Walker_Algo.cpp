/*  MIT License

    Copyright (c) 2022 Dhinesh Rajasekaran

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to
    deal in the Software without restriction, including without limitation the
    rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
    sell copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
    IN THE SOFTWARE.
*/

/**
 * @file Walker_Algo.cpp
 * @author Dhinesh Rajasekaran
 * @brief Gazebo simulation of turtlebot 3
 * @version 1.0
 * @date 2022-12-01
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Using namespace to improve code readability
using std::placeholders::_1;
using namespace std::chrono_literals;

using LIDAR = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief Obstacle Avoidance class, that receives laser data and steers the turtle bot
 *
 */
class CollisionDet : public rclcpp::Node {
 public:
  CollisionDet() : Node("ROS2_Gazebo") {
    // Initialize the publisher and subscriber
    auto callback = std::bind(&CollisionDet::timer_callback_1, this, _1);
    lidar_data = this->create_subscription<LIDAR>("scan", 10, callback);
    pub_vel = this->create_publisher<TWIST>("cmd_vel", 10);
  }

 private:
  void timer_callback_1(const LIDAR& msg) {
    if (msg.header.stamp.sec == 0) {
      return;
    }
    auto scan_data = msg.ranges;
    auto start_angle = 180;
    auto angle_range = 45;
    for (int i = start_angle; i < start_angle + angle_range; i++) {
      if (scan_data[i % 360] < 0.8) {
        execAction(0.0, 0.5);
      } else {
        execAction(0.5, 0.0);
      }
    }
  }
  void execAction(int x_vel, int z_vel) {
    auto vel = TWIST();
    vel.linear.x = x_vel;
    vel.angular.z = -z_vel;
    pub_vel->publish(vel);
  }

  rclcpp::Subscription<LIDAR>::SharedPtr lidar_data;
  rclcpp::Publisher<TWIST>::SharedPtr pub_vel;
  rclcpp::TimerBase::SharedPtr m_timer;
  LIDAR last_data;
};

/**
 * @brief Main function which initialises ROS communication and service
 * @param argc stores the no of arguements to send & receive in command line
 * @param argv stores the actual arguement data to send and receive in command
 * line
 * @return zero
 *
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionDet>());
  rclcpp::shutdown();
  return 0;
}
