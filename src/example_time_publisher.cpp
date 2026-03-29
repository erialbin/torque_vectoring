#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class TimePublisher : public rclcpp::Node {
 public:
  TimePublisher() : Node("time_publisher") {
    // Declare and get the 'publish_rate' parameter with a default value of 1.0
    // Hz
    this->declare_parameter("publish_rate", 1.0);
    double publish_rate = this->get_parameter("publish_rate").as_double();

    if (publish_rate <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "Invalid 'publish_rate' parameter. Defaulting to 1.0 Hz.");
      publish_rate = 1.0;
    }

    // Create a publisher on the "current_time" topic with a queue size of 10
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("current_time", 10);

    // Calculate the timer period from the publish rate
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);

    // Create a timer that calls the publish callback at the specified rate
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
        std::bind(&TimePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(),
                "Time publisher node started with rate: %.2f Hz", publish_rate);
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();

    // Get current time and format it as a string
    auto now = this->get_clock()->now();
    message.data = "Current Time: " + std::to_string(now.seconds());

    // Publish the message
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TimePublisher>());
  rclcpp::shutdown();
  return 0;
}
