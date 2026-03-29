#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TimeSubscriber(Node):
    """
    A ROS2 node that subscribes to the 'current_time' topic and logs received messages.
    """

    def __init__(self):
        super().__init__("time_subscriber")

        # Create a subscription to the 'current_time' topic
        self.subscription = self.create_subscription(
            String,
            "current_time",
            self.listener_callback,
            10,  # QoS history depth
        )
        self.subscription  # prevent unused variable warning

        self.get_logger().info("Time subscriber node started")

    def listener_callback(self, msg: String) -> None:
        """
        Callback function to log the received time from the 'current_time' topic.

        Args:
            msg (String): Message received from the topic.
        """
        self.get_logger().info(f"Received time: {msg.data}")


def main(args=None) -> None:
    """
    Main function to initialize the subscriber node and subscribe to the 'current_time' topic.
    """
    rclpy.init(args=args)

    time_subscriber = TimeSubscriber()

    try:
        rclpy.spin(time_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        time_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
