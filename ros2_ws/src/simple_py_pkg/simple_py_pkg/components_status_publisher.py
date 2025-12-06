#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simple_interfaces_pkg.msg import ComponentStatus

class ComponentsStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("components_status_publisher")

        self.component_status_publisher_ = self.create_publisher(
            ComponentStatus,
            "components_status",
            10,
        )

        self.timer_ = self.create_timer(1.0, self.publish_component_status)
        self.get_logger().info("Components Status Publisher Node has been started.")

    def publish_component_status(self):
        msg = ComponentStatus()
        msg.camera_is_ready = True
        msg.lidar_is_ready = True
        msg.motor_is_ready = False
        msg.debug_message = "Motor is not responding."
        self.component_status_publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ComponentsStatusPublisherNode() 
    rclpy.spin(node) 
    rclpy.shutdown()
        


if __name__ == "__main__":
    main()