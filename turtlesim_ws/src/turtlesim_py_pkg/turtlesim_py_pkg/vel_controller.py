#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelControllerNode(Node): #1 kendimiz bir node oluştururken ilk olarak bunu değiştiriyoruz
    def __init__(self):
        super().__init__("vel_controller_node") #2 node ismi burada belirlenir

        self.publisher_= self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )
        self.timer_ = self.create_timer(1.0, self.publish_vel)

        self.get_logger().info("Velocity Controller Node has been started.")

    
    def publish_vel(self):
        msg = Twist()
        # w = v/r ---> burdaki formülde w açısal hız , v lineer hız ,  r yaricap
        linear_x = float(sys.argv[1])
        radius = float(sys.argv[2])
        msg.linear.x = linear_x
        msg.linear.y = 0.0
        msg.angular.z = float(linear_x / radius) #  radian: -3.14 den +3.14 e kadar
        self.publisher_.publish(msg)
        self.get_logger().info("Published velocity command.")


def main(args=None):
    rclpy.init(args=args)
    node = VelControllerNode() #3  node oluşturulur
    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()