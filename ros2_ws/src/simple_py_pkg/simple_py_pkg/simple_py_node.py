#!/usr/bin/env python3

# 1- library
import rclpy
from rclpy.node import Node

# 2 - Method

def main(args=None):
    rclpy.init(args=args) # bir node oluşturmak için rclpy başlatılır
    node = Node('py_node') # node oluşturulur

    #print("Hello ros2")
    node.get_logger().info("Hello ros2") #print gibi düşünülebilir

    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown() 

# 3 - 'if __name__' block

if __name__ == '__main__':
    main()