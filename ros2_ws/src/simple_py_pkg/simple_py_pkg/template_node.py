#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class CustomNodeName(Node): #1 kendimiz bir node oluştururken ilk olarak bunu değiştiriyoruz
    def __init__(self):
        super().__init__("custom_node_name") #2 node ismi burada belirlenir

def main(args=None):
    rclpy.init(args=args)
    node = CustomNodeName() #3  node oluşturulur
    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown()
        





if __name__ == "__main__":
    main()