#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class CounterNode(Node): #1 kendimiz bir node oluştururken ilk olarak bunu değiştiriyoruz
    def __init__(self):
        super().__init__("counter_node") #2 node ismi burada belirlenir
        self.counter_=0
        self.create_timer(1.0, self.timer_callback) #bu fonksiyon girdiğimiz belirli bir aralıklarla yapmamızı sağlar.

    
    def timer_callback(self):
        self.counter_+=1
        self.get_logger().info("Hello Word - "+ str(self.counter_))



def main(args=None):
    rclpy.init(args=args)
    node = CounterNode() #3  node oluşturulur
    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown()
        





if __name__ == "__main__":
    main()