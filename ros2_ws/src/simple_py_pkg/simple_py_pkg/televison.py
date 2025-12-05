#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class ChannelNode(Node): #1 
    def __init__(self):
        super().__init__("channel_node") #2 
        
        self.greeting_="Hi awesome people :)"
        self.publisher_ = self.create_publisher( #topic olusturulma adımlari ; ilk olarak publisher olusturulur
            String, "channel_something", 10)
#parametre olarak mesaj tipi, topic ismi ve kuyruk boyutu verilir
        self.timer_= self.create_timer(0.5,self.publish_channel) #0.5 saniyede bir publish_channel fonksiyonu cagrilir
        self.get_logger().info("[INFO] Channel Node has been published.")
    
    def publish_channel(self):
        msg = String()
        msg.data = str(self.greeting_)+" Welcome to the channel!" 
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ChannelNode() #3
    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown()
        


if __name__ == "__main__":
    main()