#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import String
class RemoteControllerNode(Node): #1 kendimiz bir node oluştururken ilk olarak bunu değiştiriyoruz
    def __init__(self):
        super().__init__("remote_controller_node") #2 node ismi burada belirlenir
        self.subscriber_=self.create_subscription(
            String,"channel_something",self.callback_television,10
        ) #burda yazdığımz channel_something oluşturduğumuz
        #publisherın ismi
        #subscriber oluşturma adımları ; ilk olarak subscriber olusturulur
        self.get_logger().info("[INFO] Remote Controller Node has been subscribed.")

    #subscriberın callback fonksiyonu
    def callback_television(self,msg):
        self.get_logger().info(msg.data)  



def main(args=None):
    rclpy.init(args=args)
    node = RemoteControllerNode() #3  node oluşturulur
    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown()
        





if __name__ == "__main__":
    main()