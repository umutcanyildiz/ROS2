#!/usr/bin/env python3

from platform import node
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClientNode(Node): #1 kendimiz bir node oluştururken ilk olarak bunu değiştiriyoruz
    def __init__(self):
        super().__init__("add_two_ints_client") #2 node ismi burada belirlenir
        self.call_add_two_ints_server(5,7) # istemci fonksiyonu çağrılır
        self.call_add_two_ints_server(34,-7)
        self.call_add_two_ints_server(-56,22)



    def call_add_two_ints_server(self,a,b): #bu fonksiyon servis istemcisini oluşturur ve servise istek gönderir
        client_ = self.create_client(AddTwoInts,"add_two_ints") 

        while not client_.wait_for_service(1.0): #5 bu fonksiyon servis içine yazılan süre kadar servisin hazır olmasını bekler 
            #(saniyede bir kontrol eder)
            self.get_logger().warn("Service not available, waiting again... [Add Two Ints ]")

        request_ = AddTwoInts.Request()
        request_.a = a
        request_.b = b
        future = client_.call_async(request_)
        future.add_done_callback(lambda future: self.callback_call_add_two_ints(future, a, b))


    def callback_call_add_two_ints(self,future, a, b):
        try:
            respose = future.result() #10 cevap alınır
            self.get_logger().info(f"Result of add_two_ints: {a} + {b} = {respose.sum}") #11 cevap loglanır

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))



def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode() #3  node oluşturulur
    rclpy.spin(node) # node çalışır durumda tutulur (yasam dongusuu)
    rclpy.shutdown()
        





if __name__ == "__main__":
    main()