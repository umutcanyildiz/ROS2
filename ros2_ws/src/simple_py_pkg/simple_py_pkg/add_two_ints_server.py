#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts #ilk olarak kullancagımız interface i çalışma dahil ediyoruz


class AddTwoIntsServerNode(Node): 
    def __init__(self):
        super().__init__("add_two_ints_server") #2 node ismi burada belirlenir (script adı ile aynı olması onerilir)
        self_server_ = self.create_service( #3 servis oluşturma
            AddTwoInts, # servis tipi
            "add_two_ints", # servis ismi
            self.add_two_ints_callback # servis çağrıldığında çalışacak fonksiyon
        )
        self.get_logger().info("Add Two Ints server has been started") # servis hazır mesajı

    def add_two_ints_callback(self, request, response): # servis çağrıldığında çalışacak fonksiyon
        response.sum = request.a + request.b # istekdeki iki sayıyı topla
        self.get_logger().info(f"Incoming request: a={request.a}, b={request.b} and response {response.sum}") # gelen istek mesajı

        return response # cevabı döndür (burda response objesi döndürülür eğer sum döndürülse hata verir)
    

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServerNode()
    rclpy.spin(node) # 
    rclpy.shutdown()
        





if __name__ == "__main__":
    main()