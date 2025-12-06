#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simple_interfaces_pkg.srv import MultiplyTwoInts #ilk olarak kullancagımız interface i çalışma dahil ediyoruz


class MultiplyTwoIntsServerNode(Node): 
    def __init__(self):
        super().__init__("multiply_two_ints_server") #2 node ismi burada belirlenir (script adı ile aynı olması onerilir)
        self_server_ = self.create_service( #3 servis oluşturma
            MultiplyTwoInts, # servis tipi
            "multiply_two_ints", # servis ismi
            self.multiply_two_ints_callback # servis çağrıldığında çalışacak fonksiyon
        )
        self.get_logger().info("Multiply Two Ints server has been started") # servis hazır mesajı

    def multiply_two_ints_callback(self, request, response): # servis çağrıldığında çalışacak fonksiyon
        response.result = request.a * request.b # istekdeki iki sayıyı çarpıp cevaba ata
        self.get_logger().info(f"Incoming request: a={request.a}, b={request.b} and response {response.result}") # gelen istek mesajı

        return response # cevabı döndür (burda response objesi döndürülür eğer sum döndürülse hata verir)
    

def main(args=None):
    rclpy.init(args=args)
    node = MultiplyTwoIntsServerNode()
    rclpy.spin(node) # 
    rclpy.shutdown()
        



if __name__ == "__main__":
    main()