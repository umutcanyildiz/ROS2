#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from simple_interfaces_pkg.srv import MultiplyTwoInts

#bu nesne yönelimsiz bir şekilde çalışan basit bir node oluşturur

def main(args=None):
    rclpy.init(args=args)
    node = Node("multiply_two_ints_client") #3  node oluşturulur

    client_ = node.create_client(
        MultiplyTwoInts
        , "multiply_two_ints" #bu isim serverdaki servis ismi ile aynı olmalı
    ) #4 servis istemcisi oluşturulur

    #bu blok ömemli

    while not client_.wait_for_service(1.0): #5 bu fonksiyon servis içine yazılan süre kadar servisin hazır olmasını bekler 
        #(saniyede bir kontrol eder)
        node.get_logger().warn("Service not available, waiting again... [Multiply Two Ints ]")
    

    request = MultiplyTwoInts.Request() #6 istek objesi oluşturulur
    request.a = 165 #7 istek objesine değerler atanır
    request.b = 35
    future = client_.call_async(request) #8 istek asenkron olarak gönderilir
    rclpy.spin_until_future_complete(node, future) #9 cevap gelene kadar beklenir
    
    try:
        respose = future.result() #10 cevap alınır
        node.get_logger().info(f"Result of multiply_two_ints: {request.a} * {request.b} = {respose.result}") #11 cevap loglanır


    except Exception as e:  
        node.get_logger().error("Service call failed %r" % (e,))


    rclpy.shutdown()
        





if __name__ == "__main__":
    main()