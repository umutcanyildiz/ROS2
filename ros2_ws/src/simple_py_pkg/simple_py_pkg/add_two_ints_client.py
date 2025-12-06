#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

#bu nesne yönelimsiz bir şekilde çalışan basit bir node oluşturur

def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_client") #3  node oluşturulur

    client_ = node.create_client(
        AddTwoInts
        , "add_two_ints" #bu isim serverdaki servis ismi ile aynı olmalı
    ) #4 servis istemcisi oluşturulur

    #bu blok ömemli

    while not client_.wait_for_service(1.0): #5 bu fonksiyon servis içine yazılan süre kadar servisin hazır olmasını bekler 
        #(saniyede bir kontrol eder)
        node.get_logger().warn("Service not available, waiting again... [Add Two Ints ]")
    

    request = AddTwoInts.Request() #6 istek objesi oluşturulur
    request.a = 14 #7 istek objesine değerler atanır
    request.b = 35
    future = client_.call_async(request) #8 istek asenkron olarak gönderilir
    rclpy.spin_until_future_complete(node, future) #9 cevap gelene kadar beklenir
    
    try:
        respose = future.result() #10 cevap alınır
        node.get_logger().info(f"Result of add_two_ints: {request.a} + {request.b} = {respose.sum}") #11 cevap loglanır


    except Exception as e:
        node.get_logger().error("Service call failed %r" % (e,))


    rclpy.shutdown()
        





if __name__ == "__main__":
    main()