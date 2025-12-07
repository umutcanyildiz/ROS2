#!/usr/bin/env python3
import random
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import math
from turtlesim_interfaces.msg import Turtle
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle

class SpawnTurtleNode(Node): 
    def __init__(self):
        super().__init__("spawn_turtle_node") 
        self.name_ = "turtle"
        self.counter_ = 1
        self.new_turtles_ = [] 

        # 1. YAYINCI: Dünyaya "Bakın şu kaplumbağalar var" diyen topic
        self.new_turtle_publisher_ = self.create_publisher(TurtleArray, "new_turtles", 10)     

        # 2. SERVER (SUNUCU): Avcı robotun "Yakaladım!" diye başvuracağı servis
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)   

        # 3. CLIENT (İSTEMCİ) - SPAWN: Turtlesim'e "Kaplumbağa yarat" diyen uç
        self.spawn_client_ = self.create_client(Spawn, "/spawn")
        while not self.spawn_client_.wait_for_service(1.0):
            self.get_logger().warn("Spawn servisi bekleniyor...")

        # 4. CLIENT (İSTEMCİ) - KILL: Turtlesim'e "Kaplumbağayı sil" diyen uç
        self.kill_client_ = self.create_client(Kill, "/kill")
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().warn("Kill servisi bekleniyor...")

        # Timer: 5 saniyede bir kaplumbağa üret
        self.timer_ = self.create_timer(5.0, self.spawn_turtle)

    # --- SERVER KISMI: Avcıdan gelen isteği karşılar ---
    def callback_catch_turtle(self, request, response):
        self.get_logger().info(f"Yakalama isteği geldi: {request.name}")
        # Hemen Turtlesim'in kill servisini çağırıp siliyoruz
        self.call_kill_server(request.name)
        response.success = True
        return response

    def publish_new_turtle(self):
        msg = TurtleArray()
        msg.turtles = self.new_turtles_
        self.new_turtle_publisher_.publish(msg)

    # --- KAPLUMBAĞA YARATMA SÜRECİ ---
    def spawn_turtle(self):
        self.counter_ += 1
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        turtle_name = self.name_ + str(self.counter_)
        self.call_spawn_turtle_server(x, y, theta, turtle_name)

    def call_spawn_turtle_server(self, x, y, theta, turtle_name): 
        request_ = Spawn.Request()
        request_.x = x; request_.y = y; request_.theta = theta; request_.name = turtle_name
        
        # init içinde oluşturduğumuz spawn_client_'ı kullanıyoruz
        future = self.spawn_client_.call_async(request_)
        future.add_done_callback(
            lambda future: self.callback_call_spawn_turtle(future, x, y, theta, turtle_name)
        )

    def callback_call_spawn_turtle(self, future, x, y, theta, turtle_name):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Kaplumbağa doğdu: {response.name}")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x; new_turtle.y = y; new_turtle.theta = theta
                self.new_turtles_.append(new_turtle)
                self.publish_new_turtle() # Güncel listeyi yayınla
        except Exception as e:
            self.get_logger().error("Spawn hatası: %r" % (e,))

    # --- KAPLUMBAĞA SİLME SÜRECİ ---
    def call_kill_server(self, turtle_name): 
        request_ = Kill.Request()
        request_.name = turtle_name
        
        # init içinde oluşturduğumuz kill_client_'ı kullanıyoruz (HATA BURADAYDI)
        future = self.kill_client_.call_async(request_)
        future.add_done_callback(
            lambda future: self.callback_call_kill_turtle(future, turtle_name)
        )

    def callback_call_kill_turtle(self, future, turtle_name):
        try:
            future.result() # Kill servisi boş döner ama hata olup olmadığını kontrol ederiz
            # Listeden silme işlemi
            for (i, turtle) in enumerate(self.new_turtles_):
                if turtle.name == turtle_name:
                    del self.new_turtles_[i]
                    self.publish_new_turtle() # Güncel listeyi yayınla
                    self.get_logger().info(f"Kaplumbağa silindi: {turtle_name}")
                    break
        except Exception as e:
            self.get_logger().error("Kill hatası: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()