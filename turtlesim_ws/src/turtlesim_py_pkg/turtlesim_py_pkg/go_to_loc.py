#!/usr/bin/env python3
import sys
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose    
from turtlesim_interfaces.msg import Turtle  
from turtlesim_interfaces.msg import TurtleArray
from turtlesim_interfaces.srv import CatchTurtle 

class GoToLocationNode(Node): 
    def __init__(self):
        super().__init__("go_to_location_node") 
        
        self.pose = None 
        self.new_turtle_to_catch = None
        
        # Hedef yakalandı mı? (Sürekli servis çağırmamak için bayrak)
        self.catch_request_sent = False 

        # Parametreler
        self.kp_linear = 1.5
        self.kp_angular = 4.0
        self.pose_threshold_linear = 0.1 
        self.pose_threshold_angular = 0.05

        # Publisher & Subscriber
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscription_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback_pose, 10)
        self.new_turtle_subscriber_ = self.create_subscription(TurtleArray, "/new_turtles", self.callback_new_turtles, 10)
        
        # CLIENT: Yöneticiye "Yakaladım" demek için
        self.catch_client_ = self.create_client(CatchTurtle, "/catch_turtle")
        while not self.catch_client_.wait_for_service(1.0):
            self.get_logger().warn("Catch servisi bekleniyor...")

        self.timer_ = self.create_timer(0.01, self.turtle_contoller) # Daha akıcı kontrol için 0.01

        self.get_logger().info("Avcı Robot Hazır!")
    
    def callback_new_turtles(self, msg):
        # Eğer listem boşsa ve yeni kaplumbağa varsa ilkini hedef al
        if len(msg.turtles) > 0 and self.new_turtle_to_catch is None:
            self.new_turtle_to_catch = msg.turtles[0]
            self.catch_request_sent = False # Yeni hedef için bayrağı sıfırla
            self.get_logger().info(f"YENİ HEDEF: {self.new_turtle_to_catch.name}")

    def pose_callback_pose(self, msg):
        self.pose = msg
    
    def turtle_contoller(self):
        # Veriler hazır değilse veya hedef yoksa dur
        if self.pose is None or self.new_turtle_to_catch is None:
            return

        msg = Twist()
        dist_x = self.new_turtle_to_catch.x - self.pose.x
        dist_y = self.new_turtle_to_catch.y - self.pose.y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        target_theta = math.atan2(dist_y, dist_x)
        angle_diff = target_theta - self.pose.theta

        # Açı farkını normalize et (-pi ile +pi arasına sıkıştır)
        # Bu sayede robot 350 derece dönmek yerine tersten 10 derece döner
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        # --- Hareket Mantığı ---
        if distance > self.pose_threshold_linear:
            # Hedefe git
            msg.linear.x = self.kp_linear * distance
            msg.angular.z = self.kp_angular * angle_diff
        else:
            # Hedefe VARILDI
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            # Eğer daha önce istek atmadıysak, şimdi atalım
            if not self.catch_request_sent:
                self.get_logger().info(f"YAKALANDI: {self.new_turtle_to_catch.name}")
                self.call_catch_turtle_service(self.new_turtle_to_catch.name)
                self.catch_request_sent = True 
                # Not: self.new_turtle_to_catch = None yapmıyoruz. 
                # Bunu topic'ten yeni liste gelince zaten güncelleyeceğiz.

        self.publisher_.publish(msg)

    def call_catch_turtle_service(self, turtle_name): 
        request = CatchTurtle.Request()
        request.name = turtle_name
        
        future = self.catch_client_.call_async(request)
        future.add_done_callback(
            lambda future: self.callback_catch_turtle_response(future, turtle_name)
        )

    def callback_catch_turtle_response(self, future, turtle_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"{turtle_name} başarıyla silindi.")
                # Başarılı olunca hedefi boşa çıkar, yeni hedef bekle
                self.new_turtle_to_catch = None
            else:
                self.get_logger().warn(f"{turtle_name} silinemedi!")
        except Exception as e:
            self.get_logger().error("Catch servisi hatası: %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    node = GoToLocationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()