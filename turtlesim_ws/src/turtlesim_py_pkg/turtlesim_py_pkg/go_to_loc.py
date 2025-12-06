#!/usr/bin/env python3
import sys
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist # Robotu hareket ettirmek için hız mesajı
from turtlesim.msg import Pose    # Robotun konumunu okumak için pozisyon mesajı

class GoToLocationNode(Node): 
    def __init__(self):
        # 1. Node ismini başlatıyoruz
        super().__init__("go_to_location_node") 
        
        # 2. Değişkenlerin Tanımlanması
        self.pose = None # HATA ÖNLEYİCİ: Henüz veri gelmediği için başlangıçta boş atıyoruz
        self.pose_threshold_linear = 0.1 # Hedefe ne kadar yaklaşırsak "vardık" sayacağız? (Tolerans)
        self.pose_threshold_angular = 0.01
        # Terminalden gelen argümanları alıyoruz (Örn: ros2 run ... 5.0 5.0)
        # sys.argv[0] dosya adıdır, o yüzden 1 ve 2. elemanları alıyoruz.
        self.target_x = float(sys.argv[1])  
        self.target_y = float(sys.argv[2]) 
        
        # 3. Publisher (Yayıncı) Oluşturma
        # Kaplumbağaya hız komutlarını göndereceğimiz kanal
        self.publisher_ = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10
        )
        
        # 4. Subscriber (Abone) Oluşturma
        # Kaplumbağanın o an nerede olduğunu simülasyondan dinlediğimiz kanal
        self.subscription_ = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback_pose, # Veri geldiğinde bu fonksiyona git
            10
        )
        
        # 5. Timer (Zamanlayıcı) Oluşturma
        # Kontrol döngüsü saniyede kaç kez çalışacak? (Burada 1 saniye ayarlanmış)
        # Not: Gerçek robotlarda genelde 0.1 veya 0.05 gibi daha seri değerler kullanılır.
        self.timer_ = self.create_timer(1.0, self.turtle_contoller)

        self.get_logger().info(f"Hedefe Gitme Node'u Başladı! Hedef: X={self.target_x}, Y={self.target_y}")
    
    # Subscriber'dan veri geldiğinde çalışan fonksiyon
    def pose_callback_pose(self, msg):
        # Gelen konum mesajını (msg) sınıfın değişkenine kaydediyoruz ki
        # diğer fonksiyonlarda kullanabilelim.
        self.pose = msg
    
    # Ana Kontrol Döngüsü (Beyin Kısmı)
    def turtle_contoller(self):
        # Eğer henüz subscriber'dan veri gelmediyse işlem yapma, bekle.
        if self.pose is None:
            self.get_logger().info("Konum verisi bekleniyor...")
            return

        msg = Twist() # Göndereceğimiz hız mesajı taslağını oluşturuyoruz

        # --- MATEMATİKSEL HESAPLAMALAR ---
        
        # X ve Y eksenindeki farkı bul (Hata Hesaplama)
        dist_x = self.target_x - self.pose.x 
        dist_y = self.target_y - self.pose.y

        # Pisagor teoremi ile kuş bakışı mesafeyi (hipotenüs) bul
        distance = math.sqrt(dist_x**2 + dist_y**2) 
        
        # Hedefe gitmek için robotun bakması gereken açıyı hesapla (Arctan2 fonksiyonu)
        target_theta = math.atan2(dist_y, dist_x) 
        
        # O anki açı ile hedef açı arasındaki fark
        angle_diff = target_theta - self.pose.theta

        # --- HAREKET MANTIĞI (ALGORİTMA) ---

        # Durum 1: Yönümüz hedefe bakmıyorsa, önce dön!
        # (Açı farkı toleranstan büyükse sadece olduğu yerde döner)
        if abs(angle_diff) > self.pose_threshold_angular:
            msg.linear.x = 0.0 # İlerleme durur
            msg.angular.z = angle_diff # Açı farkı kadar dön (Basit P Kontrol)
        
        # Durum 2: Yönümüz hedefe bakıyor, artık ilerleyebiliriz.
        else:
            # Hedefe henüz varmadıysak ilerle
            if distance >= self.pose_threshold_linear:
                msg.linear.x = distance # Mesafeye orantılı hız (Uzaksa hızlı, yakınsa yavaş)
                msg.angular.z = 0.0 # Dönmeyi bırak
            
            # Hedefe vardıysak dur
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.get_logger().info("Hedefe Varıldı!")
                # İsteğe bağlı: exit() diyerek node kapatılabilir.

        # Hesaplanan hız komutunu robota gönder
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args) # ROS 2 iletişimini başlat
    node = GoToLocationNode() # Bizim yazdığımız sınıfı oluştur
    rclpy.spin(node) # Node'u sürekli çalışır halde tut (Ctrl+C basılana kadar)
    rclpy.shutdown() # Çıkışta temizlik yap

if __name__ == "__main__":
    main()