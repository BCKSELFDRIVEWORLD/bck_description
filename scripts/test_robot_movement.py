#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class RobotMovementTest(Node):
    def __init__(self):
        super().__init__('robot_movement_test')

        # Publishers ve Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Odometry verileri
        self.current_odom = None
        self.start_odom = None

        # Test parametreleri
        self.test_duration = 2.0  # saniye
        self.tolerance_linear = 0.15  # metre (±15cm tolerans)
        self.tolerance_angular = 0.2  # radyan (±11 derece tolerans)

        self.get_logger().info('Robot Hareket Test Scripti Başlatıldı!')

    def odom_callback(self, msg):
        """Odometry verilerini al"""
        self.current_odom = msg

    def wait_for_odom(self):
        """Odom verisinin gelmesini bekle"""
        self.get_logger().info('Odometry verisi bekleniyor...')
        timeout = 10.0
        start_time = time.time()

        while self.current_odom is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_odom is None:
            self.get_logger().error('Odometry verisi alınamadı! /odom topic çalışıyor mu kontrol edin.')
            return False

        self.get_logger().info('Odometry verisi alındı ✓')
        return True

    def get_position(self, odom_msg):
        """Odometry mesajından pozisyon al"""
        return (
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        )

    def get_yaw(self, odom_msg):
        """Quaternion'dan yaw açısını hesapla"""
        q = odom_msg.pose.pose.orientation
        # Quaternion to Euler (yaw only)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def stop_robot(self):
        """Robotu durdur"""
        stop_msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.1)

    def send_velocity(self, linear_x, angular_z, duration):
        """Belirli bir süre hız komutu gönder"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        end_time = time.time() + duration
        while time.time() < end_time:
            self.cmd_vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)

        self.stop_robot()

    def calculate_distance(self, pos1, pos2):
        """İki nokta arası mesafe"""
        dx = pos2[0] - pos1[0]
        dy = pos2[1] - pos1[1]
        return math.sqrt(dx*dx + dy*dy)

    def test_forward_movement(self):
        """İleri hareket testi"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 1: İLERİ HAREKET')
        self.get_logger().info('='*60)

        # Başlangıç pozisyonunu kaydet
        rclpy.spin_once(self, timeout_sec=0.1)
        start_pos = self.get_position(self.current_odom)

        # İleri hareket komutu gönder
        linear_speed = 0.5  # m/s
        self.get_logger().info(f'Komut: {linear_speed} m/s hızla {self.test_duration} saniye ileri git')
        self.send_velocity(linear_speed, 0.0, self.test_duration)

        # Bekle ve tekrar odom al
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        end_pos = self.get_position(self.current_odom)

        # Mesafeyi hesapla
        traveled_distance = self.calculate_distance(start_pos, end_pos)
        expected_distance = linear_speed * self.test_duration
        error = abs(traveled_distance - expected_distance)

        # Sonuçları göster
        self.get_logger().info(f'Başlangıç pozisyonu: ({start_pos[0]:.3f}, {start_pos[1]:.3f})')
        self.get_logger().info(f'Bitiş pozisyonu: ({end_pos[0]:.3f}, {end_pos[1]:.3f})')
        self.get_logger().info(f'Beklenen mesafe: {expected_distance:.3f} m')
        self.get_logger().info(f'Gidilen mesafe: {traveled_distance:.3f} m')
        self.get_logger().info(f'Hata: {error:.3f} m')

        if error <= self.tolerance_linear:
            self.get_logger().info('✓ TEST BAŞARILI - Robot doğru ileri gidiyor!')
            return True
        else:
            self.get_logger().error(f'✗ TEST BAŞARISIZ - Hata toleranstan ({self.tolerance_linear}m) büyük!')
            return False

    def test_backward_movement(self):
        """Geri hareket testi"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 2: GERİ HAREKET')
        self.get_logger().info('='*60)

        rclpy.spin_once(self, timeout_sec=0.1)
        start_pos = self.get_position(self.current_odom)

        linear_speed = -0.3  # m/s (negatif = geri)
        self.get_logger().info(f'Komut: {abs(linear_speed)} m/s hızla {self.test_duration} saniye geri git')
        self.send_velocity(linear_speed, 0.0, self.test_duration)

        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        end_pos = self.get_position(self.current_odom)

        traveled_distance = self.calculate_distance(start_pos, end_pos)
        expected_distance = abs(linear_speed) * self.test_duration
        error = abs(traveled_distance - expected_distance)

        self.get_logger().info(f'Başlangıç pozisyonu: ({start_pos[0]:.3f}, {start_pos[1]:.3f})')
        self.get_logger().info(f'Bitiş pozisyonu: ({end_pos[0]:.3f}, {end_pos[1]:.3f})')
        self.get_logger().info(f'Beklenen mesafe: {expected_distance:.3f} m')
        self.get_logger().info(f'Gidilen mesafe: {traveled_distance:.3f} m')
        self.get_logger().info(f'Hata: {error:.3f} m')

        if error <= self.tolerance_linear:
            self.get_logger().info('✓ TEST BAŞARILI - Robot doğru geri gidiyor!')
            return True
        else:
            self.get_logger().error(f'✗ TEST BAŞARISIZ - Hata toleranstan ({self.tolerance_linear}m) büyük!')
            return False

    def test_rotation(self):
        """Dönme testi"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 3: DÖNME HAREKETİ')
        self.get_logger().info('='*60)

        rclpy.spin_once(self, timeout_sec=0.1)
        start_yaw = self.get_yaw(self.current_odom)

        angular_speed = 0.5  # rad/s
        self.get_logger().info(f'Komut: {angular_speed} rad/s hızla {self.test_duration} saniye sola dön')
        self.send_velocity(0.0, angular_speed, self.test_duration)

        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        end_yaw = self.get_yaw(self.current_odom)

        # Açı farkını hesapla (-pi, pi aralığında normalize et)
        angle_diff = end_yaw - start_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        expected_angle = angular_speed * self.test_duration
        error = abs(abs(angle_diff) - expected_angle)

        self.get_logger().info(f'Başlangıç açısı: {math.degrees(start_yaw):.1f}°')
        self.get_logger().info(f'Bitiş açısı: {math.degrees(end_yaw):.1f}°')
        self.get_logger().info(f'Beklenen dönüş: {math.degrees(expected_angle):.1f}°')
        self.get_logger().info(f'Gerçek dönüş: {math.degrees(abs(angle_diff)):.1f}°')
        self.get_logger().info(f'Hata: {math.degrees(error):.1f}°')

        if error <= self.tolerance_angular:
            self.get_logger().info('✓ TEST BAŞARILI - Robot doğru dönüyor!')
            return True
        else:
            self.get_logger().error(f'✗ TEST BAŞARISIZ - Hata toleranstan ({math.degrees(self.tolerance_angular):.1f}°) büyük!')
            return False

    def test_combined_movement(self):
        """Birleşik hareket testi (ileri + dönme)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST 4: BİRLEŞİK HAREKET (İleri + Dönme)')
        self.get_logger().info('='*60)

        rclpy.spin_once(self, timeout_sec=0.1)
        start_pos = self.get_position(self.current_odom)
        start_yaw = self.get_yaw(self.current_odom)

        linear_speed = 0.3  # m/s
        angular_speed = 0.3  # rad/s
        self.get_logger().info(f'Komut: İleri {linear_speed} m/s + Dönme {angular_speed} rad/s, {self.test_duration} saniye')
        self.send_velocity(linear_speed, angular_speed, self.test_duration)

        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)
        end_pos = self.get_position(self.current_odom)
        end_yaw = self.get_yaw(self.current_odom)

        traveled_distance = self.calculate_distance(start_pos, end_pos)
        angle_diff = end_yaw - start_yaw

        self.get_logger().info(f'Gidilen mesafe: {traveled_distance:.3f} m')
        self.get_logger().info(f'Dönüş açısı: {math.degrees(angle_diff):.1f}°')

        # Birleşik harekette hem ilerleme hem dönme olmalı
        if traveled_distance > 0.1 and abs(angle_diff) > 0.1:
            self.get_logger().info('✓ TEST BAŞARILI - Robot hem ilerliyor hem dönüyor!')
            return True
        else:
            self.get_logger().error('✗ TEST BAŞARISIZ - Birleşik hareket düzgün çalışmıyor!')
            return False

    def run_all_tests(self):
        """Tüm testleri çalıştır"""
        if not self.wait_for_odom():
            return

        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('ROBOT HAREKET TESTLERİ BAŞLIYOR')
        self.get_logger().info('='*60)

        results = []

        # Test 1: İleri
        time.sleep(1)
        results.append(('İleri Hareket', self.test_forward_movement()))

        # Test 2: Geri
        time.sleep(1)
        results.append(('Geri Hareket', self.test_backward_movement()))

        # Test 3: Dönme
        time.sleep(1)
        results.append(('Dönme', self.test_rotation()))

        # Test 4: Birleşik
        time.sleep(1)
        results.append(('Birleşik Hareket', self.test_combined_movement()))

        # Özet
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('TEST SONUÇLARI ÖZETİ')
        self.get_logger().info('='*60)

        passed = 0
        failed = 0
        for test_name, result in results:
            status = '✓ BAŞARILI' if result else '✗ BAŞARISIZ'
            self.get_logger().info(f'{test_name}: {status}')
            if result:
                passed += 1
            else:
                failed += 1

        self.get_logger().info('='*60)
        self.get_logger().info(f'Toplam: {passed} başarılı, {failed} başarısız')
        self.get_logger().info('='*60 + '\n')

        # Robotu durdur
        self.stop_robot()

def main(args=None):
    rclpy.init(args=args)
    test_node = RobotMovementTest()

    try:
        test_node.run_all_tests()
    except KeyboardInterrupt:
        test_node.get_logger().info('Test kullanıcı tarafından durduruldu.')
    finally:
        test_node.stop_robot()
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
