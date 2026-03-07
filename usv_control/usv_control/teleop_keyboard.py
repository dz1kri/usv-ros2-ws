#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty

# Pesan instruksi untuk user
msg = """
=================================
   Kontrol Perahu USV
=================================
Gunakan keyboard:

   W = Maju
   S = Mundur
   A = Belok Kiri
   D = Belok Kanan
   
   SPACE = Berhenti
   CTRL+C = Keluar

=================================
"""

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Buat publisher untuk thruster kiri dan kanan
        self.left_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/left/thrust', 
            10
        )
        self.right_pub = self.create_publisher(
            Float64, 
            '/wamv/thrusters/right/thrust', 
            10
        )
        
        # Variabel untuk menyimpan kekuatan thruster
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.thrust_increment = 10.0  # Kenaikan thrust per tombol
        
        print(msg)
        
        # Simpan setting terminal
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        """Fungsi untuk membaca input keyboard"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        """Fungsi utama untuk menjalankan kontrol"""
        try:
            while True:
                key = self.get_key()
                
                # Proses tombol yang ditekan
                if key == 'w' or key == 'W':
                    # Maju: tambah thrust kedua motor
                    self.left_thrust += self.thrust_increment
                    self.right_thrust += self.thrust_increment
                    print("⬆️  MAJU")
                    
                elif key == 's' or key == 'S':
                    # Mundur: kurangi thrust kedua motor
                    self.left_thrust -= self.thrust_increment
                    self.right_thrust -= self.thrust_increment
                    print("⬇️  MUNDUR")
                    
                elif key == 'a' or key == 'A':
                    # Belok kiri: kurangi thrust kiri, tambah thrust kanan
                    self.left_thrust -= self.thrust_increment
                    self.right_thrust += self.thrust_increment
                    print("⬅️  BELOK KIRI")
                    
                elif key == 'd' or key == 'D':
                    # Belok kanan: tambah thrust kiri, kurangi thrust kanan
                    self.left_thrust += self.thrust_increment
                    self.right_thrust -= self.thrust_increment
                    print("➡️  BELOK KANAN")
                    
                elif key == ' ':
                    # Berhenti: set thrust ke 0
                    self.left_thrust = 0.0
                    self.right_thrust = 0.0
                    print("⏹️  BERHENTI")
                    
                elif key == '\x03':  # CTRL-C
                    print("\n👋 Keluar dari program...")
                    break
                
                # Batasi nilai thrust antara -100 sampai 100
                self.left_thrust = max(min(self.left_thrust, 100), -100)
                self.right_thrust = max(min(self.right_thrust, 100), -100)
                
                # Buat message dan publish
                left_msg = Float64()
                right_msg = Float64()
                left_msg.data = self.left_thrust
                right_msg.data = self.right_thrust
                
                self.left_pub.publish(left_msg)
                self.right_pub.publish(right_msg)
                
                # Tampilkan status
                print(f"Thrust -> Kiri: {self.left_thrust:.1f} | Kanan: {self.right_thrust:.1f}")
                
        except Exception as e:
            print(f"Error: {e}")
        finally:
            # Kembalikan setting terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
