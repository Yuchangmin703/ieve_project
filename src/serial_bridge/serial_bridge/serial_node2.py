import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.ser = None
        self.speed_pub = self.create_publisher(Float32, '/ego_speed', 10)

        # 큐 사이즈 1 (최신 데이터만 받음)
        self.subscription = self.create_subscription(
            AckermannDriveStamped, '/drive', self.drive_callback, 1)

        self.last_drive_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.main_loop) # 50Hz 
        self.rx_buffer = b''
        
        # 중간 데이터 스킵용 변수
        self.latest_speed = 0.0
        self.latest_steer = 0.0
        self.has_new_cmd = False 

    def drive_callback(self, msg):
        self.last_drive_msg_time = self.get_clock().now()
        self.latest_speed = max(-0.4, min(0.4, msg.drive.speed))
        self.latest_steer = msg.drive.steering_angle
        self.has_new_cmd = True 

    def main_loop(self):
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn('⏳ ESP32 연결 대기 중...', throttle_duration_sec=2.0)
            try:
                # C8: write_timeout 설정 — write()가 블로킹되어 제어 체인이 멈추는 것 방지
                self.ser = serial.Serial('/dev/ttyCH341USB0', 115200, timeout=0, write_timeout=0.1)
                self.rx_buffer = b''
                self.get_logger().info("✅ ESP32 연결 성공!")
            except Exception:
                return

        # 모아둔 최신 조이스틱 명령 1개만 발사
        if self.has_new_cmd and self.ser is not None and self.ser.is_open:
            data = f"{self.latest_speed:.2f},{self.latest_steer:.2f}\n"
            try:
                self.ser.write(data.encode('ascii'))
                self.has_new_cmd = False 
            except Exception:
                self.ser.close()
                self.ser = None

        # 아두이노 데이터 수신 파싱
        try:
            if self.ser is not None and self.ser.in_waiting > 0:
                self.rx_buffer += self.ser.read(self.ser.in_waiting)
                # M1: rx_buffer 무한 증가 방지 — newline 없는 데이터가 계속 들어올 경우 OOM 예방
                if len(self.rx_buffer) > 1024:
                    self.rx_buffer = b''
                if b'\n' in self.rx_buffer:
                    lines = self.rx_buffer.split(b'\n')
                    self.rx_buffer = lines[-1]
                    if len(lines) > 1:
                        latest_line = lines[-2].decode('ascii', errors='ignore').strip()
                        if latest_line:
                            try:
                                speed_val = float(latest_line)
                                # M2: ESP32 비정상 속도값 필터링 (±10m/s 초과 시 무시)
                                if -10.0 < speed_val < 10.0:
                                    speed_msg = Float32()
                                    speed_msg.data = speed_val
                                    self.speed_pub.publish(speed_msg)
                            except ValueError:
                                pass
        except Exception:
            pass 

        # 워치독
        now = self.get_clock().now()
        time_diff = (now - self.last_drive_msg_time).nanoseconds / 1e9
        if time_diff > 0.5:
            if self.ser is not None and self.ser.is_open:
                try:
                    self.ser.write(b"0.00,0.00\n")
                except:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.write(b"0.00,0.00\n")
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
