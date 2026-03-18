import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
import serial

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        
        # ⭐ 핵심: 변수를 먼저 None으로 선언해서 AttributeError를 원천 차단합니다.
        self.ser = None
        
        # [1. 발행자 설정] 아두이노에서 받은 속도를 ROS 2로 전달
        self.speed_pub = self.create_publisher(Float32, '/ego_speed', 10)

        # [2. 구독자 설정] 제어 노드에서 계산한 조향/속도 명령 수신
        self.subscription = self.create_subscription(
            AckermannDriveStamped, '/drive', self.drive_callback, 10)

        # [3. 안전 및 모니터링 변수]
        self.last_drive_msg_time = self.get_clock().now()
        self.watchdog_timeout = 0.5 

        # [4. 타이머 설정] 100Hz (0.01초) 주기로 루프 실행
        self.timer = self.create_timer(0.01, self.main_loop)

        self.get_logger().info("📡 Serial Bridge Node Started. Waiting for ESP32 connection...")

    def drive_callback(self, msg):
        self.last_drive_msg_time = self.get_clock().now()
        self.send_to_arduino(msg.drive.speed, msg.drive.steering_angle)

    def send_to_arduino(self, speed, steer):
        # ⭐ 연결이 안 되어 있으면 전송 시도를 건너뜀 (에러 방지)
        if self.ser is None or not self.ser.is_open:
            return 
            
        data = f"{speed:.2f},{steer:.2f}\n"
        try:
            self.ser.write(data.encode())
        except Exception as e:
            self.get_logger().error(f"Serial Write Error: {e}")
            # 전송 중 에러가 나면 선이 뽑힌 것으로 간주하고 연결 초기화
            self.ser.close()
            self.ser = None

    def main_loop(self):
        # ==========================================
        # ⭐ [1단계] 자동 재연결 로직 (Auto-Reconnect)
        # ==========================================
        if self.ser is None or not self.ser.is_open:
            # 2초에 한 번씩만 화면에 연결 대기 로그를 띄웁니다 (터미널 도배 방지)
            self.get_logger().warn('⏳ ESP32 연결 대기 중... (/dev/ttyUSB0)', throttle_duration_sec=2.0)
            try:
                # 타임아웃 0으로 설정하여 블로킹(멈춤) 현상 방지
                self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0)
                self.get_logger().info("✅ ESP32 Serial Port Connected Successfully!")
            except Exception:
                return # 연결에 실패하면 이번 루프는 여기서 바로 종료하고 다음 틱을 기약함

        # ==========================================
        # ⭐ [2단계] 데이터 수신 및 파싱 (버그 수정됨)
        # ==========================================
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line:
                    try:
                        # 쉼표(,) 쪼개기 없이 아두이노가 보낸 단일 속도값을 바로 실수형(float)으로 변환
                        current_speed = float(line)
                        
                        speed_msg = Float32()
                        speed_msg.data = current_speed
                        self.speed_pub.publish(speed_msg)
                    except ValueError:
                        # 숫자가 아닌 문자("==== Ready for ROS2 ====" 등)는 무시
                        pass
        except OSError:
            # 주행 중 케이블이 물리적으로 뽑혔을 때 프로그램이 죽는 것을 방지
            self.get_logger().error("🔌 Serial cable disconnected unexpectedly!")
            self.ser.close()
            self.ser = None
        except Exception:
            pass 

        # ==========================================
        # [3단계] 통신 워치독 (안전 장치)
        # ==========================================
        now = self.get_clock().now()
        time_diff = (now - self.last_drive_msg_time).nanoseconds / 1e9
        if time_diff > self.watchdog_timeout:
            # 워치독 정지 명령도 실제 포트가 열려 있을 때만 전송
            if self.ser is not None and self.ser.is_open:
                self.send_to_arduino(0.0, 0.0)

# ==========================================
# ROS 2 진입점 (Entry Point)
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 프로그램 종료 시 포트가 열려있다면 안전하게 닫기
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()