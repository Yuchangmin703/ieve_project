import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
import time

class DummyTrackSimNode(Node):
    def __init__(self):
        super().__init__('dummy_track_sim')
        self.path_pub = self.create_publisher(Path, '/planning/local_path', 10)
        
        self.track_length_x = 5.0  
        self.track_length_y = 2.0  
        self.num_points = 100      
        
        self.timer = self.create_timer(0.1, self.publish_track)
        self.get_logger().info('🏁 [Dummy Sim] 바닥에 밀착된 타원형 트랙 생성 시작!')

    def publish_track(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map' 

        current_time = time.time()
        time_offset = current_time * 0.2 

        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header = path_msg.header
            
            t = (i / self.num_points) * 2.0 * math.pi + time_offset
            
            # 🌟 수정 1: 타원이 차량 시작점(0,0)을 지나도록 X축을 이동시킵니다.
            pose.pose.position.x = self.track_length_x * math.cos(t) - self.track_length_x
            pose.pose.position.y = self.track_length_y * math.sin(t)
            
            # 🌟 수정 2: Z값을 0으로 줘서 RViz 격자점(바닥)에 딱 붙게 만듭니다!
            pose.pose.position.z = 0.0
            
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyTrackSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
