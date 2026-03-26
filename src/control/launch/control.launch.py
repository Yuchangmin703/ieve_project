import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. 하드웨어(조이스틱) 신호를 읽어오는 기본 joy 노드
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # 2. 조이스틱 신호를 차량 제어 명령(AckermannDrive)으로 변환하는 노드
        Node(
            package='control',
            executable='joy_drive_node',
            name='joy_drive_node',
            output='screen'
        ),
        
        # 3. 조이스틱/자율주행 모드를 선택하고 영점(Trim)을 보정하는 Mux 노드
        Node(
            package='control',
            executable='drive_mux_node',
            name='drive_mux_node',
            output='screen'
        )
    ])