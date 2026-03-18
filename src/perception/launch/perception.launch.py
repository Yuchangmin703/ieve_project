import os
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_camera_info():
    # v4l2-ctl 명령어를 통해 현재 연결된 장치 리스트 확인
    output = subprocess.getoutput('v4l2-ctl --list-devices')
    
    # HW40 우선 탐색
    if "HW40" in output:
        # grep과 head를 이용해 정확한 /dev/videoX 경로 추출
        dev = subprocess.getoutput('v4l2-ctl --list-devices | grep -A 1 "HW40" | grep "/dev/video" | head -n 1').strip()
        # xargs나 strip으로 공백 제거 후 첫 번째 장치만 선택
        dev = dev.split()[0] if dev else "/dev/video0"
        return "HW40", dev
    
    # USB 2.0 Camera 탐색
    elif "USB 2.0 Camera" in output:
        dev = subprocess.getoutput('v4l2-ctl --list-devices | grep -A 1 "USB 2.0 Camera" | grep "/dev/video" | head -n 1').strip()
        dev = dev.split()[0] if dev else "/dev/video0"
        return "USB2", dev
        
    return "UNKNOWN", "/dev/video0"

def generate_launch_description():
    pkg_share = get_package_share_directory('perception')
    cam_type, dev_path = get_camera_info()
    
    # 설정 파일 및 스크립트 경로
    script_path = os.path.join(pkg_share, 'scripts', 'camera_auto_setup.sh')

    # [Action 1] 하드웨어 설정 스크립트 실행 정의
    camera_setup = ExecuteProcess(
        cmd=['bash', script_path],
        output='screen'
    )

    # [Action 2] 노드 실행 정의 (TimerAction으로 감싸서 지연 실행)
    # 스크립트가 실행되고 약 2초 후에 노드들이 뜨도록 설정합니다.
    nodes_to_start = TimerAction(
        period=2.0,  # 2초 대기
        actions=[
            # 1. Undistort Node
            Node(
                package='perception',
                executable='undistort_node',
                name='undistort_node',
                output='screen',
                parameters=[
                    {'device_path': dev_path},
                    {'camera_type': cam_type},
                    {'fps': 30},
                ]
            ),

            # 2. BEV Warp Node
            Node(
                package='perception',
                executable='bev_warp_node',
                name='bev_warp_node',
                output='screen',
                parameters=[
                    {'width': 640}, {'height': 480},
                    {'h_top': 240}, {'h_bottom': 480},
                    {'w_top': 200}, {'w_bottom': 640},
                ]
            ),

            # 3. Lane Candidate Mask Node
            Node(
                package='perception',
                executable='lane_candidate_mask_node',
                name='lane_candidate_mask_node',
                output='screen',
                parameters=[
                    {'publish_debug': True},
                    # 여기서 파라미터를 직접 넘겨줄 수도 있습니다.
                    {'black_v_min': 0},
                    {'black_v_max': 150},
                ]
            ),

            # 4. Centerline Extractor Node
            Node(
                package='perception',
                executable='centerline_extractor_node',
                name='centerline_extractor_node',
                output='screen',
                parameters=[
                    {'frame_id': 'base_link'},
                ]
            ),
        ]
    )

    return LaunchDescription([
        camera_setup,   # 먼저 스크립트 실행 시작
        nodes_to_start  # 2초 뒤에 노드들 실행
    ])
