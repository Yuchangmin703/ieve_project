import os
import subprocess
import yaml

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_camera_info():
    output = subprocess.getoutput('v4l2-ctl --list-devices')
    if "HW40" in output:
        dev = subprocess.getoutput('v4l2-ctl --list-devices | grep -A 1 "HW40" | grep "/dev/video" | head -n 1').strip()
        return "HW40", dev.split()[0] if dev else "/dev/video0"
    elif "USB 2.0 Camera" in output:
        dev = subprocess.getoutput('v4l2-ctl --list-devices | grep -A 1 "USB 2.0 Camera" | grep "/dev/video" | head -n 1').strip()
        return "USB2", dev.split()[0] if dev else "/dev/video0"
    return "UNKNOWN", "/dev/video0"

def generate_launch_description():
    pkg_share = get_package_share_directory('perception')
    cam_type, dev_path = get_camera_info()
    script_path = os.path.join(pkg_share, 'scripts', 'camera_auto_setup.sh')

    yaml_file_name = 'hw40_params.yaml' if cam_type == 'HW40' else 'usb2_params.yaml'
    yaml_path = os.path.join(pkg_share, 'config', yaml_file_name)

    with open(yaml_path, 'r') as f:
        config_data = yaml.safe_load(f)

    bev_cfg = config_data['bev']
    metric_cfg = config_data['metric']
    viz_metric_cfg = config_data['viz_metric']

    global_bev_params = [
        {'bev_x_min': float(bev_cfg['x_min'])},
        {'bev_x_max': float(bev_cfg['x_max'])},
        {'bev_y_min': float(bev_cfg['y_min'])},
        {'bev_y_max': float(bev_cfg['y_max'])},
    ]

    global_res_params = [
        {'width': int(bev_cfg['output_width'])},
        {'height': int(bev_cfg['output_height'])},
    ]

    global_metric_params = [
        {'meters_per_pixel_x': float(metric_cfg['meters_per_pixel_x'])},
        {'meters_per_pixel_y': float(metric_cfg['meters_per_pixel_y'])},
        {'origin_u': float(metric_cfg['origin_u'])},
        {'origin_v': float(metric_cfg['origin_v'])},
        {'bev_origin_to_camera_x_m': float(metric_cfg['bev_origin_to_camera_x_m'])},
        {'bev_origin_to_camera_y_m': float(metric_cfg['bev_origin_to_camera_y_m'])},
        {'camera_to_base_x_m': float(metric_cfg['camera_to_base_x_m'])},
        {'camera_to_base_y_m': float(metric_cfg['camera_to_base_y_m'])},
    ]
    global_viz_metric_params = [
        {'viz_x_min': float(viz_metric_cfg['x_min'])},
        {'viz_x_max': float(viz_metric_cfg['x_max'])},
        {'viz_y_min': float(viz_metric_cfg['y_min'])},
        {'viz_y_max': float(viz_metric_cfg['y_max'])},
    ]

    use_viz_arg = DeclareLaunchArgument('use_viz', default_value='true', description='Enable perception visualization node')
    debug_arg = DeclareLaunchArgument('publish_debug', default_value='true', description='Enable intermediate debug image publishing')

    camera_setup = ExecuteProcess(cmd=['bash', script_path], output='screen')

    nodes_to_start = TimerAction(
        period=2.0,
        actions=[
            # ⭐ 통합된 새 노드 하나만 실행!
            Node(
                package='perception',
                executable='unified_camera_node',
                name='unified_camera_node',
                output='screen',
                parameters=[
                    {'device_path': dev_path},
                    {'camera_type': cam_type},
                    {'fps': 30}
                ]
            ),
            # 마스크 노드
            Node(
		    package='perception',
		    executable='lane_candidate_mask_node',
		    name='lane_candidate_mask_node',
		    output='screen',
		    parameters=[
			{'publish_debug': LaunchConfiguration('publish_debug')},

			{'proc_width': 320},
			{'proc_height': 240},

			{'strict_white_v_min': 105},
			{'loose_white_v_min': 90},
			{'white_s_max': 25},

			{'black_v_min': 0},
			{'black_v_max': 125},

			{'yellow_h_min': 20},
			{'yellow_h_max': 35},
			{'yellow_s_min': 70},
			{'yellow_s_max': 180},
			{'yellow_v_min': 145},
			{'yellow_v_max': 255},

			{'tophat_size': 10},
			{'blast_size': 12},
			{'noise_eraser_size': 3},

			{'yellow_fat_radius': 3},
			{'black_fat_radius': 5},
			{'seed_row_from_bottom': 25},
		    ] + global_bev_params
		),
            # 중심선 추출 노드
            Node(
                package='perception',
                executable='centerline_extractor_node',
                name='centerline_extractor_node',
                output='screen',
                parameters=[
                    {'publish_debug': LaunchConfiguration('publish_debug')},
                    {'frame_id': 'base_link'}, {'y_thresh': 0.25}, {'x_thresh': 0.60}, {'min_lane_pts': 15}
                ] + global_res_params + global_metric_params
            ),
            # 시각화 노드
            Node(
                package='perception',
                executable='perception_viz_node',
                name='perception_viz_node',
                output='screen',
                parameters=global_viz_metric_params,
                condition=IfCondition(LaunchConfiguration('use_viz'))
            ),
        ]
    )

    return LaunchDescription([use_viz_arg, debug_arg, camera_setup, nodes_to_start])
