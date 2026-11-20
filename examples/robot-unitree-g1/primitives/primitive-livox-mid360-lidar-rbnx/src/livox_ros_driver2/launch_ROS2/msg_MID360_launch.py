import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
# rbnx vendored default: 0 = ROS2 sensor_msgs/PointCloud2 (PointXYZRTLT).
# Override at runtime by exporting LIVOX_XFER_FORMAT before `ros2 launch`.
xfer_format   = int(os.environ.get('LIVOX_XFER_FORMAT', '0'))
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = float(os.environ.get('LIVOX_PUBLISH_FREQ', '10.0'))
output_type   = 0
frame_id      = os.environ.get('LIVOX_FRAME_ID', 'livox_frame')
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
# Optional path from env (rbnx / prepare_livox_config.sh) so host_net_info matches this host
user_config_path = os.environ.get(
    'LIVOX_MID360_CONFIG',
    os.path.join(cur_config_path, 'MID360_config.json'),
)
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver,
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=livox_rviz,
        #         on_exit=[
        #             launch.actions.EmitEvent(event=launch.events.Shutdown()),
        #         ]
        #     )
        # )
    ])
