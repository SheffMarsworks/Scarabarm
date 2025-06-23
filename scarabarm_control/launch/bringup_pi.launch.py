from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ODrive CAN 드라이버
        Node(
            package='odrive_can',          # ← 패키지·실행기 이름 확인됨
            executable='odrive_can_node',
            name='odrive_can',
            output='screen',
            parameters=[{'can_iface': 'can0'}],
        ),

        # FollowJointTrajectory 액션 서버
        Node(
            package='scarabarm_control',
            executable='scarab_controller',
            name='scarab_controller',
            output='screen',
        ),

        # JointState 집계 노드
        Node(
            package='scarabarm_control',
            executable='scarab_js_agg',
            name='scarab_js_agg',
            output='screen',
        ),
    ])
