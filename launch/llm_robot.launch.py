"""Chay node dieu khien bang ngon ngu tu nhien.

Day la launch file chay o terminal thu hai, sau khi workcell.launch.py da len.
Node se goi LLM qua 9Router, kiem tra ke hoach roi giao cho MoveIt thuc thi.

Bien moi truong bat buoc: NINEROUTER_API_KEY
Bien tuy chon      : NINEROUTER_BASE_URL, NINEROUTER_MODEL
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lenh = LaunchConfiguration("lenh")
    world = LaunchConfiguration("world")
    cho_nut_bam = LaunchConfiguration("cho_nut_bam")

    goi = FindPackageShare("ur3_llm_control")

    node = Node(
        package="ur3_llm_control",
        executable="nut_dieu_khien",
        name="nut_dieu_khien_llm",
        output="screen",
        emulate_tty=True,
        parameters=[{"use_sim_time": True}],
        arguments=[
            "--cau-hinh", PathJoinSubstitution([goi, "config"]),
            "--prompt", PathJoinSubstitution([goi, "prompt"]),
            "--world", world,
            "--lenh", lenh,
            "--cho-nut-bam-neu", cho_nut_bam,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("lenh", default_value="",
                              description="Chay mot cau lenh roi thoat; de trong la vao che do go tay"),
        DeclareLaunchArgument("world", default_value="ur3_workcell",
                              description="Ten world trong Gazebo, phai khop voi file sdf"),
        DeclareLaunchArgument("cho_nut_bam", default_value="false",
                              description="true de dung truoc moi skill cho toi khi bam Next trong RViz"),
        node,
    ])
