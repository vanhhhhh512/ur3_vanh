"""Dung toan bo workcell mo phong: Gazebo + UR3e + controllers + MoveIt + RViz.

Day la launch file chay o terminal thu nhat. No khong chua logic cua bai tap,
chi ghep cac launch chinh thuc cua Universal Robots lai voi world rieng cua
workcell (ban thao tac, ba khoi, ba vung dat).

Cac include duoc boc trong GroupAction de moi launch con giu duoc launch
argument cua rieng no; neu khong RViz va move_group se khong nhan dung tham so.

Rieng RViz cua launch mo phong UR: no chi duoc tao sau khi spawner
joint_state_broadcaster ket thuc, luc do GroupAction da dong nen dieu kien
IfCondition(launch_rviz) doc gia tri launch_rviz CHUNG (mac dinh true) chu
khong phai "false" ta truyen vao, va mo them mot cua so RViz trong. Vi vay
gia tri launch_rviz cua nguoi dung duoc doc ra truoc, roi launch_rviz chung
bi dat han ve false.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _dung_workcell(context):
    ur_type = LaunchConfiguration("ur_type")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    # Doc ngay gia tri nguoi dung chon, truoc khi launch_rviz chung bi dat false
    mo_rviz = LaunchConfiguration("launch_rviz").perform(context)

    world_cua_bai = PathJoinSubstitution(
        [FindPackageShare("ur3_llm_control"), "worlds", "ur3_workcell.sdf"])

    # Gazebo + robot + ros2_control. RViz cua launch nay bi tat vi ta dung
    # cau hinh RViz rieng di kem move_group o duoi.
    mo_phong = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "world_file": world_cua_bai,
            "gazebo_gui": gazebo_gui,
            "launch_rviz": "false",
            "initial_joint_controller": "joint_trajectory_controller",
            # Dung file controller rieng cua bai tap (nguong bam quy dao noi long)
            "runtime_config_package": "ur3_llm_control",
            "controllers_file": "ur3_controllers.yaml",
            # URDF rieng: UR3e + tay kep hai ngon gan vao tool0
            "description_package": "ur3_llm_control",
            "description_file": "ur3e_tay_kep.urdf.xacro",
        }.items(),
    )

    # move_group cua MoveIt 2 cho UR, kem RViz voi cau hinh rieng cua workcell
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": ur_type,
            "safety_limits": "true",
            "use_sim_time": "true",
            # RViz nay luon mo <moveit_config_package>/rviz/view_robot.rviz (launch
            # cua UR khong dung tham so rviz_config_file), tuc la file
            # rviz/view_robot.rviz cua goi nay.
            "launch_rviz": mo_rviz,
            # Cung URDF co tay kep, va SRDF co them cac cap link cua tay kep.
            # Goi nay chua ban sao kinematics.yaml / joint_limits.yaml cua
            # ur_moveit_config vi launch cua UR tim chung trong moveit_config_package.
            "description_package": "ur3_llm_control",
            "description_file": "ur3e_tay_kep.urdf.xacro",
            "moveit_config_package": "ur3_llm_control",
            "moveit_config_file": "ur3e_tay_kep.srdf.xacro",
            "moveit_joint_limits_file": "moveit_joint_limits.yaml",
        }.items(),
    )

    # Launch mo phong cua UR chi bat controller cua tay may; controller hai
    # ngon kep duoc bat rieng sau khi controller_manager da len.
    bat_tay_kep = TimerAction(period=8.0, actions=[Node(
        package="controller_manager",
        executable="spawner",
        arguments=["tay_kep_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
    )])

    return [
        SetLaunchConfiguration("launch_rviz", "false"),
        GroupAction([mo_phong]),
        GroupAction([moveit]),
        bat_tay_kep,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ur_type", default_value="ur3e",
                              description="ur3 hoac ur3e"),
        DeclareLaunchArgument("gazebo_gui", default_value="true",
                              description="Mo cua so Gazebo (tat khi chay kiem thu khong man hinh)"),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Mo RViz"),
        OpaqueFunction(function=_dung_workcell),
    ])
