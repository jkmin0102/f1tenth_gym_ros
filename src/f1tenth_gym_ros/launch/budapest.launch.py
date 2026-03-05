from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_yaml = "/sim_ws/src/f1tenth_gym_ros/config/sim.yaml"
    map_yaml = "/sim_ws/src/f1tenth_gym_ros/maps/Budapest/Budapest_map.yaml"
    rviz_cfg = "/sim_ws/src/f1tenth_gym_ros/config/budapest.rviz"

    xacro_file = PathJoinSubstitution([
        FindPackageShare("f1tenth_gym_ros"),
        "launch",
        "ego_racecar.xacro",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="ego_robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", xacro_file]),
            "use_tf_static": True,
        }],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml}],
    )

    # map_server를 항상 확실히 활성화(타이밍 문제 해결)
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["map_server"],
            "bond_timeout": 0.0,
        }],
    )

    gym_bridge = Node(
        package="f1tenth_gym_ros",
        executable="gym_bridge",
        name="bridge",          # sim.yaml top key 'bridge'와 매칭
        output="screen",
        parameters=[sim_yaml],
    )

    odom_to_tf = Node(
        package="control",
        executable="odom_to_tf",
        name="odom_to_tf",
        output="screen",
        parameters=[{
            "odom_topic": "/ego_racecar/odom",
            "parent_frame": "map",
            "child_frame": "ego_racecar/base_link",
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription([
        robot_state_publisher,
        map_server,
        lifecycle_manager,
        gym_bridge,
        odom_to_tf,
        rviz,
    ])
