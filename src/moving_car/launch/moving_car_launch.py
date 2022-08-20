from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    model_urdf = PathJoinSubstitution(
        [FindPackageShare("moving_car"), "urdf/moving_car.urdf"]
    )

    # TODO: 这里处理urdf文件的处理不够优雅
    urdf_preprocess = ExecuteProcess(
        cmd=["xacro ", model_urdf, " > /tmp/moving_car.urdf"],
        name="generate_urdf",
        shell=True,
        log_cmd=True,
    )

    start_ign_gazebo = ExecuteProcess(
        cmd=["ign gazebo empty.sdf"], name="ign_gazebo", shell=True
    )

    spawn_urdf_model = ExecuteProcess(
        cmd=[
            'ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 1000 --req \'sdf_filename: "/tmp/moving_car.urdf", name: "mvcar"\''
        ],
        name="load_model_urdf",
        shell=True,
    )

    lidar_topic_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan"],
    )

    camera_topic_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/car_camera_raw@sensor_msgs/msg/Image[ignition.msgs.Image"],
    )

    return LaunchDescription(
        [
            urdf_preprocess,
            start_ign_gazebo,
            spawn_urdf_model,
            lidar_topic_bridge,
            camera_topic_bridge,
        ]
    )
