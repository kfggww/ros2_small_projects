from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    model_urdf = PathJoinSubstitution(
        [FindPackageShare("moving_car"), "urdf/moving_car.urdf"]
    )

    empty_world = PathJoinSubstitution(
        [FindPackageShare("moving_car"), "worlds/empty.sdf"]
    )

    urdf_preprocess = ExecuteProcess(
        cmd=["xacro ", model_urdf, " > /tmp/moving_car.urdf"],
        name="generate_urdf",
        shell=True,
        log_cmd=True,
    )

    start_ign_gazebo = ExecuteProcess(
        cmd=["ign gazebo ", empty_world], name="ign_gazebo", shell=True
    )

    spawn_urdf_model = ExecuteProcess(
        cmd=[
            'ign service -s /world/empty/create --reqtype ignition.msgs.EntityFactory --reptype ignition.msgs.Boolean --timeout 3000 --req \'sdf_filename: "/tmp/moving_car.urdf", name: "small_car"\''
        ],
        name="load_model_urdf",
        shell=True,
    )

    set_model_pose = ExecuteProcess(
        cmd=[
            "ign service -s /world/empty/set_pose --reqtype ignition.msgs.Pose --reptype ignition.msgs.Boolean --timeout 6000 --req 'name: \"small_car\", position: {x: 0, y: 0, z: 0.03}, orientation: {x: 0, y: 0, z: 1, w: 0.8}'"
        ],
        name="set_model_pose",
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

    cmd_vel_topic_bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=["/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist"],
    )

    car_controller = Node(package="moving_car", executable="car_controller")

    return LaunchDescription(
        [
            urdf_preprocess,
            start_ign_gazebo,
            spawn_urdf_model,
            set_model_pose,
            lidar_topic_bridge,
            camera_topic_bridge,
            cmd_vel_topic_bridge,
            car_controller,
        ]
    )
