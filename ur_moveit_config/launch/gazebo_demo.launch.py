import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit,OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
from pathlib import Path

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_args",
            default_value="-r -v 4 empty.sdf",
            description="Arguments for Gazebo",
        )
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[
            str(Path(get_package_share_directory("ur5e_description")).parent.resolve())
            ]
        )
    
    
    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")


    joint_controllers_file = os.path.join(
        get_package_share_directory("ur5e_description"), "config", "ur_controllers.yaml"
    )


    moveit_config = (
        MoveItConfigsBuilder("custom_robot", package_name="ur_moveit_config")
        .robot_description(file_path="config/ur.urdf.xacro")
        .robot_description_semantic(file_path="config/ur.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_planning_scene=True
        )
        .planning_pipelines(
            pipelines=[
                "ompl",
                "chomp",
                "pilz_industrial_motion_planner"
            ]
        )
        .to_moveit_configs()
    )

    x_arg = DeclareLaunchArgument(
        name="x",
        default_value="0.0",
        description="X position of the robot in the world frame"
    )
    y_arg = DeclareLaunchArgument(
        name="y",
        default_value="0.0",
        description="Y position of the robot in the world frame"
    )  
    z_arg = DeclareLaunchArgument(
        name="z",
        default_value="0.0",
        description="Z position of the robot in the world frame"
    ) 

    # # Include Gazebo launch file
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(gazebo_launch_file),
    #     launch_arguments={
    #         "use_sim_time": "true",
    #         "debug": "false",
    #         "gui" : 'true',
    #         "paused": "true",
    #         # "world": "empty.world",
    #     }.items(),
    # )

    # Start Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
            "/gz_sim.launch.py"
        ]),
        launch_arguments=[("gz_args", gz_args)]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('ur_moveit_config'),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen',
        arguments=[
            "-d", rviz_config_path,
            {"use_sim_time": use_sim_time}
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ]
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "ur5e_robot",
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}]
    )


    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description, 
            joint_controllers_file, 
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output='screen',

    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description
        ],
        output='screen'
    )

    joint_state_broadcaster_spwaner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    
    joint_trajectory_controller_spwaner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )


    robotiq_activation_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "robotiq_activation_controller", 
            "--controller-manager", "/controller_manager",
            "--activate" 
        ],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    config_dict = moveit_config.to_dict()

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            config_dict,
            {"use_sim_time": use_sim_time},
            {"publish_robot_description_semantic": True}
        ],
        arguments=["--ros-args", "--log-level", "info"]
    )

    delay_controller_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                controller_manager_node
            ]
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                joint_state_broadcaster_spwaner
            ]
        )
    )

    delay_joint_trajectory_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spwaner,
            on_start=[
                joint_trajectory_controller_spwaner
            ]
        )
    )

    delay_robotiq_activation = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spwaner,
            on_start=[
                robotiq_activation_controller_spawner
            ]
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_trajectory_controller_spwaner,
            on_start=[
                gripper_controller_spawner
            ]
        )
    )

    delay_rviz_node = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                rviz_node
            ]
        )
    )

    return LaunchDescription(declared_arguments + [
        gazebo_resource_path,
        x_arg,
        y_arg,
        z_arg,
        gazebo,
        gz_ros2_bridge,
        delay_controller_manager,
        spawn_entity,
        robot_state_publisher,
        move_group_node,
        delay_joint_state_broadcaster,
        delay_joint_trajectory_controller,
        delay_robotiq_activation,
        delay_gripper_controller,
        delay_rviz_node
    ])
