


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare arguments
    declare_launch_arguments = []
    declare_launch_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time",
        )
    )
    declare_launch_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz",
        )
    )
    
    


    return LaunchDescription([

    ])