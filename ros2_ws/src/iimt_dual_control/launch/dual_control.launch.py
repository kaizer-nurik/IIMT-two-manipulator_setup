from iimt_dual_control.bringup import launch_defaults, launch_setup
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def generate_launch_description():
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in launch_defaults().items()
            ],
            OpaqueFunction(function=launch_setup),
        ]
    )
