import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(package="hw1", executable="pub", name="pub"),
            launch_ros.actions.Node(package="hw1", executable="sub", name="sub"),
        ]
    )
