from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    path_simulator = "/home/aymeric/simulator" #TODO args

    # get stonefish launch file
    stonefish_path = os.path.join(
        FindPackageShare("stonefish_ros2").find("stonefish_ros2"),
        "launch",
        "stonefish_simulator.launch.py"
    )

    # get Ardupilot ROS2 bridge launch file
    ardupilotros2_path = os.path.join(
        FindPackageShare("ardupilotstonefish").find("ardupilotstonefish"),
        "launch",
        "parser_launch.yaml"
    )

    # create v4l2 loopback commands HAS TO BE DONE NOT HERE
    #v4l2_cmd = """
    #    echo "Creating v4l2 loopback camera:";
    #    sudo modprobe -r v4l2loopback;
    #    sudo modprobe v4l2loopback devices=2 video_nr=42,43 card_label="VirtualCam42,VirtualCam43" exclusive_caps=1,1;
    #"""

    # get v4l2 ROS2 package launch file
    v4l2_path = os.path.join(
        FindPackageShare("v4l2ros2").find("v4l2ros2"),
        "launch",
        "video_launch.py"
    )

    # raytracer bash command
    raytracer_cmd = f"""
        echo "Starting raytracer:";
        {path_simulator}/colcon_ws/install/raytracer/lib/raytracer/raytracer;
    """

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stonefish_path),
            launch_arguments={"simulation_data": f"{path_simulator}/data/other/data",
                              "scenario_desc": f"{path_simulator}/data/other/scenarios/bluerov_heavy_simplified_shipwreck.scn",
                              "window_res_x": "800",
                              "window_res_y": "600",
                              "simulation_rate": "1000"}.items()
        ),  
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(ardupilotros2_path),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(v4l2_path),
        ),
        ExecuteProcess(
            cmd=['bash', '-c', raytracer_cmd],
            shell=True,
            output='screen'
        )
    ])
