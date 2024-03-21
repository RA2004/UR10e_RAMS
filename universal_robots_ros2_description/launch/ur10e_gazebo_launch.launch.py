import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import (OnProcessStart,OnProcessExit)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
    )

    moveit_config=(
        MoveItConfigsBuilder("my")
        .robot_description(file_path="config/ur10e.urdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .robot_description_semantic(file_path="config/ur10e-test.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    #moveit_config = MoveItConfigsBuilder("name", package_name="my_moveit_config").to_moveit_configs()

    # move_group_node = Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #         {"trajectory_execution.allowed_execution_duration_scaling": 2.0,},
    #         {"publish_robot_description_semantic": True},
    #         {"use_sim_time": True},
    #     ],
    # )

    use_sim_time = {"use_sim_time":True}
    config_dict = moveit_config.to_dict()
    config_dict.update(use_sim_time)
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[config_dict],
    )

    rviz_base = os.path.join(os.path.join(get_package_share_directory("my_moveit_config"),"launch"))
    rviz_empty_config = os.path.join(rviz_base,"moveit_empty.rviz")
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments = ["-d",rviz_empty_config],
        parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
        ]
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    ) 
    
    
    #gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_share_directory("universal_robots_ros2_description"), "share"))

    package_path = os.path.join(
        get_package_share_directory('universal_robots_ros2_description'))
    
    xacro_file = os.path.join(package_path,
                                'urdf',
                                'ur10e.urdf')
    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description':doc.toxml()}


    node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
            #arguments=[xacro_file]
            
    )

    load_joint_state_controller = ExecuteProcess(
        cmd = ['ros2', 'control','load_controller', '--set-state', 'active',
                'joint_state_broadcaster'],
        output = 'screen'
    )

    load_arm_controller = ExecuteProcess(
        cmd = ['ros2', 'control','load_controller', '--set-state', 'active','ur_manipulator_controller'],
        output = 'screen'
    )

    spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=["-topic", "/robot_description", "-entity", "ur10e-test"],
            output='screen'
            
    )

    return LaunchDescription([
        # [move_group_node],
        # generate_moveit_rviz_launch(moveit_config),

        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action =  spawn_entity,
                on_exit = [load_joint_state_controller],
            )
        ),

        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action =  load_joint_state_controller,
                on_exit = [load_arm_controller],
            )
        ),

        
        gazebo,
        #moveit_config,
        #generate_moveit_rviz_launch(moveit_config),
        run_move_group_node,
        rviz_node_tutorial,
        static_tf,
        node_robot_state_publisher,
        spawn_entity,

     ])