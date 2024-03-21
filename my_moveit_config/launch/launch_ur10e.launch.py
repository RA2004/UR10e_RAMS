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
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'),'launch'),'/gazebo.launch.py']),
    )

    # moveit_config=(
    #     MoveItConfigsBuilder("my")
    #     .robot_description(file_path="config/ur10e.urdf")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .robot_description_kinematics(file_path="config/kinematics.yaml")
    #     .robot_description_semantic(file_path="config/ur10e-test.srdf")
    #     .planning_scene_monitor(
    #         publish_robot_description=True, publish_robot_description_semantic=True
    #     )
    #     .planning_pipelines(pipelines=["ompl"])
    #     .to_moveit_configs()
    # )
    
    moveit_config = MoveItConfigsBuilder("ur10e-test", package_name="my_moveit_config").to_moveit_configs()

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


    # rviz_base = os.path.join(os.path.join(get_package_share_directory("my_moveit_config"),"launch"))
    # rviz_empty_config = os.path.join(rviz_base,"moveit_empty.rviz")
    # rviz_node_tutorial = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments = ["-d",rviz_empty_config],
    #     parameters=[
    #             moveit_config.robot_description,
    #             moveit_config.robot_description_semantic,
    #             moveit_config.robot_description_kinematics,
    #             moveit_config.planning_pipelines,
    #     ]
    # 
    
    
    #gazebo_env = SetEnvironmentVariable("GAZEBO_MODEL_PATH", os.path.join(get_package_share_directory("universal_robots_ros2_description"), "share"))

    package_path = os.path.join(
        get_package_share_directory('my_moveit_config'))
    
    xacro_file = os.path.join(package_path,
                                'config',
                                'ur10e.urdf')
    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description':doc.toxml()}


    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher",
        output='screen',
        parameters=[params]                     
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    ) 


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "/robot_description", "-entity", "ur10e-test"],
        output='screen'
            
    )

    # joint_state_broadcaster_spawner = Node(
    # package="controller_manager",
    # executable="spawner",
    # arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    joint_trajectory_controller = Node(
           package="controller_manager",
           executable="spawner",    
            arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )


    controllers = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[xacro_file, "/home/rajammu/ros2/test_ws/src/my_moveit_config/config/ros2_controllers.yaml"]
    )

    # moveit_controllers = Node(
    # package="controller_manager",
    # executable="ros2_control_node",
    # parameters=[params, "/home/rajammu/ros2/test_ws/src/my_moveit_config/config/moveit_controllers.yaml"]
    # )
    # load_joint_state_controller = ExecuteProcess(
    #     cmd = ['ros2', 'control','load_controller', '--set-state', 'active',
    #             'joint_state_broadcaster'],
    #     output = 'screen'
    # )

    # load_arm_controller = ExecuteProcess(
    #      cmd = ['ros2', 'control','load_controller', '--set-state', 'active','ur_manipulator_controller'],
    #      output = 'screen'
    # )  



    node_joint_state_publisher = Node(
        package = "joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher"
    )

    



    return LaunchDescription([

        gazebo,
        spawn_entity,
        node_robot_state_publisher,
        node_joint_state_publisher,
        static_tf,
        # joint_state_broadcaster_spawner,
        # joint_trajectory_controller,
        
        #controllers,
        #moveit_controllers,
        generate_demo_launch(moveit_config),
        #generate_move_group_launch(moveit_config),
        #generate_moveit_rviz_launch(moveit_config),

        RegisterEventHandler(
            event_handler = OnProcessExit(
                target_action =  spawn_entity,
                on_exit = [controllers],
            )
        ),

        # RegisterEventHandler(
        #     event_handler = OnProcessExit(
        #         target_action =  controllers,
        #         on_exit = [joint_trajectory_controller],
        #     )
        # ),


        # controllers,
        
        #load_arm_controller,

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        # [move_group_node],
        
        # #run_move_group_node,
        #generate_moveit_rviz_launch(moveit_config),
        
        # RegisterEventHandler(
        #     event_handler = OnProcessExit(
        #         target_action =  generate_move_group_launch(moveit_config),
        #         on_exit = [generate_moveit_rviz_launch(moveit_config)],
        #     )
        # ),
        

        

        # joint_state_broadcaster,
        # joint_trajectory_controller,
        # generate_move_group_launch(moveit_config),
        # generate_moveit_rviz_launch(moveit_config),
        # node_joint_state_publisher,

        #moveit_config,
        #generate_moveit_rviz_launch(moveit_config),
        #run_move_group_node,
        #rviz_node_tutorial,
        
        
        #spawn_entity,

     ])