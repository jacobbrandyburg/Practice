import os

import ament_index_python.packages
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.actions
import launch_ros

import yaml


def generate_launch_description():
    #Kobuki_Node

    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    kobuki_ros_node = launch_ros.actions.Node(
	package='kobuki_node',
       	executable='kobuki_ros_node',
        output='both',
        parameters=[params]
    )

   #Turtlebot Description

    pkg_share = launch_ros.substitutions.FindPackageShare(package='turtlebot_description').find('turtlebot_description')
    default_model_path = os.path.join(pkg_share, 'robots/kobuki_hexagons_hokuyo.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    #DepthImage_To_Laserscan

    param_config = os.path.join(
        get_package_share_directory('depthimage_to_laserscan'), 'cfg', 'param.yaml')

    depthimage_to_laserscan = launch_ros.actions.Node(
        package='depthimage_to_laserscan',
	executable='depthimage_to_laserscan_node',
	name='depthimage_to_laserscan',
	remappings=[('depth', '/camera/depth/image_raw'), #Changed image_rect_raw to image_raw
                        ('depth_camera_info', '/camera/depth/camera_info')],
	parameters=[param_config])
    )


    return launch.LaunchDescription([

	#Turtlebot Description

	launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),

	#Astra Camera converted from xml to python

	launch.actions.DeclareLaunchArgument(name='camera_name', default_value='camera'),
	launch.actions.DeclareLaunchArgument(name='depth_registration', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='serial_number', default_value=''),
	launch.actions.DeclareLaunchArgument(name='device_num', default_value='1'),
	launch.actions.DeclareLaunchArgument(name='vendor_id', default_value='0x2bc5'),
	launch.actions.DeclareLaunchArgument(name='product_id', default_value=''),
	launch.actions.DeclareLaunchArgument(name='enable_colored_point_cloud', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='point_cloud_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='connection_delay', default_value='100'),
	launch.actions.DeclareLaunchArgument(name='color_width', default_value='640'),
	launch.actions.DeclareLaunchArgument(name='color_height', default_value='480'),
	launch.actions.DeclareLaunchArgument(name='color_fps', default_value='30'),
	launch.actions.DeclareLaunchArgument(name='enable_color', default_value='True'),
	launch.actions.DeclareLaunchArgument(name='flip_color', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='color_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='color_camera_info_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='depth_width', default_value='640'),
	launch.actions.DeclareLaunchArgument(name='depth_height', default_value='480'),
	launch.actions.DeclareLaunchArgument(name='depth_fps', default_value='30'),
	launch.actions.DeclareLaunchArgument(name='enable_depth', default_value='True'),
	launch.actions.DeclareLaunchArgument(name='flip_depth', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='depth_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='depth_camera_info_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='ir_width', default_value='640'),
	launch.actions.DeclareLaunchArgument(name='ir_height', default_value='480'),
	launch.actions.DeclareLaunchArgument(name='ir_fps', default_value='30'),
	launch.actions.DeclareLaunchArgument(name='enable_ir', default_value='True'),
	launch.actions.DeclareLaunchArgument(name='flip_ir', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='ir_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='ir_camera_info_qos', default_value='default'),
	launch.actions.DeclareLaunchArgument(name='publish_tf', default_value='True'),
	launch.actions.DeclareLaunchArgument(name='tf_publish_rate', default_value='10.0'),
	launch.actions.DeclareLaunchArgument(name='ir_info_url', default_value=''),
	launch.actions.DeclareLaunchArgument(name='color_depth_synchronization', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='oni_log_level', default_value='verbose'),
	launch.actions.DeclareLaunchArgument(name='oni_log_to_console', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='oni_log_to_file', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='enable_d2c_viewer', default_value='False'),
	launch.actions.DeclareLaunchArgument(name='enable_publish_extrinsic', default_value='False'),

	


	kobuki_ros_node,
	joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
	depthimage_to_laserscan

    ])
