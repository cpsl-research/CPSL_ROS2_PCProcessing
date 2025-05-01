from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


pkg_pc_processing = get_package_share_directory("pc_processing")

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='namespace'),
    DeclareLaunchArgument('param_file', default_value='radsar_uav.yaml',
                          description='.yaml config file in the configs folder'),
    DeclareLaunchArgument('model_state_dict', default_value='RadSAR_1_chirp_10e.pth',
                          description='.pth config file in the model_state_dicts folder'),
    DeclareLaunchArgument('radar_config', default_value='1843_RadSAR.cfg',
                          description='.cfg config file in the radar_cfgs folder'),
    DeclareLaunchArgument('scan_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='If enabled, additionally publish a /LaserScan message on the radar_0/scan topic'),
]  

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    model_state_dict = LaunchConfiguration('model_state_dict')
    radar_config = LaunchConfiguration('radar_config')
    scan_enable = LaunchConfiguration('scan_enable')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str):
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        scan_target_frame = '{}/base_footprint'.format(tf_prefix)
    else:
        tf_prefix = ""
        scan_target_frame = "base_footprint"
    
    param_file_str = param_file.perform(context)
    param_file_path = PathJoinSubstitution([pkg_pc_processing, 'configs', param_file_str])

    model_state_dict_str = model_state_dict.perform(context)
    model_state_dict_path = PathJoinSubstitution([pkg_pc_processing,'model_state_dicts', model_state_dict_str])

    radar_config_str = radar_config.perform(context)
    radar_config_path = PathJoinSubstitution([pkg_pc_processing,'radar_cfgs',radar_config_str])

    param_substitutions = {
        "state_dict_path":model_state_dict_path,
        "radar_config_path":radar_config_path
    }

    #update the parameter file with 
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=param_file_path,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Apply the following re-mappings only within this group
    bringup_group = GroupAction([
        PushRosNamespace(namespace),

        #launch the point cloud combiner node
        Node(
            package='pc_processing',
            executable='radsar_processor',
            name='radsar_processor',
            output='screen',
            parameters=[configured_params],
        ),
        #Launch laserscan topic
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan_node',
            output='screen',
            parameters=[
                {'min_height':-0.1},
                {'max_height':0.1},
                {'angle_min':-3.141592653589793},
                {'angle_max':3.141592653589793},
                {'angle_increment':0.0174532925}, #pi/180
                {'queue_size':10},
                {'scan_time':1.0/20.0},
                {'range_min':0.25},
                {'range_max':5.0},
                {'target_frame':scan_target_frame}, #use lidar's point cloud frame
                {'transform_tolerance':0.01},
                {'use_inf':True},
            ],
            condition=IfCondition(scan_enable),
            remappings=[
                ('cloud_in', 'radar_0/radsar_point_cloud'),  # Remap input point cloud topic
                ('scan', 'radar_0/scan')  # Remap output laser scan topic
            ],
        ),
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
