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
    DeclareLaunchArgument('param_file', default_value='RaGNNarok_gnn.yaml',
                          description='.yaml config file in the configs folder'),
    DeclareLaunchArgument('model_state_dict', default_value='Sage_10fp_20fh_0_50_th_5mRng_0_2_res.pth',
                          description='.pth config file in the model_state_dicts folder'),
    DeclareLaunchArgument('scan_enable',
                          default_value='false',
                          choices=['true','false'],
                          description='If enabled, additionally publish a /LaserScan message on the radar_combined/scan topic'),
]  

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')
    model_state_dict = LaunchConfiguration('model_state_dict')
    scan_enable = LaunchConfiguration('scan_enable')

    #updating paths
    namespace_str = namespace.perform(context)
    if (namespace_str):
        if not namespace_str.startswith('/'):
            namespace_str = '/' + namespace_str
        tf_prefix = namespace_str.strip("/")
        laser_scan_target_frame = '{}/base_link'.format(tf_prefix)
    else:
        tf_prefix = ""
        laser_scan_target_frame = "base_link"
    
    param_file_str = param_file.perform(context)
    param_file_path = PathJoinSubstitution([pkg_pc_processing, 'configs', param_file_str])

    model_state_dict_str = model_state_dict.perform(context)
    model_state_dict_path = PathJoinSubstitution([pkg_pc_processing,'model_state_dicts', model_state_dict_str])

    param_substitutions = {
        "state_dict_path":model_state_dict_path
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

        # SetRemap('/tf', namespace_str + '/tf'),
        # SetRemap('/tf_static', namespace_str + '/tf_static'),

        #launch the point cloud combiner node
        Node(
            package='pc_processing',
            executable='pc_combiner',
            name='pc_combiner',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='pc_processing',
            executable='vel_filtering',
            name='vel_filtering',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='pc_processing',
            executable='pc_integrator_gnn',
            name='pc_integrator_gnn',
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
                {'max_height':0.25},
                {'angle_min':-3.141592653589793},
                {'angle_max':3.141592653589793},
                {'angle_increment':3.141592653589793/90}, #pi/180
                {'queue_size':10},
                {'scan_time':1.0/20.0},
                {'range_min':0.5},
                {'range_max':10.0},
                {'target_frame':laser_scan_target_frame}, #use radar's point cloud frame
                {'transform_tolerance':0.01},
                {'use_inf':True},
            ],
            condition=IfCondition(scan_enable),
            remappings=[
                ('cloud_in', 'radar_combined/integrated_pc'),  # Remap input point cloud topic
                ('scan', 'radar_combined/scan')  # Remap output laser scan topic
            ],
        ),
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
