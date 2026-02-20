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
    DeclareLaunchArgument('param_file', default_value='ugv_pc_combiner.yaml',
                          description='.yaml config file in the configs folder'),
]  

def launch_setup(context, *args, **kwargs):

    #load parameters
    namespace = LaunchConfiguration('namespace')
    param_file = LaunchConfiguration('param_file')

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

    param_substitutions = {
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
    ])

    return [bringup_group]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
