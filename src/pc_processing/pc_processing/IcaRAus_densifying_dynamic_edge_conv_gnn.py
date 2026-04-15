import rclpy
import rclpy.duration
from rclpy.node import Node, ParameterDescriptor, ParameterType
import rclpy.publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy.time
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import numpy as np
import torch

from odometry.point_cloud_processing.vel_filtering import VelFiltering
from geometries.pose.pose import Pose
from geometries.pose.orientation import Orientation
from geometries.pose.position import Position

from mmwave_model_integrator.input_encoders._node_encoder import _NodeEncoder
from odometry.point_cloud_processing.accumulation.integrators.temporal_density_pc_integrator_gnn import TemporalDensityPCIntegratorGNN
from odometry.point_cloud_processing.clustering.occlusion_aware_clustering import OcclusionAwareClustering
from odometry.point_cloud_processing.accumulation.pc_accumulator import GtPointLabelingStrategy

from mmwave_model_integrator.config import Config
from mmwave_model_integrator.model_runner.gnn_runner import GNNRunner
from mmwave_model_integrator.torch_training.models.DensifyingDeepDynamicEdgeConvGnn import DensifyingDeepDynamicEdgeConvGnn


class IcaRAusDensifyingDynamicEdgeConvGnnNode(Node):
    """
    ROS 2 node that combines multiple point clouds, filters velocities based on odometry,
    and runs a Temporal Density Point Cloud Integrator backed by a GNN.

    This node effectively replaces three separate components:
    - pc_combiner
    - vel_filtering
    - pc_integrator

    It aggregates data synchronously before executing dense evaluations
    to minimize inter-process communication overhead.
    """
    def __init__(self):
        """
        Initializes the IcaRAus Densifying Dynamic Edge Conv GNN node.

        It initializes ROS parameters, configures tf2 operations, creates the
        necessary publishers/subscribers, and sets up a timer loop logic for
        batch processing the latest subscribed messages.
        """
        super().__init__('icaraus_densifying_dynamic_edge_conv_gnn')

        # Parameters
        self.base_frame = ""
        self.in_point_cloud_topics = []
        self.odom_sub_topic = ""
        
        self.detected_point_cloud_pub_topic = ""
        self.dynamic_point_cloud_pub_topic = ""
        self.static_point_cloud_pub_topic = ""
        self.integrated_point_cloud_pub_topic = ""
        
        self.update_rate = 10.0
        self.v_thresh = 0.05
        self.grid_resolution_m = 0.1
        
        self.model_state_dict_path = ""
        self.model_config_path = ""

        # QoS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # tf
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self,
            spin_thread=True,
            qos=self.qos_profile
        )

        # Subscribers
        self.in_pc_subs = []
        self.in_pc_msgs_latest = []
        self.pc_fields = []
        self.latest_stamp = None

        self.odom_sub = None
        self.odom_sub_latest = None
        self.vehicle_vel = np.zeros(shape=2)
        self.current_pose = Pose()
        self.vehicle_moving = False

        # Publishers
        self.detected_pc_pub = None
        self.dynamic_pc_pub = None
        self.static_pc_pub = None
        self.integrated_pc_pub = None

        # Processors
        self.vel_filtering = None
        self.pc_integrator = None

        # Initialization
        self.init_params()
        self.init_processors()
        self.init_pubs()
        self.init_subs()
        self.init_timer()

    def init_params(self):
        """
        Declares and initializes all configurable ROS 2 parameters for the node.
        
        This maps topic subscriptions, publication configurations, execution rate, 
        and model architecture dependencies.
        """
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('in_point_cloud_topics', ['/radar_0/detected_points'])
        self.declare_parameter('odom_sub_topic', '/odom')
        
        self.declare_parameter('detected_point_cloud_pub_topic', '/radar_combined/detected_points')
        self.declare_parameter('dynamic_point_cloud_pub_topic', '/radar_combined/dynamic_points')
        self.declare_parameter('static_point_cloud_pub_topic', '/radar_combined/static_points')
        self.declare_parameter('integrated_point_cloud_pub_topic', '/radar_combined/integrated_pc')
        
        self.declare_parameter('update_rate', 10.0)
        
        self.declare_parameter('model_state_dict_path', '')
        self.declare_parameter('model_config_path', '')

        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.in_point_cloud_topics = self.get_parameter('in_point_cloud_topics').get_parameter_value().string_array_value
        self.odom_sub_topic = self.get_parameter('odom_sub_topic').get_parameter_value().string_value
        
        self.detected_point_cloud_pub_topic = self.get_parameter('detected_point_cloud_pub_topic').get_parameter_value().string_value
        self.dynamic_point_cloud_pub_topic = self.get_parameter('dynamic_point_cloud_pub_topic').get_parameter_value().string_value
        self.static_point_cloud_pub_topic = self.get_parameter('static_point_cloud_pub_topic').get_parameter_value().string_value
        self.integrated_point_cloud_pub_topic = self.get_parameter('integrated_point_cloud_pub_topic').get_parameter_value().string_value
        
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        
        self.model_state_dict_path = self.get_parameter('model_state_dict_path').get_parameter_value().string_value
        self.model_config_path = self.get_parameter('model_config_path').get_parameter_value().string_value

        self.get_logger().info(f"Initialized parameters for {self.get_name()}")

    def init_processors(self):
        """
        Creates instances of the algorithms and pipeline components driving the logic.
        
        This initializes the VelFiltering object using a specified v_thresh, builds
        the DensifyingDeepDynamicEdgeConvGnn model using specifications located inside the
        state dict and config paths, loads configuration properties, and embeds the complete 
        schema into an instance of TemporalDensityPCIntegratorGNN.
        """
        # Velocity filtering
        self.vel_filtering = VelFiltering(
            v_thresh=self.v_thresh,
            min_static_rejection_radius=0.25,
            dynamic_cluster_eps=0.5,
            dynamic_cluster_min_samples=15
        )

        # GNN and PC Integrator
        config = Config(self.model_config_path)
        model_cfg = config.model
        model_type = model_cfg.pop('type')
        model = DensifyingDeepDynamicEdgeConvGnn(**model_cfg)

        dataset_cfg = config.trainer["dataset"]
        enable_downsampling = dataset_cfg.get("enable_downsampling", False)
        downsample_keep_ratio = dataset_cfg.get("downsample_keep_ratio", 1.0)
        downsample_min_points = dataset_cfg.get("downsample_min_points", 0)

        runner = GNNRunner(
            model=model,
            state_dict_path=self.model_state_dict_path,
            cuda_device="cuda:0" if torch.cuda.is_available() else "cpu",
            edge_radius=10.0,
            enable_downsampling=enable_downsampling,
            downsample_keep_ratio=downsample_keep_ratio,
            downsample_min_points=downsample_min_points,
            use_sigmoid=True,
            print_stats=False
        )

        input_encoder = _NodeEncoder()

        self.pc_integrator = TemporalDensityPCIntegratorGNN(
            gnn_runner=runner,
            input_encoder=input_encoder,
            normalize_frames=True,
            gt_distance_threshold_m=0.4,
            num_frames_history_gt=1,
            valid_fovs_deg=[(-70,70),(110,-110)],
            num_frames_history=50,
            num_frames_valid_point_history=0,
            min_detection_radius=1.0,
            max_detection_radius=8.0,
            grid_resolution_m=self.grid_resolution_m,
            subsample_percentage=1.0,
            gt_point_labeling_strategy=GtPointLabelingStrategy.USE_VALID_POINTS_FOR_GT_CLASSIFICATION,
            # gt_occlusion_aware_clustering=OcclusionAwareClustering(
            #     clustering_eps=0.5,
            #     clustering_min_samples=12,
            #     angle_res_rad=0.017,
            #     occlusion_threshold=0.7,
            #     subsample_percentage=1.0,
            #     remove_occluded=True,
            #     filter_method='ray_trace'
            # ),
            occlusion_aware_clustering=OcclusionAwareClustering(
                clustering_eps=0.25,
                clustering_min_samples=10,
                angle_res_rad=0.017,
                occlusion_threshold=0.9,
                subsample_percentage=0.20,
                remove_occluded=False,
                filter_method='ray_trace'
            )
        )

    def init_pubs(self):
        """
        Creates publishers for output topics over the target reliable Quality of Service (QoS).
        
        This establishes the output routes for detected, dynamic, static, 
        and integrated point cloud messages.
        """
        self.detected_pc_pub = self.create_publisher(PointCloud2, self.detected_point_cloud_pub_topic, self.qos_profile)
        self.dynamic_pc_pub = self.create_publisher(PointCloud2, self.dynamic_point_cloud_pub_topic, self.qos_profile)
        self.static_pc_pub = self.create_publisher(PointCloud2, self.static_point_cloud_pub_topic, self.qos_profile)
        self.integrated_pc_pub = self.create_publisher(PointCloud2, self.integrated_point_cloud_pub_topic, self.qos_profile)

    def init_subs(self):
        """
        Iterates over parametrized incoming point cloud topics continuously to spawn separate subscribers.
        
        It also initiates listening onto the base odometry topic to monitor motion capabilities.
        """
        for i in range(len(self.in_point_cloud_topics)):
            sub = self.create_subscription(
                msg_type=PointCloud2,
                topic=self.in_point_cloud_topics[i],
                callback=lambda msg, idx=i: self.in_pc_sub_callback(msg, idx),
                qos_profile=self.qos_profile
            )
            self.in_pc_subs.append(sub)
            self.in_pc_msgs_latest.append(None)

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic=self.odom_sub_topic,
            callback=self.odom_sub_callback,
            qos_profile=self.qos_profile
        )

    def in_pc_sub_callback(self, msg: PointCloud2, index: int):
        """
        Subscriber callback storing the latest detected point cloud message from an independent radar topic.
        
        Args:
            msg (PointCloud2): Raw input point cloud object.
            index (int): Identifier for matching the message's origin stream to the internal array.
        """
        self.in_pc_msgs_latest[index] = msg
        self.latest_stamp = msg.header.stamp
        if not self.pc_fields:
            self.pc_fields = msg.fields

    def odom_sub_callback(self, msg: Odometry):
        """
        Subscriber callback monitoring the system odometry tracking displacement.
        
        Updates instantaneous linear displacement vectors to accurately inform Velocity Filtering
        distinctions between moving and fixed body occurrences.
        
        Args:
            msg (Odometry): Standard Navigation ROS message reporting positional twist.
        """
        self.vehicle_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ])
        
        vehicle_vel_abs = np.abs(np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]))

        vehicle_rot_abs = np.abs(np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]))

        self.vehicle_moving = not (np.all(vehicle_vel_abs < 0.01) and np.all(vehicle_rot_abs < 0.02))

        self.current_pose = Pose(
            position=Position(
                x=msg.pose.pose.position.x,
                y=msg.pose.pose.position.y,
                z=msg.pose.pose.position.z,
            ),
            orientation=Orientation(
                qx=msg.pose.pose.orientation.x,
                qy=msg.pose.pose.orientation.y,
                qz=msg.pose.pose.orientation.z,
                qw=msg.pose.pose.orientation.w
            )
        )
        self.odom_sub_latest = msg

    def init_timer(self):
        """
        Spawns the execution timer executing core node updates mapped iteratively towards the
        parametrically defined 'update_rate'.
        """
        timer_period = 1 / self.update_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Core process loop executed via independent timer invocation fetching unified data.
        
        This loop processes identically in four stages:
        1. Fetch all latest radar clouds and combine them uniformly referencing standard frame structures locally.
        2. Perform separation into Static points and Dynamic points based on ego motion constraints.
        3. Pass statically qualified clusters towards temporal accumulation mappings dynamically evaluated by the embedded model.
        4. Transmit all evaluated combinations independently on established publisher channels.
        """
        point_clouds_to_combine = []
        for i in range(len(self.in_point_cloud_topics)):
            pc2_msg = self.in_pc_msgs_latest[i]
            if pc2_msg is None:
                continue

            pc2_msg = self.transform_pc2_to_base_frame(pc2_msg)
            if pc2_msg is None:
                continue

            point_clouds_to_combine.append(self.pointcloud2_to_np(pc2_msg))

        if len(point_clouds_to_combine) > 0 and self.latest_stamp is not None:
            combined_pc_np = np.vstack(point_clouds_to_combine)
            
            # Publish combined detected points
            detected_msg = self.np_to_pointcloud2(combined_pc_np, self.pc_fields)
            self.detected_pc_pub.publish(detected_msg)

            # Separate static and dynamic
            static_points = self.vel_filtering.get_static_detections(combined_pc_np, self.vehicle_vel)
            dynamic_points = self.vel_filtering.get_dynamic_detections(combined_pc_np, self.vehicle_vel)

            # Publish static and dynamic points
            if static_points.shape[0] > 0:
                self.static_pc_pub.publish(self.np_to_pointcloud2(static_points, self.pc_fields))
            else:
                self.static_pc_pub.publish(self.np_to_pointcloud2([], self.pc_fields))
                
            if dynamic_points.shape[0] > 0:
                self.dynamic_pc_pub.publish(self.np_to_pointcloud2(dynamic_points, self.pc_fields))
            else:
                self.dynamic_pc_pub.publish(self.np_to_pointcloud2([], self.pc_fields))

            # Publish integrated points (if moving update integrator)
            try:
                # Use identical current pose logic as in pc_integrator_IcaRAus
                if self.vehicle_moving:
                    transform = self.tf_buffer.lookup_transform(
                        target_frame=f"{self.get_namespace().strip('/')}/odom" if self.get_namespace().strip('/') else "odom",
                        source_frame=self.base_frame,#f"{self.get_namespace().strip('/')}/{self.base_frame}" if self.get_namespace().strip('/') else self.base_frame,
                        time=rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.0, nanoseconds=int(0.01 * 1e9))
                    )
                    
                    transformer_pose = Pose(
                        position=Position(
                            x=transform.transform.translation.x,
                            y=transform.transform.translation.y,
                            z=transform.transform.translation.z,
                        ),
                        orientation=Orientation(
                            qx=transform.transform.rotation.x,
                            qy=transform.transform.rotation.y,
                            qz=transform.transform.rotation.z,
                            qw=transform.transform.rotation.w
                        )
                    )
                    self.pc_integrator.add_points(
                        static_points=static_points,
                        current_pose=transformer_pose
                    )
            except Exception as e:
                self.get_logger().info(f"timer_callback: Could not get transformation from odom->base: {e}")

            processed_pc = self.pc_integrator.get_points()
            if processed_pc is not None and processed_pc.shape[0] > 0:
                self.integrated_pc_pub.publish(self.np_to_pointcloud2(processed_pc, self.pc_fields[:3])) # limit fields safely
            else:
                self.integrated_pc_pub.publish(self.np_to_pointcloud2([], self.pc_fields[:3]))

    def transform_pc2_to_base_frame(self, msg: PointCloud2) -> PointCloud2:
        """
        Applies mathematical TF spatial re-scaling utilizing lookup mapping.
        
        Args:
            msg (PointCloud2): Formatted source point cloud map structure.
            
        Returns:
            PointCloud2: The translated point cloud transformed dynamically towards 'base_frame'. Returns None on a TF transform tree fault.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,#f"{self.get_namespace().strip('/')}/{self.base_frame}" if self.get_namespace().strip('/') else self.base_frame,
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            transformed_cloud = do_transform_cloud(msg, transform)
            return transformed_cloud
        except Exception as e:
            self.get_logger().info(f"transform_pc2_to_base_frame: Could not transform point cloud: {e}")
            return None

    def pointcloud2_to_np(self, msg: PointCloud2) -> np.ndarray:
        """
        Cast conversion layer from binary byte data PointClouds natively to contiguous multi-dimensional blocks.
        
        Args:
            msg (PointCloud2): Raw ROS 2 native PointCloud object reference stream.
            
        Returns:
            np.ndarray: Reconfigured matrix array with floating precision boundaries dropping null vectors efficiently.
        """
        return pc2.read_points_numpy(cloud=msg, skip_nans=True, reshape_organized_cloud=True)

    def np_to_pointcloud2(self, points: np.ndarray, fields: list) -> PointCloud2:
        """
        Inverse data structure cast formatting array mappings systematically backwards against standardized ROS configurations.
        
        Args:
            points (np.ndarray): NumPy contiguous array vector stream.
            fields (list): Preserved meta-schema describing the payload limits to properly define encoded ranges over bytes.
            
        Returns:
            PointCloud2: Safe encoded format wrapper structured specifically for broad execution pipelines on base_frame maps.
        """
        header = std_msgs.msg.Header()
        header.stamp = self.latest_stamp if self.latest_stamp else self.get_clock().now().to_msg()
        header.frame_id = self.base_frame
        msg = pc2.create_cloud(header=header, fields=fields, points=points)
        return msg


def main(args=None):
    """
    Standard entry initialization point spinning active nodes alongside ROS background queues.
    
    Args:
        args: Passed string variables originating locally overriding baseline attributes dynamically natively.
    """
    rclpy.init(args=args)
    node = IcaRAusDensifyingDynamicEdgeConvGnnNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
