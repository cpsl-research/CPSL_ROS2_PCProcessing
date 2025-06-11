import rclpy
import rclpy.duration
from rclpy.node import Node,ParameterDescriptor,ParameterType
import rclpy.publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy.subscription
import rclpy.time
import rclpy.timer
from sensor_msgs.msg import PointCloud2,PointField
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry

import numpy as np

#pose tracking
from geometries.pose.pose import Pose
from geometries.pose.orientation import Orientation
from geometries.pose.position import Position

#model running code
from mmwave_model_integrator.input_encoders._node_encoder import _NodeEncoder
from mmwave_model_integrator.ground_truth_encoders._gt_node_encoder import _GTNodeEncoder
from mmwave_model_integrator.model_runner.gnn_runner import GNNRunner
from mmwave_model_integrator.torch_training.models.SAGEGnn import SageGNNClassifier

#point cloud processing modules
from odometry.point_cloud_processing._point_cloud_integrator import _PointCloudIntegrator
from odometry.point_cloud_processing.pc_grid.probabilistic_pc_grid_gnn import ProbabilisticPCGridGNN
from odometry.point_cloud_processing.pc_grid.historical_pc_grid import HistoricalPCGrid

class GNNPCIntegrator(Node):
    """A node that subscribesto a radar point cloud with [x,y,z,vel] information and 
    a vehicle's odometry to publish a topic of "static" detections and a topic of 
    "dynamic" detections
    NOTE: For now, only filters out 2d velocity detections
    """
    def __init__(self):
        super().__init__('pc_combiner')

        #ROS2 parameters
        self.point_cloud_sub_topic:str = ""
        self.odom_sub_topic:str = ""
        self.point_cloud_pub_topic:str = ""

        #point cloud instegrator parameters
        self.grid_resolution_m:float = 0.0
        self.grid_max_distance_m:float = 0.0
        self.num_frames_history:int = 0
        self.num_frames_persistance:int = 0
        self.state_dict_path:str = ""

        #defining qos profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        #subscribers
        self.point_cloud_sub:rclpy.subscription.Subscription=None
        self.odom_sub:rclpy.subscription.Subscription = None

        #publishers
        self.point_cloud_pub:rclpy.publisher.Publisher = None

        #tf publishing
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self,
            spin_thread=True,
            qos=self.qos_profile)
        
        #storing last odom message
        self.odom_sub_latest:Odometry = None

        #vehicle velocity variable
        self.current_pose:Pose = Pose()
        self.vehicle_moving:bool = False

        #point cloud integrator object
        self.pc_integrator:_PointCloudIntegrator = None

        #initialize the node
        self.init_params() #parameters
        self.init_pc_integrator() #pc integration
        self.init_pubs() #publishers
        self.init_subs() #point cloud subscribers

        return

    
    def init_pc_integrator(self):

        runner = GNNRunner(
            model = SageGNNClassifier(
                in_channels=4,
                hidden_channels=16,
                out_channels=1
            ), state_dict_path=self.state_dict_path,
            cuda_device='cpu',
            edge_radius=10.0
        )

        probabilistic_pc_grid = ProbabilisticPCGridGNN(
            runner=runner,
            grid_resolution_m=self.grid_resolution_m,
            grid_max_distance_m=self.grid_max_distance_m,
            num_frames_history=self.num_frames_history
        )

        historical_pc_grid = HistoricalPCGrid(
            grid_resolution_m=self.grid_resolution_m,
            grid_max_distance_m=self.grid_max_distance_m,
            num_frames_persistance=self.num_frames_persistance
        )

        self.pc_integrator = _PointCloudIntegrator(
            probabilistic_pc_grid=probabilistic_pc_grid,
            historical_pc_grid=historical_pc_grid,
            min_detection_radius=1.0,
            max_detection_radius=20.0
        )

    
    def init_params(self):
        """Declare and set all of the ROS2 parameters
        """
        # ROS2 parameters
        self.declare_parameter(
            name='point_cloud_sub_topic',
            value='/radar_combined/static_points',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The pointCloud2 topic to read radar detections from'
            )
        )
        self.declare_parameter(
            name='odom_sub_topic',
            value='/odom',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The pointCloud2 topic to read radar detections from'
            )
        )
        self.declare_parameter(
            name='point_cloud_pub_topic',
            value='/radar_combined/integrated_pc',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The pointCloud2 to publish the final processed point cloud to'
            )
        )

        #point cloud integrator commands
        self.declare_parameter(
            name='grid_resolution_m',
            value=0.20,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='resolution of the point cloud grid to create (in meters)'
            )
        )
        self.declare_parameter(
            name='grid_max_distance_m',
            value=5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The maximum range of the point cloud grid'
            )
        )
        self.declare_parameter(
            name='num_frames_history',
            value=80,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The number of frames to use when generating the probabilistic occupancy grid maps'
            )
        )
        self.declare_parameter(
            name='num_frames_persistance',
            value=1.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The number of frames that detections persist once detected'
            )
        )
        self.declare_parameter(
            name='state_dict_path',
            value='',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Full path to the state dict of the model to load'
            )
        )
        
        #set parameters - ROS2
        self.point_cloud_sub_topic = \
            self.get_parameter('point_cloud_sub_topic').get_parameter_value().string_value
        self.odom_sub_topic = \
            self.get_parameter('odom_sub_topic').get_parameter_value().string_value
        self.point_cloud_pub_topic = \
            self.get_parameter('point_cloud_pub_topic').get_parameter_value().string_value
        
        #set parameters - point cloud integration
        self.grid_resolution_m = \
            self.get_parameter('grid_resolution_m').get_parameter_value().double_value
        self.grid_max_distance_m = \
            self.get_parameter('grid_max_distance_m').get_parameter_value().double_value
        self.num_frames_history = \
            self.get_parameter('num_frames_history').get_parameter_value().integer_value
        self.num_frames_persistance = \
            self.get_parameter('num_frames_persistance').get_parameter_value().integer_value
        self.state_dict_path = \
            self.get_parameter('state_dict_path').get_parameter_value().string_value

        #log the parameter values
        self.get_logger().info(f'point_cloud_sub_topic set to: {self.point_cloud_sub_topic}')
        self.get_logger().info(f'odom_sub_topic set to: {self.odom_sub_topic}')
        self.get_logger().info(f'point_cloud_pub_topic set to: {self.point_cloud_pub_topic}')
        self.get_logger().info(f'grid_resolution_m set to: {self.grid_resolution_m}')
        self.get_logger().info(f'grid_max_distance_m set to: {self.grid_max_distance_m}')
        self.get_logger().info(f'num_frames_history set to: {self.num_frames_history}')
        self.get_logger().info(f'num_frames_persistance set to: {self.num_frames_persistance}')
        self.get_logger().info(f'state_dict_path set to: {self.state_dict_path}')

    def init_subs(self):
        """Initialize the point cloud subscribers based on the given in_point_cloud_topics list
        """

        #point cloud subscriber
        self.point_cloud_sub = self.create_subscription(
            msg_type=PointCloud2,
            topic=self.point_cloud_sub_topic,
            callback=self.point_cloud_sub_callback,
            qos_profile=self.qos_profile
        )

        #odom subscriber
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic=self.odom_sub_topic,
            callback=self.odom_sub_callback,
            qos_profile=self.qos_profile
        )

    def point_cloud_sub_callback(self,msg:PointCloud2):
        """When a new point cloud is received, separate the point cloud into 
        dynamic and static points, then publish on the respective point cloud

        Args:
            msg (PointCloud2): the most recent point cloud from the subscription
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=f"{self.get_namespace().strip("/")}/odom",
                source_frame=f"{self.get_namespace().strip("/")}/base_link",
                time=rclpy.time.Time(seconds=0,nanoseconds=0),#rclpy.time.Time.from_msg(msg.header.stamp),
                timeout=rclpy.duration.Duration(
                    seconds=0.0,
                    nanoseconds=int(0.01 * 1e9)
                )
            )

            current_pose = Pose(
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

            sec_diff = msg.header.stamp.sec - transform.header.stamp.sec
            nsec_dif = msg.header.stamp.nanosec - transform.header.stamp.nanosec

            self.get_logger().info("Time diff between radar and tf message: {}".format(
                sec_diff + nsec_dif * 1e-9
            ))

            if self.vehicle_moving:
                pc_np = self.pointcloud2_to_np(msg)

                #add code here to process the point cloud
                self.pc_integrator.add_points(
                    static_points=pc_np,
                    current_pose=current_pose
                )
        except Exception as e:
            self.get_logger().info("could get transformation from odom->base: {}".format(e))


        # if self.vehicle_moving:
        #     #get the points as a np array
        #     pc_np = self.pointcloud2_to_np(msg)
        #     #     return None  

        #     #add code here to process the point cloud
        #     self.pc_integrator.add_points(
        #         static_points=pc_np,
        #         current_pose=self.current_pose
        #     )

        #     # log the time difference between the odom and pc2 message
        #     # sec_diff = msg.header.stamp.sec - self.odom_sub_latest.header.stamp.sec
        #     # nsec_dif = msg.header.stamp.nanosec - self.odom_sub_latest.header.stamp.sec

        #     # self.get_logger().info("Time diff between radar and tf message: {}".format(
        #     #     sec_diff + nsec_dif * 1e-9
        #     # ))

        #get the latest point cloud
        processed_pc = self.pc_integrator.get_latest_pc()

        if processed_pc.shape[0] > 0:
            new_msg = pc2.create_cloud(
                header=msg.header,
                fields=msg.fields[:3],
                points=processed_pc
            )
            
        else:
            new_msg = pc2.create_cloud(
                header=msg.header,
                fields=msg.fields,
                points=[]
            )

        self.point_cloud_pub.publish(new_msg)

        return

    def odom_sub_callback(self,msg:Odometry):
        """Use the most recent Odometry message to update the vehicle velocity variable

        Args:
            msg (Odometry): the latest Odometry message from the subscriber
        """
        #update the vehicle's velocity
        vehicle_vel = np.abs(np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]))

        vehicle_rot = np.abs(np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]))

        self.vehicle_moving = not (
            np.all(vehicle_vel < 0.01) and np.all(vehicle_rot < 0.02)
            )

        #update the latest pose
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

        
        return

    def init_pubs(self):
        """Initialize the static and dynamic point cloud publishers
        """
        #dynamic point cloud publisher
        self.point_cloud_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic=self.point_cloud_pub_topic,
            qos_profile=self.qos_profile
        )

        return
    
    def pointcloud2_to_np(self,msg:PointCloud2)->np.ndarray:
        """Converts a PointCloud2 array into a numpy array

        Args:
            msg (PointCloud2): The pointCloud2 object to convert to a numpy array

        Returns:
            np.ndarray: Numpy array of points from the PointCloud2 array
        """
        point_cloud = pc2.read_points_numpy(
            cloud=msg,
            skip_nans=True,
            reshape_organized_cloud=True
        )

        return point_cloud
    
    def np_to_pointcloud2(self,points:np.ndarray,header:Header,fields:list)->PointCloud2:
        """Convert a numpy array into a pointcloud2 object in the base_frame

        Args:
            points (np.ndarray): array of points to convert to a numpy array

        Returns:
            PointCloud2: PointCloud2 object of points from the numpy array
        """
        msg = pc2.create_cloud(
            header=header,
            fields=fields,
            points=points
        )

        return msg


def main(args=None):
    rclpy.init(args=args)

    pc_integrator = GNNPCIntegrator()

    rclpy.spin(pc_integrator)

    # Destroy the node explicitly
    pc_integrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
