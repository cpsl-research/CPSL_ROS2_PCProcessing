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

from odometry.point_cloud_processing.vel_filtering import VelFiltering

class VelFilter(Node):
    """A node that subscribesto a radar point cloud with [x,y,z,vel] information and 
    a vehicle's odometry to publish a topic of "static" detections and a topic of 
    "dynamic" detections
    NOTE: For now, only filters out 2d velocity detections
    """
    def __init__(self):
        super().__init__('pc_combiner')

        #parameters
        self.point_cloud_sub_topic:str = ""
        self.odom_sub_topic:str = ""
        self.dynamic_point_cloud_pub_topic:str = ""
        self.static_point_cloud_pub_topic:str = ""
        self.v_thresh:float = 0.0

        #defining qos profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #subscribers
        self.point_cloud_sub:rclpy.subscription.Subscription=None
        self.odom_sub:rclpy.subscription.Subscription = None

        #publishers
        self.dynamic_point_cloud_pub:rclpy.publisher.Publisher = None
        self.static_point_cloud_pub:rclpy.publisher.Publisher = None

        #vehicle velocity variable
        self.vehicle_vel:np.ndarray = np.zeros(shape=2)

        #loading a vel_filtering object
        self.vel_filtering:VelFiltering = None

        #initialize the node
        self.init_params() #parameters
        self.init_vel_viltering() #vel filtering
        self.init_pubs() #publishers
        self.init_subs() #point cloud subscribers

        return

    def init_vel_viltering(self):
        self.vel_filtering = VelFiltering(
            v_thresh=self.v_thresh,
            min_static_rejection_radius=0.25, #don't care about these other parameters
            dynamic_cluster_eps=0.5,
            dynamic_cluster_min_samples=15
        )
    
    def init_params(self):
        """Declare and set all of the ROS2 parameters
        """
        # Declare parameters
        self.declare_parameter(
            name='point_cloud_sub_topic',
            value='/radar_combined/detected_points',
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
            name='dynamic_point_cloud_pub_topic',
            value='/radar_combined/dynamic_points',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The pointCloud2 topic to read radar detections from'
            )
        )
        self.declare_parameter(
            name='static_point_cloud_pub_topic',
            value='/radar_combined/static_points',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The pointCloud2 topic to read radar detections from'
            )
        )
        self.declare_parameter(
            name='v_thresh',
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Threshold for filtering out dynamic detections"
            )
        )
        
        #set parameters
        self.point_cloud_sub_topic = \
            self.get_parameter('point_cloud_sub_topic').get_parameter_value().string_value
        self.odom_sub_topic = \
            self.get_parameter('odom_sub_topic').get_parameter_value().string_value
        self.dynamic_point_cloud_pub_topic = \
            self.get_parameter('dynamic_point_cloud_pub_topic').get_parameter_value().string_value
        self.static_point_cloud_pub_topic = \
            self.get_parameter('static_point_cloud_pub_topic').get_parameter_value().string_value
        self.v_thresh = \
            self.get_parameter('v_thresh').get_parameter_value().double_value

        #log the parameter values
        self.get_logger().info(f'point_cloud_sub_topic set to: {self.point_cloud_sub_topic}')
        self.get_logger().info(f'odom_sub_topic set to: {self.odom_sub_topic}')
        self.get_logger().info(f'dynamic_point_cloud_pub_topic set to: {self.dynamic_point_cloud_pub_topic}')
        self.get_logger().info(f'static_point_cloud_pub_topic set to: {self.static_point_cloud_pub_topic}')
        self.get_logger().info(f'v_thresh set to: {self.v_thresh}')

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
        #get the points as a np array
        pc_np = self.pointcloud2_to_np(msg)

        #get the static points
        static_points = self.vel_filtering.get_static_detections(
            detections=pc_np,
            ego_vel=self.vehicle_vel
        )

        #get the dynamic points
        dynamic_points = self.vel_filtering.get_dynamic_detections(
            detections=pc_np,
            ego_vel=self.vehicle_vel
        )

        #publish the static points
        if static_points.shape[0] > 0:
            static_msg = pc2.create_cloud(
                header=msg.header,
                fields=msg.fields,
                points=static_points
            )
        else:
            static_msg = pc2.create_cloud(
                header=msg.header,
                fields=msg.fields,
                points=[]
            )
        self.static_point_cloud_pub.publish(static_msg)

        #publish the dynamic points
        if dynamic_points.shape[0] > 0:
            dynamic_msg = pc2.create_cloud(
                header=msg.header,
                fields=msg.fields,
                points=dynamic_points
            )
        else:
            dynamic_msg = pc2.create_cloud(
                header=msg.header,
                fields=msg.fields,
                points=[]
            )
        self.dynamic_point_cloud_pub.publish(dynamic_msg)

        

    def odom_sub_callback(self,msg:Odometry):
        """Use the most recent Odometry message to update the vehicle velocity variable

        Args:
            msg (Odometry): the latest Odometry message from the subscriber
        """
        #update the vehicle's velocity
        self.vehicle_vel = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ])
        
        return

    def init_pubs(self):
        """Initialize the static and dynamic point cloud publishers
        """
        #dynamic point cloud publisher
        self.dynamic_point_cloud_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic=self.dynamic_point_cloud_pub_topic,
            qos_profile=self.qos_profile
        )

        #static point cloud publisher
        self.static_point_cloud_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic=self.static_point_cloud_pub_topic,
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

    vel_filter = VelFilter()

    rclpy.spin(vel_filter)

    # Destroy the node explicitly
    vel_filter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
