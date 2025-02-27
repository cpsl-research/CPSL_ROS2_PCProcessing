import rclpy
import rclpy.duration
from rclpy.node import Node,ParameterDescriptor,ParameterType
import rclpy.publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy.time
import rclpy.timer
from sensor_msgs.msg import PointCloud2,PointField
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import std_msgs.msg
import sensor_msgs_py.point_cloud2 as pc2

import numpy as np

class PCCombiner(Node):
    """A node that combines point clouds from a given list of topics
    and then re-publishes the combined point cloud in a base_frame
    on a given topic and at a given rate
    """
    def __init__(self):
        super().__init__('pc_combiner')

        #parameters
        self.base_frame:str = ""
        self.in_point_cloud_topics:list = []
        self.out_point_cloud_topic:str = ""
        self.update_rate:int = 20

        #defining qos profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        #keeping track of the fields in the point clouds
        self.pc_fields:list = []

        #subscribers
        self.in_pc_subs:list = []
        self.in_pc_msgs_latest:list = [] #keeping track of latest pc message

        #publishers
        self.pc_pub:rclpy.publisher.Publisher = None

        #tf tree management
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            buffer=self.tf_buffer,
            node=self)

        #timers
        self.timer:rclpy.timer.Timer = None

        #initialize the node
        self.init_params() #parameters
        self.init_subs() #point cloud subscribers
        self.init_pc_pub()
        self.init_timer() #initialize the timer and start the run loop

        return

    def init_params(self):
        """Declare and set all of the ROS2 parameters
        """
        # Declare parameters
        self.declare_parameter(
            name='base_frame',
            value='base_link',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The base_link frame id to transform all point into'
            )
        )
        self.declare_parameter(
            name='in_point_cloud_topics',
            value=['/radar_0/detected_points'],
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='List of topics publishing point clouds to subscribe'
            )
        )
        self.declare_parameter(
            name='out_point_cloud_topic',
            value='/combined_point_cloud/raw_detections',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The topic on which to publish the combined point cloud'
            )
        )
        self.declare_parameter(
            name='update_rate',
            value=20.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='The rate in Hz to publish the combined point cloud at'
            )
        )
        
        #set parameters
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.in_point_cloud_topics = \
            self.get_parameter('in_point_cloud_topics').get_parameter_value().string_array_value
        self.out_point_cloud_topic = \
            self.get_parameter('out_point_cloud_topic').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value

        #log the parameter values
        self.get_logger().info(f'Base frame set to: {self.base_frame}')
        self.get_logger().info(f'In point cloud topics set to: {self.in_point_cloud_topics}')
        self.get_logger().info("Output point cloud topic set to: {}".format(self.out_point_cloud_topic))
        self.get_logger().info(f'update_rate set to: {self.update_rate}')
    
    def init_subs(self):
        """Initialize the point cloud subscribers based on the given in_point_cloud_topics list
        """
        for i in range(len(self.in_point_cloud_topics)):
            sub = self.create_subscription(
                msg_type=PointCloud2,
                topic=self.in_point_cloud_topics[i],
                callback=lambda msg, idx=i: self.in_pc_sub_callback(msg,idx),
                qos_profile=self.qos_profile
            )
            self.in_pc_subs.append(sub)
            self.in_pc_msgs_latest.append(None) #append empty message for now

    def in_pc_sub_callback(self,msg:PointCloud2,index:int):
        """Update the in_pc_msgs_latest array with the latest point cloud message from a given array

        Args:
            msg (PointCloud2): A PointCloud2 message to add to the array
            index (int): the index of the point cloud subscriber corresponding to the message
        """
        self.in_pc_msgs_latest[index] = msg

        #update the fields list if it hasn't already been
        if not self.pc_fields:
            self.pc_fields = msg.fields
        return
    
    def init_pc_pub(self):
        """Initialize the point cloud publisher
        """
        self.pc_pub = self.create_publisher(
            msg_type=PointCloud2,
            topic=self.out_point_cloud_topic,
            qos_profile=self.qos_profile
        )

        return


    def init_timer(self):
        """Initialize the runtime timer to publish the combined point cloud at
        """
        timer_period = 1/self.update_rate  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def timer_callback(self):
        """Collect all of the latest available point cloud messages,
        combine them into a single point cloud, then publish the combined point cloud
        """
        #get all point clouds
        point_clouds_to_combine = []
        for i in range(len(self.in_point_cloud_topics)):
            
            #get the latest point cloud from the subscriber
            pc2 = self.in_pc_msgs_latest[i]
            if pc2 is None:
                continue

            #transform the point cloud to the base_frame
            pc2 = self.transform_pc2_to_base_frame(pc2)
            if pc2 is None:
                continue

            #get the points from the point cloud
            point_clouds_to_combine.append(
                self.pointcloud2_to_np(pc2)
            )

        #combine all of the point clouds
        if len(point_clouds_to_combine) > 0:
            
            #combine the point cloud arrays
            combined_pc_np = np.vstack(point_clouds_to_combine)

            #convert back to point cloud2
            combined_pc = self.np_to_pointcloud2(combined_pc_np)

            #publish the point cloud2
            self.pc_pub.publish(combined_pc)
        return

    def transform_pc2_to_base_frame(self,msg:PointCloud2)->PointCloud2:
        """Transform a pointCloud2 into the base_frame

        Args:
            msg (PointCloud2): PoinCloud2 object to transform into the base link

        Returns:
            PointCloud2: a new PointCloud2 object in the base_frame
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=self.base_frame,
                source_frame=msg.header.frame_id,
                time=rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            transformed_cloud = do_transform_cloud(
                cloud=msg,
                transform=transform
            )

            return transformed_cloud
        
        except Exception as e:
            self.get_logger().info("could not transform point cloud: {}".format(e))

            return None
    
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
    
    def np_to_pointcloud2(self,points:np.ndarray)->PointCloud2:
        """Convert a numpy array into a pointcloud2 object in the base_frame

        Args:
            points (np.ndarray): array of points to convert to a numpy array

        Returns:
            PointCloud2: PointCloud2 object of points from the numpy array
        """
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.base_frame

        msg = pc2.create_cloud(
            header=header,
            fields=self.pc_fields,
            points=points
        )

        return msg


def main(args=None):
    rclpy.init(args=args)

    pc_combiner = PCCombiner()

    rclpy.spin(pc_combiner)

    # Destroy the node explicitly
    pc_combiner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
