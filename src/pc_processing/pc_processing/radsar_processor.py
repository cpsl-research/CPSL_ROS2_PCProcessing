import rclpy
import rclpy.duration
from rclpy.node import Node,ParameterDescriptor,ParameterType
import rclpy.publisher
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rclpy.subscription
import rclpy.time
import rclpy.timer
from sensor_msgs.msg import PointCloud2,PointField
from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from raw_radar_msgs.msg import ADCDataCube

import numpy as np

#pose tracking
from geometries.pose.pose import Pose
from geometries.pose.orientation import Orientation
from geometries.pose.position import Position

#model running code
from mmwave_radar_processing.config_managers.cfgManager import ConfigManager
from mmwave_model_integrator.model_runner.radsar_runner import RadSARRunner
from mmwave_model_integrator.decoders.radsar_decoder import RadSARDecoder
from mmwave_radar_processing.processors.synthetic_array_beamformer_processor_revA import SyntheticArrayBeamformerProcessor
from mmwave_model_integrator.input_encoders.radsar_encoder import RadSAREncoder


class RadSARProcessor(Node):
    """A node that subscribesto a raw adc_data_cube topic and 
    a vehicle's odometry to publish a higher resolution point cloud
    """
    def __init__(self):
        super().__init__('radsar_processor')

        #ROS2 parameters
        self.radar_raw_adc_topic:str = ""
        self.radar_config_path:str = ""
        self.odom_sub_topic:str = ""
        self.point_cloud_pub_topic:str = ""
        self.state_dict_path:str = ""

        #defining qos profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        #subscribers
        self.radar_raw_adc_sub:rclpy.subscription.Subscription=None
        self.odom_sub:rclpy.subscription.Subscription = None

        #publishers
        self.point_cloud_pub:rclpy.publisher.Publisher = None

        #point cloud publishing information
        self.pc_fields:list = None
        self.last_pc2_msg:PointCloud2 = PointCloud2()
        
        #storing last odom data message
        self.odom_sub_latest:Odometry = None
        self.current_pose:Pose = Pose()
        self.latest_vel:np.ndarray = np.array([0.0,0.0,0.0])

        #point cloud integrator object
        self.config_manager:ConfigManager = None
        self.input_encoder:RadSAREncoder = None
        self.runner:RadSARRunner = None
        self.prediction_decoder:RadSARDecoder = None

        #initialize the node
        self.init_params() #parameters
        self.init_radsar() #pc integration
        self.init_pubs() #publishers
        self.init_subs() #point cloud subscribers

        return

    
    def init_radsar(self):
        
        #initialize the configuration manager
        self.config_manager = ConfigManager()
        self.config_manager.load_cfg(self.radar_config_path)
        self.config_manager.compute_radar_perforance(profile_idx=0)

        #initialize the input encoder
        self.input_encoder = RadSAREncoder(
            config_manager=self.config_manager,
            az_angle_bins_rad=\
                np.deg2rad(np.linspace(
                    start=-90,stop=90,num=90
                    )),
            min_vel=0.2,
            max_vel_change=0.1,
            mode = SyntheticArrayBeamformerProcessor.ENDFIRE_MODE,
            min_power_threshold_dB=40
        )

        #initialize the runner
        self.runner = RadSARRunner(
            state_dict_path=self.state_dict_path,
            cuda_device="cpu"
        )

        self.prediction_decoder = RadSARDecoder(
            max_range_m=self.config_manager.range_max_m,
            num_range_bins=self.config_manager.get_num_adc_samples(),
            angle_range_rad=[np.deg2rad(-90),np.deg2rad(90)],
            num_angle_bins=90
        )

        #initialize the point fields
        self.pc_fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)
        ]

        #configure an initial point cloud
        zero_pc = np.zeros(shape=(1,3),dtype=np.float32)
        header_msg = Header()
        header_msg.frame_id = "cpsl_uav_1/base_footprint"
        self.last_pc2_msg = self.np_to_pointcloud2(
            points=zero_pc,
            header=header_msg,
            fields=self.pc_fields
        )

        self.get_logger().info("Initialized radsar model")

    
    def init_params(self):
        """Declare and set all of the ROS2 parameters
        """
        # ROS2 parameters
        self.declare_parameter(
            name='radar_raw_adc_topic',
            value='radar_0/adc_data_cube',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The ADCDataCube topic to read radar detections from'
            )
        )

        self.declare_parameter(
            name='radar_config_path',
            value='',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The path to the mmWave radar .cfg file the radar is currently running'
            )
        )

        self.declare_parameter(
            name='odom_sub_topic',
            value='/odom',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The Odometry topic to read radar detections from'
            )
        )
        self.declare_parameter(
            name='point_cloud_pub_topic',
            value='radar_0/radsar_point_cloud',
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='The pointCloud2 to publish the final processed point cloud to'
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
        self.radar_raw_adc_topic = \
            self.get_parameter('radar_raw_adc_topic').get_parameter_value().string_value
        self.radar_config_path = \
            self.get_parameter('radar_config_path').get_parameter_value().string_value
        self.odom_sub_topic = \
            self.get_parameter('odom_sub_topic').get_parameter_value().string_value
        self.point_cloud_pub_topic = \
            self.get_parameter('point_cloud_pub_topic').get_parameter_value().string_value
        self.state_dict_path = \
            self.get_parameter('state_dict_path').get_parameter_value().string_value

        #log the parameter values
        self.get_logger().info(f'radar_raw_adc_topic set to: {self.radar_raw_adc_topic}')
        self.get_logger().info(f'radar_config_path set to: {self.radar_config_path}')
        self.get_logger().info(f'odom_sub_topic set to: {self.odom_sub_topic}')
        self.get_logger().info(f'point_cloud_pub_topic set to: {self.point_cloud_pub_topic}')
        self.get_logger().info(f'state_dict_path set to: {self.state_dict_path}')

    def init_subs(self):
        """Initialize the point cloud subscribers based on the given in_point_cloud_topics list
        """

        #point cloud subscriber
        self.radar_raw_adc_sub = self.create_subscription(
            msg_type=ADCDataCube,
            topic=self.radar_raw_adc_topic,
            callback=self.radar_raw_adc_callback,
            qos_profile=self.qos_profile
        )

        #odom subscriber
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic=self.odom_sub_topic,
            callback=self.odom_sub_callback,
            qos_profile=self.qos_profile
        )

    def radar_raw_adc_callback(self,msg:ADCDataCube):
        """When a new ADC data cube is received, decode it, and then run it through the model

        Args:
            msg (PointCloud2): the most recent point cloud from the subscription
        """
        
        adc_cube = self.adc_cube_msg_to_np(msg)
        
        #attempt to get an encoding
        resp = self.input_encoder.encode(
            adc_data_cube=adc_cube,
            vels=self.latest_vel
        )

        if self.input_encoder.full_encoding_ready:
            
            pred = self.runner.make_prediction(input=resp)

            pc = self.prediction_decoder.convert_polar_to_cartesian(
                self.prediction_decoder.decode(pred)
            )

            if pc.shape[0] > 0:
                pc_3d = np.hstack([
                        pc.astype(np.float32), 
                        np.zeros((pc.shape[0], 1), dtype=np.float32)
                    ]).astype(np.float32)

                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = "cpsl_uav_1/base_footprint"#msg.header.frame_id

                pc2_msg = self.np_to_pointcloud2(
                    points=pc_3d,
                    header=header,
                    fields=self.pc_fields
                )

                self.point_cloud_pub.publish(pc2_msg)

                self.last_pc2_msg = pc2_msg

            self.get_logger().info("RadSAR: Sent PC with {} points".format(
                pc.shape[0]
            ))
        else:
            
            self.point_cloud_pub.publish(self.last_pc2_msg)
            self.get_logger().info("Latest velocity: {}".format(self.latest_vel))

            return
            
        # else:
        #     new_msg = pc2.create_cloud(
        #         header=msg.header,
        #         fields=msg.fields,
        #         points=[]
        #     )

        return

    def odom_sub_callback(self,msg:Odometry):
        """Use the most recent Odometry message to update the vehicle velocity variable

        Args:
            msg (Odometry): the latest Odometry message from the subscriber
        """
        #update the vehicle's velocity
        self.latest_vel = np.abs(np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]))

        vehicle_rot = np.abs(np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ]))

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
    
    def adc_cube_msg_to_np(self,msg:ADCDataCube)->np.ndarray:
        """Convert an ADCDataCube message into a numpy array

        Args:
            msg (ADCDataCube): ADC data cube message

        Returns:
            np.ndarray: ADC data cube indexed by [rx,sample,chirp]
        """

        #get the real data
        real_data = np.array(msg.real_data)
        real_data = real_data.reshape(
            (msg.layout.dim[0].size,
            msg.layout.dim[1].size,
            msg.layout.dim[2].size)
        )

        #get the imag data
        imag_data = np.array(msg.imag_data)
        imag_data = imag_data.reshape(
            (msg.layout.dim[0].size,
            msg.layout.dim[1].size,
            msg.layout.dim[2].size)
        )

        data = real_data + 1j * imag_data

        return data

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

    radsar_processor = RadSARProcessor()

    rclpy.spin(radsar_processor)

    # Destroy the node explicitly
    radsar_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
