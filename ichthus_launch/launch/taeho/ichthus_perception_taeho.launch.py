import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import yaml
from launch_ros.actions import Node


class Perception:
    def __init__(self, context):
        self.context = context
        self.vehicle_info = self.get_vehicle_info()
        self.apollo_vlp16_param = os.path.join(
            get_package_share_directory('ichthus_launch'), 'param/vlp-16.param.yaml'
        )
        self.apollo_hdl64_param = os.path.join(
            get_package_share_directory('ichthus_launch'), 'param/hdl-64.param.yaml'
        )
        self.lidar_channel = LaunchConfiguration('lidar_channel').perform(self.context)
        self.apollo_lidar_model_param = self.apollo_vlp16_param if self.lidar_channel == '16' else self.apollo_hdl64_param

        self.apollo_output_topic= '/lidar_front/apollo/clusters'
        self.centerpoint_output_topic = '/lidar_front/centerpoint/objects'

        self.compressed = LaunchConfiguration('compressed').perform(self.context)
        self.raw = not self.compressed
        self.tensorrt_yolo_param_path = os.path.join(
            get_package_share_directory('ichthus_launch'), 'param/tensorrt_yolo/',LaunchConfiguration('yolo_type').perform(self.context)+'.param.yaml'
        )
        self.tensorrt_yolo_onnx_file = os.path.join(
            get_package_share_directory('tensorrt_yolo'), 'data/',LaunchConfiguration('yolo_type').perform(self.context)+'.onnx')
        self.tensorrt_yolo_engine_file = os.path.join(
            get_package_share_directory('tensorrt_yolo'), 'data/',LaunchConfiguration('yolo_type').perform(self.context)+'.engine')
        self.tensorrt_yolo_label_file = os.path.join(
            get_package_share_directory('tensorrt_yolo'), 'data/','coco.names')
        self.tensorrt_yolo_calib_image_directory = os.path.join(
            get_package_share_directory('tensorrt_yolo'), 'calib_image')
        self.tensorrt_yolo_calib_cache_file = os.path.join(
            get_package_share_directory('tensorrt_yolo'), 'data/',LaunchConfiguration('yolo_type').perform(self.context)+'.cache')

        self.lidar_centerpoint_model_name = LaunchConfiguration('model_name').perform(self.context)
        self.lidar_centerpoint_model_path = os.path.join(
            get_package_share_directory('lidar_centerpoint'), 'data'
        )
        self.lidar_centerpoint_model_param_path = os.path.join(
            get_package_share_directory('ichthus_launch') , 'param', self.lidar_centerpoint_model_name+'.param.yaml'
        )
        self.lidar_centerpoint_encoder_onnx_path = self.lidar_centerpoint_model_path + '/pts_voxel_encoder_'+self.lidar_centerpoint_model_name+'.onnx'
        self.lidar_centerpoint_encode_engine_path = self.lidar_centerpoint_model_path + '/pts_voxel_encoder_'+self.lidar_centerpoint_model_name+'.engine'
        self.lidar_centerpoint_head_onnx_path = self.lidar_centerpoint_model_path + '/pts_backbone_neck_head_'+self.lidar_centerpoint_model_name+'.onnx'
        self.lidar_centerpoint_head_engine_path = self.lidar_centerpoint_model_path + '/pts_backbone_neck_head_'+self.lidar_centerpoint_model_name+'.engine'


    def get_vehicle_info(self):
        path = LaunchConfiguration('vehicle_info_param_path').perform(self.context)
        with open(path, 'r') as f:
            p = yaml.safe_load(f)["/**"]["ros__parameters"]
            p["vehicle_length"] = p["front_overhang"] + p["wheel_base"] + p["rear_overhang"]
            p["vehicle_width"] = p["wheel_tread"] + p["left_overhang"] + p["right_overhang"]
            p["min_longitudinal_offset"] = -p["rear_overhang"]
            p["max_longitudinal_offset"] = p["front_overhang"] + p["wheel_base"]
            p["min_lateral_offset"] = -(p["wheel_tread"] / 2.0 + p["right_overhang"])
            p["max_lateral_offset"] = p["wheel_tread"] / 2.0 + p["left_overhang"]
            p["min_height_offset"] = 0.0
            p["max_height_offset"] = p["vehicle_height"] 
        return p
        
    def camera(self, compressed_image_topic, raw_image_topic, output_objects_topic):
        image_transport_decompressor = Node(
            package='image_transport_decompressor',
            executable='image_transport_decompressor_node',
            name='image_decompressor',
            parameters=[
                {"encoding": "rgb8"},
                {'use_sim_time' : LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                ('~/input/compressed_image', compressed_image_topic),
                ('~/output/raw_image', raw_image_topic)
            ],
            condition=IfCondition(LaunchConfiguration('compressed'))
        )

        tensorrt_yolo = Node(
            package='tensorrt_yolo',
            executable='tensorrt_yolo_node',
            name='tensorrt_yolo',
            parameters=[
                self.tensorrt_yolo_param_path,
                {
                    'onnx_file' : self.tensorrt_yolo_onnx_file,
                    'engine_file' : self.tensorrt_yolo_engine_file,
                    'label_file' : self.tensorrt_yolo_label_file,
                    'calib_image_directory' : self.tensorrt_yolo_calib_image_directory,
                    'calib_cache_file' : self.tensorrt_yolo_calib_cache_file,
                    'mode' : LaunchConfiguration('mode'),
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            remappings=[
                ('in/image', raw_image_topic),
                ('/out/objects', output_objects_topic)
            ],
        )

        detected_object_feature_remover = Node(
            package='detected_object_feature_remover',
            executable='detected_object_feature_remover',
            name='detected_object_feature_remover',
            remappings=[
                ('~/input', '/camera/detected_objects'),
                ('~/output', '/camera/objects')
            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        return [image_transport_decompressor, tensorrt_yolo]

    def lidar_detection(self, range_cropped_pointcloud, obstacle_segmentation_pointcloud, output_topic):

        lidar_centerpoint_param_path = LaunchConfiguration('lidar_centerpoint_param_path').perform(self.context)
        with open(lidar_centerpoint_param_path, "r") as f:
            lidar_centerpoint_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        lidar_centerpoint = Node(
            package='lidar_centerpoint',
            executable='lidar_centerpoint_node',
            name='lidar_centerpoint',
            remappings=[
                ('~/input/pointcloud', range_cropped_pointcloud),
                ('~/output/objects', output_topic)
            ],
            parameters=[
                lidar_centerpoint_param,
                {
                    'score_threshold' : 0.4,
                    'densification_world_frame_id' : 'map',
                    'densification_num_past_frames' : 1,
                    'trt_precision' : 'fp16',
                    'encoder_onnx_path' : self.lidar_centerpoint_encoder_onnx_path,
                    'encoder_engine_path' : self.lidar_centerpoint_encode_engine_path,
                    'head_onnx_path' : self.lidar_centerpoint_head_onnx_path,
                    'head_engine_path' : self.lidar_centerpoint_head_engine_path,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            condition = IfCondition(LaunchConfiguration('use_centerpoint'))
        )

        # lidar_apollo_instance_segmentation_param_path = LaunchConfiguration('lidar_apollo_instance_segmentation_param_path').perform(self.context)

        # with open(lidar_apollo_instance_segmentation_param_path, "r") as f:
        #     lidar_apollo_instance_segmentation_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        # with open(self.apollo_lidar_model_param, 'r') as f:
        #     apollo_lidar_model_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        # lidar_apollo_instance_segmentation = Node(
        #     package='lidar_apollo_instance_segmentation',
        #     executable='lidar_apollo_instance_segmentation_node',
        #     name='lidar_apollo_instance_segmentation',
        #     remappings=[
        #         ('input/pointcloud', range_cropped_pointcloud),
        #         ('output/labeled_clusters', self.apollo_output_topic),
        #     ],
        #     parameters=[
        #         {
        #         "z_offset" : -2,
        #         "target_frame" : "base_link",
        #         },
        #         lidar_apollo_instance_segmentation_param,
        #         apollo_lidar_model_param,
        #     ],
        # )

        # shape_estimation = Node(
        #     package='shape_estimation',
        #     executable='shape_estimation',
        #     name='shape_estimation',
        #     remappings=[
        #         ('input', self.apollo_output_topic),
        #         ('objects','/lidar_front/objects_with_feature')
        #     ],
        #     parameters=[
        #         {
        #             'use_filter' : True,
        #             'use_corrector' : True,
        #             'use_vehicle_reference_yaw' : True
        #         }
        #     ],
        #     condition = IfCondition(LaunchConfiguration('use_apollo'))
        # )

        # detected_object_feature_remover = Node(
        #     package='detected_object_feature_remover',
        #     executable='detected_object_feature_remover',
        #     name='detected_object_feature_remover',
        #     remappings=[
        #         ('~/input', '/lidar_front/objects_with_feature'),
        #         ('~/output', '/lidar_front/objects')
        #     ]
        #     condition = IfCondition(LaunchConfiguration('use_apollo'))
        # )


        # detected_object_validation = Node(
        #     package='detected_object_validation',
        #     executable='obstacle_pointcloud_based_validator_node',
        #     name='obstacle_pointcloud_base_validator',
        #     remappings=[
        #         ('~/input/detected_objects', self.centerpoint_output_topic),
        #         ('~/input/obstacle_pointcloud', obstacle_segmentation_pointcloud),
        #         ('~/output/objects', '/lidar_front/validation/objects')
        #     ]
        # )

        # detection_by_tracker = Node(
        #     package='detection_by_tracker',
        #     executable='detection_by_tracker',
        #     name='detection_by_tracker',
        #     remappings=[
        #         ('~/input/tracked_objects', '/perception/object_recognition/tracking/objects'),
        #         ('~/input/initial_objects', '/lidar_front/objects_with_feature'),
        #         ('~/output', '/detection_by_tracker/objects')
        #     ]
        # )

        # object_merger = Node(
        #     package='object_merger',
        #     executable='object_association_merger_node',
        #     name='object_merger',
        #     remappings=[
        #         ('input/object1', 'lidar_front/validation/objects'),
        #         ('input/object0', '/detection_by_tracker/objects'),
        #         ('output/object', '/objects')
        #     ]
        # )

        return [lidar_centerpoint]

    def occupancy_grid_map(self, no_ground_pointcloud, range_cropped_pointcloud, grid_map, obstacle_segmentation_pointcloud):

        occupancy_grid_map = Node(
            package="probabilistic_occupancy_grid_map",
            executable="pointcloud_based_occupancy_grid_map_node",
            name="occupancy_grid_map",
            remappings=[
                ("~/input/obstacle_pointcloud", no_ground_pointcloud),
                ("~/input/raw_pointcloud", range_cropped_pointcloud),
                ("~/output/occupancy_grid_map", grid_map),
            ],
            parameters=[
                {
                    "map_resolution": 0.5,
                    "use_height_filter": False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ],
            output='screen'
        )

        occupancy_grid_map_outlier_filter = Node(
            package="occupancy_grid_map_outlier_filter",
            executable="occupancy_grid_map_outlier_filter_node",
            name="occupancy_grid_map_outlier_filter",
            remappings=[
                ("~/input/occupancy_grid_map", grid_map),
                ("~/input/pointcloud", no_ground_pointcloud),
                ("~/output/pointcloud", obstacle_segmentation_pointcloud),
            ],
            parameters=[
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        return [occupancy_grid_map, occupancy_grid_map_outlier_filter]

    def lidar_tracking(self, input_topic, output_topic):

        tracker_setting_param_path = LaunchConfiguration('tracker_setting_param_path').perform(self.context)

        with open(tracker_setting_param_path, "r") as f:
            tracker_setting_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        data_association_matrix_param_path = LaunchConfiguration('data_association_matrix_param_path').perform(self.context)

        with open(data_association_matrix_param_path, "r") as f:
            data_association_matrix_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        multi_object_tracker = Node(
            package='multi_object_tracker',
            executable='multi_object_tracker',
            name='multi_object_tracker',
            remappings=[
                ('input', input_topic),
                ('output', output_topic),
            ],
            parameters=[
                {
                    'world_frame_id' : 'map',
                    'publish_rate' : 10.0, #fix to 10.0 when enable_delay_compensation is False
                    'enable_delay_compensation' : False,
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                },
                tracker_setting_param,
                data_association_matrix_param
            ],
        )

        return [multi_object_tracker]

    def lidar_prediction(self, tracked_objects_topic, predicted_object_topic):
        
        map_based_prediction_param_path = LaunchConfiguration('map_based_prediction_param_path').perform(self.context)
    
        with open(map_based_prediction_param_path, "r") as f:
            map_based_prediction_param = yaml.safe_load(f)["/**"]["ros__parameters"]

        map_based_prediction = Node(
            package='map_based_prediction',
            executable='map_based_prediction',
            name='map_based_prediction',
            remappings=[
                ('/vector_map', '/map/vector_map'),
                ('/perception/object_recognition/tracking/objects', tracked_objects_topic), #input objects
                ('objects', predicted_object_topic),
            ],
            parameters=[
                map_based_prediction_param,
                {
                    'use_sim_time' : LaunchConfiguration('use_sim_time')
                }
            ]
        )

        return [map_based_prediction]

def launch_setup(context, *args, **kwargs):
    pipeline = Perception(context)

    nodes = list()

    range_cropped_pointcloud = LaunchConfiguration('range_cropped_pointcloud_topic').perform(context)
    no_ground_pointcloud = LaunchConfiguration('no_ground_pointcloud_topic').perform(context)
    grid_map = LaunchConfiguration('grid_map_topic').perform(context)
    obstacle_segmentation_pointcloud = LaunchConfiguration('obstacle_segmentation_pointcloud_topic').perform(context)
    lidar_detected_objects_topic = LaunchConfiguration('lidar_detected_objects_topic').perform(context)
    tracked_objects_topic = LaunchConfiguration('tracked_objects_topic').perform(context)
    predicted_object_topic = LaunchConfiguration('predicted_objects_topic').perform(context)
    compressed_image_topic = LaunchConfiguration('compressed_image_topic').perform(context)
    raw_image_topic = LaunchConfiguration('raw_image_topic').perform(context)
    camera_detected_objects_topic = LaunchConfiguration('camera_detected_objects_topic').perform(context)

    lidar_detection_nodes = pipeline.lidar_detection(range_cropped_pointcloud, obstacle_segmentation_pointcloud, lidar_detected_objects_topic)
    lidar_tracking_nodes = pipeline.lidar_tracking(lidar_detected_objects_topic, tracked_objects_topic)
    lidar_prediction_nodes = pipeline.lidar_prediction(tracked_objects_topic, predicted_object_topic)
    camera_nodes = pipeline.camera(compressed_image_topic, raw_image_topic, camera_detected_objects_topic)
    nodes.extend(lidar_detection_nodes)
    nodes.extend(lidar_tracking_nodes)
    nodes.extend(lidar_prediction_nodes)
    nodes.extend(camera_nodes)

    return nodes

def generate_launch_description():

    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    vehicle_info_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/vehicle_info.param.yaml'
    )

    lidar_centerpoint_param_path_default = os.path.join(
            get_package_share_directory('ichthus_launch'), 'param/lidar_centerpoint.param.yaml'
    )

    lidar_apollo_instance_segmentation_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/lidar_apollo_instance_segmentation.param.yaml'
    )

    tracker_setting_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/default_tracker.param.yaml'
    )

    data_association_matrix_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/data_association_matrix.param.yaml'
    )

    map_based_prediction_param_path_default = os.path.join(
        get_package_share_directory('ichthus_launch'), 'param/map_based_prediction.param.yaml'
    )

    add_launch_arg('range_cropped_pointcloud_topic', '/lidar_front/range_cropped_pointcloud')
    add_launch_arg('obstacle_segmentation_pointcloud_topic', '/perception/obstacle_segmentation/pointcloud')
    add_launch_arg('lidar_detected_objects_topic', '/lidar_front/detected_objects')
    add_launch_arg('tracked_objects_topic', '/perception/object_recognition/tracking/objects')
    add_launch_arg('predicted_objects_topic', '/perception/object_recognition/objects')
    add_launch_arg('no_ground_pointcloud_topic', '/lidar_front/no_ground_pointcloud')
    add_launch_arg('grid_map_topic', '/perception/occupancy_grid_map/map')

    add_launch_arg('compressed_image_topic', '/image/compressed')
    add_launch_arg('raw_image_topic', '/image_raw')
    add_launch_arg('compressed', 'true')
    add_launch_arg('camera_detected_objects_topic', '/camera/detected_objects')
    add_launch_arg('yolo_type', 'yolov5m')
    add_launch_arg('mode', 'INT8')

    add_launch_arg('vehicle_info_param_path', vehicle_info_param_path_default),
    add_launch_arg('lidar_channel', "16")
    add_launch_arg('use_centerpoint', 'true')
    add_launch_arg('use_apollo', 'false')
    add_launch_arg('model_name', 'default')
    add_launch_arg('lidar_centerpoint_param_path', lidar_centerpoint_param_path_default)
    add_launch_arg('lidar_apollo_instance_segmentation_param_path', lidar_apollo_instance_segmentation_param_path_default)
    
    add_launch_arg('tracker_setting_param_path', tracker_setting_param_path_default)
    add_launch_arg('data_association_matrix_param_path', data_association_matrix_param_path_default)
    add_launch_arg('map_based_prediction_param_path', map_based_prediction_param_path_default)
    
    add_launch_arg('use_sim_time', 'False')

    return launch.LaunchDescription(
        launch_arguments 
        + [OpaqueFunction(function=launch_setup)]
    )