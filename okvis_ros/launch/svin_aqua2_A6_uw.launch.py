from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
  # Resolve full path to config file
  okvis_config_rel = LaunchConfiguration('okvis_config').perform(context)
  launch_file_dir = ThisLaunchFileDir().perform(context)
  abs_okvis_config_path = os.path.abspath(os.path.join(launch_file_dir, okvis_config_rel))

  print(f"\n[INFO] Full path to okvis_config: {abs_okvis_config_path}\n")

  # OKVIS node
  okvis_node = Node(
    package='okvis_ros',
    executable='okvis_node',
    name='okvis_node',
    parameters=[{
      'config_filename': abs_okvis_config_path,
      'mesh_file': 'firefly.dae',
      'synchronized_compressed_images': True,
      'synchronized_compressed_camera_topics': [
        '/a6/camera/left/image_raw/compressed',
        '/a6/camera/right/image_raw/compressed'
      ],
      'synchronized_compressed_queue_size': 100,
      'synchronized_compressed_log_counters': True
    }],
    remappings=[
      ('/imu', '/a6/imu/imu/data')
    ]
  )

  # Pose Graph node
  pose_graph_node = Node(
    package='pose_graph',
    executable='pose_graph_node',
    name='pose_graph_node',
    parameters=[{
      'config_file': abs_okvis_config_path
    }],
    condition=IfCondition(LaunchConfiguration('use_pose_graph'))
  )

  # The pose graph's global mapper subscribes to /cam0/image_raw for landmark
  # colourization. OKVIS continues to decode the synchronized compressed
  # streams internally, so this extra raw stream is only needed by pose graph.
  left_uncompressor_node = Node(
    package='okvis_ros',
    executable='uncompress_image',
    name='left_uncompressor',
    condition=IfCondition(LaunchConfiguration('use_pose_graph')),
    output='screen',
    parameters=[{
      'compressed_img_topic': '/a6/camera/left/image_raw/compressed',
      'ouput_img_topic': '/cam0/image_raw'
    }]
  )

  # RViz node
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    arguments=['-d', os.path.join(
      FindPackageShare('okvis_ros').perform(context),
      'rviz_config/svin.rviz')],
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_rviz'))
  )

  return [okvis_node, pose_graph_node, left_uncompressor_node, rviz_node]


def generate_launch_description():
  # Declare launch argument for config path
  config_arg = DeclareLaunchArgument(
    'okvis_config',
    default_value=PathJoinSubstitution([
      FindPackageShare('okvis_ros'),
      'config',
      'config_aqua2_A6_BBDOS26_1280_720.yaml',
    ])
  )

  use_pose_graph_arg = DeclareLaunchArgument(
    'use_pose_graph',
    default_value='true'
  )

  use_rviz_arg = DeclareLaunchArgument(
    'use_rviz',
    default_value='true'
  )

  return LaunchDescription([
    config_arg,
    use_pose_graph_arg,
    use_rviz_arg,
    OpaqueFunction(function=launch_setup)
  ])
