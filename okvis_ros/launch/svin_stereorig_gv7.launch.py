from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
      'mesh_file': 'firefly.dae'
    }],
    remappings=[
      ('/camera0', '/cam0/image_raw'),
      ('/camera1', '/cam1/image_raw'),
      ('/imu', '/imu/data'),
      ('/depth', '/bar30/depth')
    ]
  )

  # Pose Graph node
  pose_graph_node = Node(
    package='pose_graph',
    executable='pose_graph_node',
    name='pose_graph_node',
    parameters=[{
      'config_file': abs_okvis_config_path
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
    output='screen'
  )

  return [okvis_node, pose_graph_node, rviz_node]


def generate_launch_description():
  # Declare launch argument for config path
  config_arg = DeclareLaunchArgument(
    'okvis_config',
    default_value=PathJoinSubstitution([
      FindPackageShare('okvis_ros'),
      'config',
      'config_flir_stereo_gv7.yaml'
    ])
  )

  # To un-compress and sync the image topics
  stereo_sync_node = Node(
    package='okvis_ros',
    executable='stereo_sync',
    name='stereo_sync',
    output='screen',
    parameters=[{
      'config_filename': LaunchConfiguration('okvis_config'),
      'left_img_topic': '/left/image_raw',
      'right_img_topic': '/right/image_raw',
      'compressed': True
    }]
  )

  return LaunchDescription([
    config_arg,
    stereo_sync_node,
    OpaqueFunction(function=launch_setup)
  ])
