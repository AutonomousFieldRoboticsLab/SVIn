from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare
import os


def workspace_rerun_executable(context):
  """Resolve <colcon-workspace>/.venv/bin/rerun from this package prefix."""
  package_prefix = os.path.normpath(
    FindPackagePrefix('okvis_ros').perform(context))
  install_prefix = (
    os.path.dirname(package_prefix)
    if os.path.basename(package_prefix) == 'okvis_ros'
    else package_prefix)
  return os.path.join(os.path.dirname(install_prefix), '.venv', 'bin', 'rerun')


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
        LaunchConfiguration('left_compressed_image_topic'),
        LaunchConfiguration('right_compressed_image_topic')
      ],
      'synchronized_compressed_queue_size': 100,
      'synchronized_compressed_log_counters': False,
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
      'compressed_img_topic': LaunchConfiguration('left_compressed_image_topic'),
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

  rerun_node = Node(
    package='svin_rerun_bridge',
    executable='svin_rerun_bridge_node',
    name='svin_rerun_bridge',
    condition=IfCondition(LaunchConfiguration('use_rerun')),
    parameters=[LaunchConfiguration('rerun_config').perform(context), {
      'enable_live': LaunchConfiguration('rerun_enable_live'),
      'enable_recording': LaunchConfiguration('rerun_enable_recording'),
      'rrd_path': LaunchConfiguration('rerun_rrd_path'),
      'viewer_url': LaunchConfiguration('rerun_viewer_url'),
      'raw_compressed_image_topics': [
        LaunchConfiguration('left_compressed_image_topic').perform(context),
        LaunchConfiguration('right_compressed_image_topic').perform(context)
      ],
      'keyframe_image_count': 2,
      'okvis_config': abs_okvis_config_path
    }],
    output='screen'
  )

  actions = [okvis_node, pose_graph_node, left_uncompressor_node, rviz_node]
  use_rerun = LaunchConfiguration('use_rerun').perform(context).lower() in (
    '1', 'true', 'yes', 'on')
  enable_live = LaunchConfiguration('rerun_enable_live').perform(context).lower() in (
    '1', 'true', 'yes', 'on')
  start_viewer = LaunchConfiguration('rerun_start_viewer').perform(context).lower() in (
    '1', 'true', 'yes', 'on')
  if use_rerun and enable_live and start_viewer:
    rerun_executable = LaunchConfiguration('rerun_executable').perform(context)
    if not rerun_executable:
      rerun_executable = workspace_rerun_executable(context)
    viewer_mode = LaunchConfiguration('rerun_viewer_mode').perform(context).lower()
    viewer_port = LaunchConfiguration('rerun_viewer_port').perform(context)
    if viewer_mode == 'native':
      viewer_cmd = [
        rerun_executable,
        '--renderer', LaunchConfiguration('rerun_native_renderer').perform(context),
        '--port', viewer_port
      ]
    elif viewer_mode == 'web':
      viewer_cmd = [
        rerun_executable,
        '--web-viewer',
        '--web-viewer-port', LaunchConfiguration('rerun_web_viewer_port').perform(context),
        '--port', viewer_port
      ]
    else:
      raise RuntimeError(
        "rerun_viewer_mode must be 'native' or 'web', got " + repr(viewer_mode))
    actions.append(ExecuteProcess(cmd=viewer_cmd, output='screen'))

  actions.append(rerun_node)
  return actions


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

  use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
  use_rerun_arg = DeclareLaunchArgument('use_rerun', default_value='true')
  rerun_config_arg = DeclareLaunchArgument(
    'rerun_config',
    default_value=PathJoinSubstitution([
      FindPackageShare('svin_rerun_bridge'),
      'config',
      'svin_rerun_bridge_pinhole.yaml'
    ]),
    description='Reusable pinhole Rerun visualization configuration')
  rerun_enable_live_arg = DeclareLaunchArgument('rerun_enable_live', default_value='true')
  rerun_start_viewer_arg = DeclareLaunchArgument('rerun_start_viewer', default_value='true')
  rerun_viewer_mode_arg = DeclareLaunchArgument('rerun_viewer_mode', default_value='native')
  rerun_native_renderer_arg = DeclareLaunchArgument(
    'rerun_native_renderer', default_value='vulkan')
  rerun_executable_arg = DeclareLaunchArgument(
    'rerun_executable', default_value='',
    description='Empty resolves <workspace>/.venv/bin/rerun')
  rerun_viewer_port_arg = DeclareLaunchArgument('rerun_viewer_port', default_value='9876')
  rerun_web_viewer_port_arg = DeclareLaunchArgument(
    'rerun_web_viewer_port', default_value='9090')
  rerun_enable_recording_arg = DeclareLaunchArgument(
    'rerun_enable_recording', default_value='false')
  rerun_rrd_path_arg = DeclareLaunchArgument('rerun_rrd_path', default_value='')
  rerun_viewer_url_arg = DeclareLaunchArgument(
    'rerun_viewer_url', default_value='rerun+http://127.0.0.1:9876/proxy')
  left_compressed_image_topic_arg = DeclareLaunchArgument(
    'left_compressed_image_topic',
    default_value='/a6/camera/left/image_raw/compressed')
  right_compressed_image_topic_arg = DeclareLaunchArgument(
    'right_compressed_image_topic',
    default_value='/a6/camera/right/image_raw/compressed')

  return LaunchDescription([
    config_arg,
    use_pose_graph_arg,
    use_rviz_arg,
    use_rerun_arg,
    rerun_config_arg,
    rerun_enable_live_arg,
    rerun_start_viewer_arg,
    rerun_viewer_mode_arg,
    rerun_native_renderer_arg,
    rerun_executable_arg,
    rerun_viewer_port_arg,
    rerun_web_viewer_port_arg,
    rerun_enable_recording_arg,
    rerun_rrd_path_arg,
    rerun_viewer_url_arg,
    left_compressed_image_topic_arg,
    right_compressed_image_topic_arg,
    OpaqueFunction(function=launch_setup)
  ])
