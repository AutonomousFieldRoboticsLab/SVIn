#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/Trigger.h>

#include <boost/optional.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "common/Definitions.h"
#include "pose_graph/GlobalMapping.h"
#include "pose_graph/Keyframe.h"
#include "pose_graph/Parameters.h"
#include "pose_graph/PoseGraph.h"
#include "utils/CameraPoseVisualization.h"
#include "utils/ThreadSafeQueue.h"
#include "utils/ThreadsafeTemporalBuffer.h"

class LoopClosure {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LoopClosure(std::shared_ptr<Parameters> params);
  ~LoopClosure() = default;

  void run();

  void updatePublishGlobalMap(const ros::TimerEvent& event);
  void updateGlobalMap();
  void getGlobalPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud);  // NOLINT (already pointer)
  bool savePointCloud(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);  // NOLINT
  void addPointsToGlobalMap(const int64_t keyframe_index,
                            const cv::Mat& color_image,
                            const Eigen::Matrix3d& camera_orientation,
                            const Eigen::Vector3d& camera_position,
                            const std::vector<cv::Point3f>& keyframe_points,
                            const std::vector<float>& point_qualities,
                            const std::vector<Eigen::Vector3i>& point_ids,
                            const std::vector<cv::KeyPoint>& cv_keypoints);

  inline void fillKeyframeTrackingQueue(std::unique_ptr<KeyframeInfo> keyframe_info) {
    CHECK(keyframe_info);
    keyframe_tracking_queue_.push(std::move(keyframe_info));
  }

  inline void fillImageQueue(std::unique_ptr<std::pair<ros::Time, cv::Mat>> original_image_with_timestamp) {
    CHECK(original_image_with_timestamp);
    raw_image_buffer_.addValue(original_image_with_timestamp->first.toNSec(), original_image_with_timestamp->second);
  }

  void shutdown();

  void setKeyframePoseCallback(const KeframeWithLoopClosureCallback& keyframe_pose_callback);
  void setLoopClosureCallback(const PathWithLoopClosureCallback& loop_closure_callback);

 private:
  void setup();
  static constexpr int64_t kBufferLengthNs = 3000000000;  // 3 seconds
  ros::NodeHandle nh_private_;

  std::shared_ptr<Parameters> params_;
  std::unique_ptr<PoseGraph> pose_graph_;
  std::unique_ptr<CameraPoseVisualization> camera_pose_visualizer_;
  std::unique_ptr<GlobalMap> global_map_;
  std::map<int, Keyframe*> kfMapper_;  // Mapping between kf_index and Keyframe*; to make KFcounter

  int frame_index_;
  int sequence_;
  // TODO(bjoshi): these varibales are bullshit. Keep for now
  int SKIP_CNT;
  int skip_cnt;
  double SKIP_DIS;
  Eigen::Vector3d last_translation_;

  BriefVocabulary* voc_;

  uint64_t last_keyframe_time_;
  double last_primitive_estmator_time_;
  double scale_between_vio_prim_;
  uint64_t consecutive_tracking_failures_, consecutive_tracking_successes_;
  Eigen::Matrix4d last_t_w_svin_, last_t_w_prim_;
  Eigen::Matrix4d init_t_w_prim_, init_t_w_svin_;
  Eigen::Matrix4d switch_svin_pose_, switch_prim_pose_, switch_uber_pose_, last_scaled_prim_pose_;

  std::vector<geometry_msgs::PoseStamped> primitive_estimator_poses_;
  std::vector<geometry_msgs::PoseStamped> uber_estimator_poses_;

  TrackingStatus tracking_status_;
  uint32_t prim_estimator_keyframes_;
  double vio_traj_length_;
  double prim_traj_length_;

  void updatePrimiteEstimatorTrajectory(const nav_msgs::OdometryConstPtr& prim_estimator_odom_msg);
  void setupOutputLogDirectories();
  bool healthCheck(const okvis_ros::SvinHealthConstPtr& health_msg, boost::optional<std::string> error_msg);  // NOLINT

  ThreadsafeQueue<std::unique_ptr<KeyframeInfo>> keyframe_tracking_queue_;
  utils::ThreadsafeTemporalBuffer<cv::Mat> raw_image_buffer_;

  bool shutdown_;
};
