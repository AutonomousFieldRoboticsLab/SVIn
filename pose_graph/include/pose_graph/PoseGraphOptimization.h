#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <std_srvs/Trigger.h>

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "common/Definitions.h"
#include "pose_graph/GlobalMapping.h"
#include "pose_graph/KFMatcher.h"
#include "pose_graph/LoopClosing.h"
#include "pose_graph/Parameters.h"
#include "pose_graph/Publisher.h"
#include "utils/CameraPoseVisualization.h"
#include "utils/ThreadSafeQueue.h"

class PoseGraphOptimization {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseGraphOptimization(const Parameters& params);
  ~PoseGraphOptimization() = default;

  void setup();
  void run();

  void updatePublishGlobalMap(const ros::TimerEvent& event);
  void updateGlobalMap();
  void getGlobalPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud);  // NOLINT (already pointer)
  bool savePointCloud(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response);  // NOLINT
  Publisher publisher;

  // ros::Publisher pubSparseMap;

  inline void fillKeyframeTrackingQueue(std::unique_ptr<KeyframeInfo> keyframe_info) {
    CHECK(keyframe_info);
    keyframe_tracking_queue_.push(std::move(keyframe_info));
  }

  void shutdown();

 private:
  ros::NodeHandle nh_private_;

  std::shared_ptr<Parameters> params_;
  std::unique_ptr<LoopClosing> loop_closing_;
  std::unique_ptr<CameraPoseVisualization> camera_pose_visualizer_;
  std::unique_ptr<GlobalMap> global_map_;
  std::map<int, KFMatcher*> kfMapper_;  // Mapping between kf_index and KFMatcher*; to make KFcounter

  std::mutex processMutex_;
  int frame_index_;
  int sequence_;
  // TODO(bjoshi): these varibales are bullshit. Keep for now
  int SKIP_CNT;
  int skip_cnt;
  double SKIP_DIS;
  Eigen::Vector3d last_translation_;

  BriefVocabulary* voc_;

  ros::Timer timer_;  // for periodic publishing
  ros::ServiceServer save_pointcloud_service_;

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
  bool healthCheck(const okvis_ros::SvinHealthConstPtr& health_msg, std::string& error_msg);  // NOLINT

  ThreadsafeQueue<std::unique_ptr<KeyframeInfo>> keyframe_tracking_queue_;

  bool shutdown_;
};
