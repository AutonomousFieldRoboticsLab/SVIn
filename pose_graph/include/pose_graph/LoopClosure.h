#pragma once

#include <glog/logging.h>
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
#include "pose_graph/Keyframe.h"
#include "pose_graph/Parameters.h"
#include "pose_graph/PoseGraph.h"
#include "pose_graph/SwitchingEstimator.h"
#include "utils/ThreadSafeQueue.h"
#include "utils/ThreadsafeTemporalBuffer.h"

class LoopClosure {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LoopClosure(Parameters& params);
  ~LoopClosure() = default;

  void run();

  void getGlobalMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud);
  void updateGlobalMap();
  void getGlobalPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud);  // NOLINT (already pointer)
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
    if (params_.loop_closure_params_.keyframe_queue_size > 0) {
      keyframe_tracking_queue_.pushOverflowIfFull(std::move(keyframe_info),
                                                  params_.loop_closure_params_.keyframe_queue_size);
    } else {
      keyframe_tracking_queue_.push(std::move(keyframe_info));
    }
  }

  inline void fillImageQueue(std::unique_ptr<std::pair<Timestamp, cv::Mat>> original_image_with_timestamp) {
    CHECK(original_image_with_timestamp);
    raw_image_buffer_.addValue(original_image_with_timestamp->first, original_image_with_timestamp->second);
  }

  inline void fillPrimitiveEstimatorBuffer(std::unique_ptr<std::pair<Timestamp, cv::Mat>> primitive_estimator_pose) {
    DLOG(INFO) << "Filling primitive estimator buffer";
    primitive_estimator_poses_buffer_.addValue(primitive_estimator_pose->first, primitive_estimator_pose->second);
  }

  void shutdown();

  void setKeyframePoseCallback(const KeframeWithLoopClosureCallback& keyframe_pose_callback);
  void setLoopClosureCallback(const PathWithLoopClosureCallback& loop_closure_callback);
  inline void setPrimitivePublishCallback(const PoseCallback& primitive_publish_callback) {
    primitive_publish_callback_ = primitive_publish_callback;
    DLOG(INFO) << "Primitive publish callback set";
  }

 private:
  void setup();
  static constexpr int64_t kBufferLengthNs = 2000000000;  // 2 seconds

  Parameters params_;
  std::unique_ptr<PoseGraph> pose_graph_;
  std::unique_ptr<GlobalMap> global_map_;
  std::unique_ptr<SwitchingEstimator> switching_estimator_;

  std::map<int, Keyframe*> kfMapper_;  // Mapping between kf_index and Keyframe*; to make KFcounter

  int frame_index_;
  int sequence_;
  // TODO(bjoshi): these varibales are bullshit. Keep for now
  int SKIP_CNT;
  int skip_cnt;
  double SKIP_DIS;
  Eigen::Vector3d last_translation_;

  BriefVocabulary* voc_;

  std::vector<std::pair<Timestamp, Eigen::Matrix4d>> primitive_estimator_poses_;
  std::vector<std::pair<Timestamp, Eigen::Matrix4d>> robust_estimator_poses_;

  TrackingStatus tracking_status_;
  uint32_t prim_estimator_keyframes_;
  double vio_traj_length_;
  double prim_traj_length_;

  // void updatePrimiteEstimatorTrajectory(const nav_msgs::OdometryConstPtr& prim_estimator_odom_msg);
  bool healthCheck(const TrackingInfo& tracking_info, std::string error_message);

  ThreadsafeQueue<std::unique_ptr<KeyframeInfo>> keyframe_tracking_queue_;
  utils::ThreadsafeTemporalBuffer<cv::Mat> raw_image_buffer_;

  // TODO(bjoshi): I had issues with using pointers to Eigen::Matrix4d. So using cv::Mat for now
  utils::ThreadsafeTemporalBuffer<cv::Mat> primitive_estimator_poses_buffer_;

  bool shutdown_;

  PoseCallback primitive_publish_callback_;
};
