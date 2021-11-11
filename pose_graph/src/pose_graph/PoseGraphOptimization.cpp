#include "pose_graph/PoseGraphOptimization.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

PoseGraphOptimization::PoseGraphOptimization()
    : nh_private_("~"),
      params_(nullptr),
      loop_closing_(nullptr),
      camera_pose_visualizer_(nullptr),
      subscriber_(nullptr),
      global_map_(nullptr) {
  frame_index_ = 0;
  sequence_ = 1;

  last_translation_ = Eigen::Vector3d(-100, -100, -100);
  SKIP_CNT = 0;
  skip_cnt = 0;

  SKIP_DIS = 0;

  params_ = std::make_shared<Parameters>();
  params_->loadParameters(nh_private_);

  setup();

  // pubSparseMap = nh_private_.advertise<sensor_msgs::PointCloud2>("sparse_pointcloud", 10);

  save_pointcloud_service_ =
      nh_private_.advertiseService("save_pointcloud", &PoseGraphOptimization::savePointCloud, this);
}

void PoseGraphOptimization::setup() {
  global_map_ = std::unique_ptr<GlobalMap>(new GlobalMap());

  loop_closing_ = std::unique_ptr<LoopClosing>(new LoopClosing());
  loop_closing_->setPublishers(nh_private_);
  loop_closing_->set_svin_results_file(params_->svin_w_loop_path_);
  loop_closing_->set_fast_relocalization(params_->fast_relocalization_);

  loop_closing_->registerLoopClosureOptimizationCallback(
      std::bind(&GlobalMap::loopClosureOptimizationFinishCallback, global_map_.get(), std::placeholders::_1));

  camera_pose_visualizer_ = std::unique_ptr<CameraPoseVisualization>(new CameraPoseVisualization(1, 0, 0, 1));
  camera_pose_visualizer_->setScale(params_->camera_visual_size_);
  camera_pose_visualizer_->setLineWidth(params_->camera_visual_size_ / 10.0);

  // Loading vocabulary
  voc_ = new BriefVocabulary(params_->vocabulary_file_);
  BriefDatabase db;
  db.setVocabulary(*voc_, false, 0);
  loop_closing_->setBriefVocAndDB(voc_, db);

  subscriber_ = std::unique_ptr<Subscriber>(new Subscriber(nh_private_, *params_));
  publisher.setParameters(*params_);
  publisher.setPublishers();

  timer_ = nh_private_.createTimer(ros::Duration(2), &PoseGraphOptimization::updatePublishGlobalMap, this);
}

void PoseGraphOptimization::run() {
  while (true) {
    sensor_msgs::ImageConstPtr image_msg = nullptr;
    sensor_msgs::PointCloudConstPtr point_msg = nullptr;
    nav_msgs::Odometry::ConstPtr pose_msg = nullptr;
    okvis_ros::SvinHealthConstPtr health_msg = nullptr;

    subscriber_->getSyncMeasurements(image_msg, pose_msg, point_msg, health_msg);

    if (pose_msg) {
      if (skip_cnt < SKIP_CNT) {
        skip_cnt++;
        continue;
      } else {
        skip_cnt = 0;
      }

      // build keyframe
      Vector3d T =
          Vector3d(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
      Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                               pose_msg->pose.pose.orientation.x,
                               pose_msg->pose.pose.orientation.y,
                               pose_msg->pose.pose.orientation.z)
                       .toRotationMatrix();

      // std::cout << "T: " << T.transpose() << std::endl;
      // std::cout << "R: " << R << std::endl;

      assert(point_msg);
      assert(image_msg);

      if ((T - last_translation_).norm() > SKIP_DIS) {
        vector<cv::Point3f> point_3d;
        vector<cv::KeyPoint> point_2d_uv;
        vector<Eigen::Vector3d> point_ids;  // @Reloc: landmarkId, mfId, keypointIdx related to each point
        // For every KF, a map <observed_kf, weight> describing which other kfs how many MapPoints are common.
        map<KFMatcher*, int> KFcounter;

        int kf_index = -1;
        cv::Mat kf_image = subscriber_->readRosImage(image_msg);
        cv::Mat orig_color_image = subscriber_->getCorrespondingImage(pose_msg->header.stamp.toNSec());
        cv::Mat undistort_image;
        cv::remap(orig_color_image,
                  undistort_image,
                  params_->cam0_undistort_map_x_,
                  params_->cam0_undistort_map_y_,
                  cv::INTER_LINEAR);

        // cv::imshow("image", orig_image);
        // cv::waitKey(1);

        // Need this because we want to update global map only if there is new loop
        // LoopClosing::addKFToPoseGraph updates keyframe to account for loop closure correction
        // Hence, need to add global points after the function.

        std::vector<uint64_t> landmark_ids;
        std::vector<double> qualities;
        std::vector<Eigen::Vector3d> colors;
        std::vector<Eigen::Vector3d> local_positions;

        for (unsigned int i = 0; i < point_msg->points.size(); i++) {
          double quality = point_msg->channels[i].values[3];
          if (quality < 1e-6) continue;

          cv::Point3f p_3d;
          p_3d.x = point_msg->points[i].x;
          p_3d.y = point_msg->points[i].y;
          p_3d.z = point_msg->points[i].z;
          point_3d.push_back(p_3d);

          Eigen::Vector3d point_3d_eigen;
          point_3d_eigen << p_3d.x, p_3d.y, p_3d.z;

          Eigen::Vector3d point_cam_frame = R.transpose() * (point_3d_eigen - T);
          // pcl::PointXYZRGB pcl_point;
          // pcl_point.x = point_cam_frame.x();
          // pcl_point.y = point_cam_frame.y();
          // pcl_point.z = point_cam_frame.z();
          // pcl_point.r = 255;
          // pcl_point.g = 0;
          // pcl_point.b = 0;
          // sparse_pointcloud_->push_back(pcl_point);

          // @Reloc
          Eigen::Vector3d p_ids;
          p_ids(0) = point_msg->channels[i].values[0];  // landmarkId
          p_ids(1) = point_msg->channels[i].values[1];  // poseId or MultiFrameId
          p_ids(2) = point_msg->channels[i].values[2];  // keypointIdx
          point_ids.push_back(p_ids);

          cv::KeyPoint p_2d_uv;
          double p_id;
          kf_index = point_msg->channels[i].values[4];
          p_2d_uv.pt.x = point_msg->channels[i].values[5];
          p_2d_uv.pt.y = point_msg->channels[i].values[6];
          p_2d_uv.size = point_msg->channels[i].values[7];
          p_2d_uv.angle = point_msg->channels[i].values[8];
          p_2d_uv.octave = point_msg->channels[i].values[9];
          p_2d_uv.response = point_msg->channels[i].values[10];
          p_2d_uv.class_id = point_msg->channels[i].values[11];

          point_2d_uv.push_back(p_2d_uv);

          // std::cout << "CV Keypoint of size 8:" << p_2d_uv.pt.x << " , " << p_2d_uv.pt.y << " size: " << p_2d_uv.size
          //           << "angle : " << p_2d_uv.angle << " octave : " << p_2d_uv.octave
          //           << " response : " << p_2d_uv.response << " class_id: " << p_2d_uv.class_id << std::endl;

          cv::Vec3b color =
              undistort_image.at<cv::Vec3b>(static_cast<uint16_t>(p_2d_uv.pt.y), static_cast<uint16_t>(p_2d_uv.pt.x));
          Eigen::Vector3d color_eigen(
              static_cast<double>(color[2]), static_cast<double>(color[1]), static_cast<double>(color[0]));

          colors.push_back(color_eigen);
          qualities.push_back(quality);
          local_positions.push_back(point_cam_frame);
          landmark_ids.push_back(static_cast<uint64_t>(p_ids(0)));

          for (size_t sz = 12; sz < point_msg->channels[i].values.size(); sz++) {
            int observed_kf_index = point_msg->channels[i].values[sz];  // kf_index where this point_3d has been
                                                                        // observed
            if (observed_kf_index == kf_index) {
              continue;
            }

            map<int, KFMatcher*>::iterator mkfit;
            mkfit = kfMapper_.find(observed_kf_index);
            if (mkfit == kfMapper_.end()) {
              continue;
            }

            KFMatcher* observed_kf =
                kfMapper_.find(observed_kf_index)->second;  // Keyframe where this point_3d has been observed
            KFcounter[observed_kf]++;
          }
        }

        KFMatcher* keyframe = new KFMatcher(pose_msg->header.stamp.toSec(),
                                            point_ids,
                                            kf_index,
                                            T,
                                            R,
                                            kf_image,
                                            point_3d,
                                            point_2d_uv,
                                            KFcounter,
                                            sequence_,
                                            voc_,
                                            *params_);

        keyframe->setRelocalizationPCLCallback(
            std::bind(&Publisher::kfMatchedPointCloudCallback, &publisher, std::placeholders::_1));
        kfMapper_.insert(std::make_pair(kf_index, keyframe));

        {
          std::lock_guard<std::mutex> l(processMutex_);
          // start_flag = 1;
          loop_closing_->addKFToPoseGraph(keyframe, 1);
        }

        // sensor_msgs::PointCloud2 sparse_pointcloud_msg;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr sparse_pointcloud_(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < landmark_ids.size(); i++) {
          Eigen::Vector3d pos_cam_frame = local_positions.at(i);

          if (kfMapper_.find(kf_index) == kfMapper_.end()) {
            cout << "Keyframe not found" << endl;
            continue;
          }

          KFMatcher* kf = kfMapper_.find(kf_index)->second;
          Eigen::Matrix3d R_w_kf;
          Eigen::Vector3d T_w_kf;
          kf->getPose(T_w_kf, R_w_kf);

          Eigen::Vector3d global_pos = R_w_kf * pos_cam_frame + T_w_kf;
          Eigen::Vector3d color = colors.at(i);
          double quality = qualities.at(i);
          uint64_t landmark_id = landmark_ids.at(i);

          global_map_->addLandmark(global_pos, landmark_id, quality, kf_index, pos_cam_frame, color);

          // pcl::PointXYZRGB pcl_point;
          // pcl_point.x = global_pos(0);
          // pcl_point.y = global_pos(1);
          // pcl_point.z = global_pos(2);
          // pcl_point.r = (int)color(0);
          // pcl_point.g = (int)color(1);
          // pcl_point.b = (int)color(2);
          // sparse_pointcloud_->push_back(pcl_point);
        }

        frame_index_++;
        last_translation_ = T;

        // pcl::toROSMsg(*sparse_pointcloud_, sparse_pointcloud_msg);
        // sparse_pointcloud_msg.header.frame_id = "world";
        // sparse_pointcloud_msg.header.stamp = pose_msg->header.stamp;
        // pubSparseMap.publish(sparse_pointcloud_msg);
      }
    }

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}

void PoseGraphOptimization::updatePublishGlobalMap(const ros::TimerEvent& event) {
  // only update the global map if the pose graph optimization is finished after loop closure

  if (global_map_->loop_closure_optimization_finished_) updateGlobalMap();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);
  getGlobalPointCloud(global_map_pcl);
  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(*global_map_pcl, pcl_msg);
  pcl_msg.header.frame_id = "world";
  pcl_msg.header.stamp = ros::Time::now();

  publisher.publishGlobalMap(pcl_msg);
}

void PoseGraphOptimization::getGlobalPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud) {
  for (auto point_landmark_map : global_map_->getMapPoints()) {
    Landmark point_landmark = point_landmark_map.second;
    Eigen::Vector3d global_pos = point_landmark.point_;
    Vector3d color = point_landmark.color_;
    double quality = point_landmark.quality_;

    if (quality > 0.005) {
      pcl::PointXYZRGB point;
      point.x = global_pos(0);
      point.y = global_pos(1);
      point.z = global_pos(2);
      float intensity = std::min(0.025, quality) / 0.025;
      point.r = static_cast<uint8_t>(color.x());
      point.g = static_cast<uint8_t>(color.y());
      point.b = static_cast<uint8_t>(color.z());
      pointcloud->push_back(point);
    }
  }
}

void PoseGraphOptimization::updateGlobalMap() {
  for (auto point_landmark_map : global_map_->getMapPoints()) {
    uint64_t landmark_id = point_landmark_map.first;
    Landmark point_landmark = point_landmark_map.second;

    Eigen::Vector3d point_3d = Eigen::Vector3d::Zero();
    Eigen::Vector3d color = Eigen::Vector3d::Zero();
    double quality = 0.0;
    uint64_t total_observations = 0;
    for (auto kf_observation : point_landmark.keyframe_observations_) {
      uint64_t kf_id = kf_observation.first;
      Observation obs = kf_observation.second;
      Eigen::Vector3d local_pos = obs.local_pos_;
      Eigen::Vector3d local_color = obs.color_;
      double kf_quality = obs.quality_;

      // Converting to global coordinates
      if (kfMapper_.find(kf_id) == kfMapper_.end()) {
        cout << "Keyframe not found" << endl;
        continue;
      }

      KFMatcher* kf = kfMapper_.find(kf_id)->second;
      Eigen::Matrix3d R_kf_w;
      Eigen::Vector3d T_kf_w;
      kf->getPose(T_kf_w, R_kf_w);

      Eigen::Vector3d global_pos = R_kf_w * local_pos + T_kf_w;
      point_3d = point_3d + global_pos * kf_quality;
      color = color + local_color * kf_quality;
      quality = quality + kf_quality;
      total_observations += 1;
    }

    point_3d = point_3d / quality;
    color = color / quality;
    quality = quality / total_observations;

    global_map_->updateLandmark(landmark_id, point_3d, quality, color);
  }
  global_map_->loop_closure_optimization_finished_ = false;
}

bool PoseGraphOptimization::savePointCloud(std_srvs::TriggerRequest& request, std_srvs::TriggerResponse& response) {
  ROS_INFO_STREAM("!! Saving Point Cloud !!");
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  getGlobalPointCloud(pointcloud);

  std::string pkg_path = ros::package::getPath("pose_graph");
  std::string pointcloud_file = pkg_path + "/reconstruction_results/pointcloud.ply";

  pcl::io::savePLYFileBinary(pointcloud_file, *pointcloud);
  response.success = true;
  response.message = "Saving Point Cloud ";
  return true;
}
