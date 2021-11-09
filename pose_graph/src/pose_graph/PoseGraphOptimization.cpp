#include "pose_graph/PoseGraphOptimization.h"

#include <map>
#include <memory>
#include <utility>
#include <vector>

PoseGraphOptimization::PoseGraphOptimization()
    : nh_private_("~"),
      params_(nullptr),
      loop_closing_(nullptr),
      camera_pose_visualizer_(nullptr),
      subscriber_(nullptr) {
  frame_index_ = 0;
  sequence_ = 1;

  last_translation_ = Eigen::Vector3d(-100, -100, -100);
  SKIP_CNT = 0;
  skip_cnt = 0;

  SKIP_DIS = 0;

  params_ = std::make_shared<Parameters>();
  params_->loadParameters(nh_private_);

  setup();
}

void PoseGraphOptimization::setup() {
  loop_closing_ = std::make_unique<LoopClosing>();
  loop_closing_->setPublishers(nh_private_);
  loop_closing_->set_svin_results_file(params_->svin_w_loop_path_);
  loop_closing_->set_fast_relocalization(params_->fast_relocalization_);

  camera_pose_visualizer_ = std::make_unique<CameraPoseVisualization>(1, 0, 0, 1);
  camera_pose_visualizer_->setScale(params_->camera_visual_size_);
  camera_pose_visualizer_->setLineWidth(params_->camera_visual_size_ / 10.0);

  // Loading vocabulary
  voc_ = new BriefVocabulary(params_->vocabulary_file_);
  BriefDatabase db;
  db.setVocabulary(*voc_, false, 0);
  loop_closing_->setBriefVocAndDB(voc_, db);

  subscriber_ = std::make_unique<Subscriber>(nh_private_, *params_);
  publisher.setParameters(*params_);
  publisher.setPublishers();

  // std::thread loop_closing(&PoseGraphOptimization::run, this);
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
        cv::Mat image = subscriber_->readRosImage(image_msg);

        for (unsigned int i = 0; i < point_msg->points.size(); i++) {
          double quality = point_msg->channels[i].values[3];
          if (quality < 1e-6) continue;

          cv::Point3f p_3d;
          p_3d.x = point_msg->points[i].x;
          p_3d.y = point_msg->points[i].y;
          p_3d.z = point_msg->points[i].z;
          point_3d.push_back(p_3d);

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
                                            image,
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

        frame_index_++;
        last_translation_ = T;
      }
    }

    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
  }
}
