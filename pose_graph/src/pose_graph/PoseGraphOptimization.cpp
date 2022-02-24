#include "pose_graph/PoseGraphOptimization.h"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>

#include <Eigen/SVD>
#include <algorithm>
#include <boost/filesystem.hpp>
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

  consecutive_tracking_failures_ = 0;
  last_keyframe_time_ = 0;
  last_primitive_estmator_time_ = 0.0;
  tracking_status_ = TrackingStatus::NOT_INITIALIZED;

  init_t_w_prim_.setIdentity();
  init_t_w_svin_.setIdentity();

  switch_prim_pose_.setIdentity();
  switch_svin_pose_.setIdentity();
  switch_uber_pose_.setIdentity();

  last_t_w_prim_.setIdentity();
  last_scaled_prim_pose_.setZero();

  prim_estimator_keyframes_ = 0;
  vio_traj_length_ = 0.0;
  prim_traj_length_ = 0.0;
  scale_between_vio_prim_ = 0.0;
}

void PoseGraphOptimization::setup() {
  global_map_ = std::unique_ptr<GlobalMap>(new GlobalMap());

  loop_closing_ = std::unique_ptr<LoopClosing>(new LoopClosing());
  loop_closing_->setPublishers(nh_private_);
  loop_closing_->set_svin_results_file(params_->svin_w_loop_path_);
  loop_closing_->set_fast_relocalization(params_->fast_relocalization_);
  loop_closing_->registerLoopClosureOptimizationCallback(
      std::bind(&GlobalMap::loopClosureOptimizationFinishCallback, global_map_.get(), std::placeholders::_1));
  loop_closing_->startOptimizationThread();

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

  timer_ = nh_private_.createTimer(ros::Duration(3), &PoseGraphOptimization::updatePublishGlobalMap, this);

  if (params_->debug_image_) {
    setupOutputLogDirectories();
  }
}

void PoseGraphOptimization::run() {
  HealthParams health_params = params_->health_params_;
  std::string pkg_path = ros::package::getPath("pose_graph");
  std::string switching_info = pkg_path + "/output_logs/switch_info.txt";

  while (true) {
    std::ofstream switch_file(switching_info, std::ios::app);
    switch_file.setf(ios::fixed, ios::floatfield);
    switch_file.precision(9);

    sensor_msgs::ImageConstPtr image_msg = nullptr;
    sensor_msgs::PointCloudConstPtr point_msg = nullptr;
    nav_msgs::Odometry::ConstPtr pose_msg = nullptr;
    okvis_ros::SvinHealthConstPtr health_msg = nullptr;

    subscriber_->getSyncMeasurements(image_msg, pose_msg, point_msg, health_msg);

    static int last_keyframe_index = -1;
    if (pose_msg) {
      if (skip_cnt < SKIP_CNT) {
        skip_cnt++;
        continue;
      } else {
        skip_cnt = 0;
      }

      // build keyframe
      Eigen::Vector3d T = Eigen::Vector3d(
          pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
      Eigen::Matrix3d R = Eigen::Quaterniond(pose_msg->pose.pose.orientation.w,
                                             pose_msg->pose.pose.orientation.x,
                                             pose_msg->pose.pose.orientation.y,
                                             pose_msg->pose.pose.orientation.z)
                              .toRotationMatrix();

      // std::cout << "T: " << T.transpose() << std::endl;
      // std::cout << "R: " << R << std::endl;

      assert(point_msg);
      assert(image_msg);

      nav_msgs::OdometryConstPtr primitive_estimator_odom;
      geometry_msgs::PoseStamped uber_pose;
      nav_msgs::Odometry uber_odom;
      uber_pose.header.seq = uber_estimator_poses_.size() + 1;
      uber_pose.header.frame_id = "world";
      uber_pose.header.stamp = pose_msg->header.stamp;

      bool new_pose = false;

      Eigen::Matrix4d svin_cam_pose = Utility::rosPoseToMatrix(pose_msg->pose.pose);
      Eigen::Matrix4d svin_body_pose = svin_cam_pose * params_->T_imu_cam0_.inverse() * params_->T_body_imu_.inverse();
      Eigen::Matrix4d prim_estimator_pose, scaled_prim_estimator_pose;

      if (health_params.health_monitoring_enabled) {
        assert(health_msg);

        std::string vio_health_error_msg;
        bool vio_working = healthCheck(health_msg, vio_health_error_msg);

        if (!vio_working) {
          // ROS_WARN_STREAM(vio_health_error_msg);
          consecutive_tracking_failures_ += 1;
          consecutive_tracking_successes_ = 0;
        } else {
          consecutive_tracking_successes_ += 1;
          consecutive_tracking_failures_ = 0;
        }

        // ROS_WARN_STREAM("Consecutive Tracking successes: " << consecutive_tracking_successes_);

        primitive_estimator_odom = subscriber_->getPrimitiveEstimatorPose(pose_msg->header.stamp.toNSec());
        if (primitive_estimator_odom) {
          if (tracking_status_ == TrackingStatus::NOT_INITIALIZED) {
            init_t_w_prim_ = Utility::rosPoseToMatrix(primitive_estimator_odom->pose.pose);
            init_t_w_svin_ = svin_body_pose;
            switch_prim_pose_ = init_t_w_prim_;
            switch_svin_pose_ = init_t_w_svin_;
            switch_uber_pose_ = init_t_w_svin_;
            tracking_status_ = TrackingStatus::TRACKING_VIO;
          }

          prim_estimator_pose =
              init_t_w_svin_ * init_t_w_prim_.inverse() * Utility::rosPoseToMatrix(primitive_estimator_odom->pose.pose);

          scaled_prim_estimator_pose = prim_estimator_pose;

          if (scale_between_vio_prim_ != 0) {
            Eigen::Matrix4d relative_lkf_kf = last_t_w_prim_.inverse() * prim_estimator_pose;
            relative_lkf_kf.block<3, 1>(0, 3) = relative_lkf_kf.block<3, 1>(0, 3) * scale_between_vio_prim_;
            scaled_prim_estimator_pose = last_scaled_prim_pose_ * relative_lkf_kf;
          }

          // TODO(bjoshi): publish the scaled primitive odometry
          updatePrimiteEstimatorTrajectory(primitive_estimator_odom);
          nav_msgs::Odometry primtive_odometry;
          primtive_odometry.pose.pose = primitive_estimator_poses_.back().pose;
          primtive_odometry.header = primitive_estimator_poses_.back().header;
          publisher.publishOdometry(primtive_odometry, publisher.pub_prim_odometry_);
        }

        if (tracking_status_ == TrackingStatus::TRACKING_PRIMITIVE_ESTIMATOR) {
          if (consecutive_tracking_successes_ < health_params.consecutive_keyframes) {
            if (primitive_estimator_odom) {
              uber_pose.pose =
                  Utility::matrixToRosPose(switch_uber_pose_ * switch_prim_pose_.inverse() *
                                           scaled_prim_estimator_pose * params_->T_body_imu_ * params_->T_imu_cam0_);
              new_pose = true;
            }
          } else {
            switch_uber_pose_ = Utility::rosPoseToMatrix(uber_estimator_poses_.back().pose) *
                                params_->T_imu_cam0_.inverse() * params_->T_body_imu_.inverse();
            switch_svin_pose_ = svin_body_pose;
            switch_prim_pose_ = scaled_prim_estimator_pose;
            uber_pose.pose = Utility::matrixToRosPose(switch_uber_pose_ * switch_svin_pose_.inverse() * svin_cam_pose);
            tracking_status_ = TrackingStatus::TRACKING_VIO;
            new_pose = true;
            switch_file << 1 << " " << pose_msg->header.stamp.toSec() << " "
                        << primitive_estimator_odom->header.stamp.toSec() << " " << uber_pose.header.stamp << std::endl;

            ROS_INFO_STREAM("!!!!!!!! Switching to VIO !!!!!!!!!!");
          }
        } else {
          if (consecutive_tracking_failures_ >= (health_params.consecutive_keyframes + 3ul) &&
              !last_scaled_prim_pose_.isZero() && primitive_estimator_odom) {
            switch_uber_pose_ = Utility::rosPoseToMatrix(uber_estimator_poses_.back().pose) *
                                params_->T_imu_cam0_.inverse() * params_->T_body_imu_.inverse();
            switch_svin_pose_ = svin_body_pose;
            switch_prim_pose_ = last_scaled_prim_pose_;
            uber_pose.pose =
                Utility::matrixToRosPose(switch_uber_pose_ * switch_prim_pose_.inverse() * scaled_prim_estimator_pose *
                                         params_->T_body_imu_ * params_->T_imu_cam0_);
            tracking_status_ = TrackingStatus::TRACKING_PRIMITIVE_ESTIMATOR;
            ROS_INFO_STREAM(
                "Switching to Primitive Estimator. Consecutive Tracking failures: " << consecutive_tracking_failures_);

            switch_file << 0 << " " << pose_msg->header.stamp.toSec() << " "
                        << primitive_estimator_odom->header.stamp.toSec() << " " << uber_pose.header.stamp << std::endl;
          } else {
            uber_pose.pose = Utility::matrixToRosPose(switch_uber_pose_ * switch_svin_pose_.inverse() * svin_cam_pose);
          }
          new_pose = true;
        }
      } else {
        uber_pose.pose = pose_msg->pose.pose;
        new_pose = true;
      }

      if (new_pose) {
        uber_estimator_poses_.push_back(uber_pose);

        uber_pose.header.stamp = uber_estimator_poses_.back().header.stamp;
        uber_odom.pose.pose = uber_pose.pose;
        uber_odom.header = uber_pose.header;

        publisher.publishOdometry(uber_odom, publisher.pub_uber_odometry_);
        publisher.publishPath(uber_estimator_poses_, publisher.pub_uber_path_);
      }

      if ((T - last_translation_).norm() > SKIP_DIS && new_pose) {
        vector<cv::Point3f> point_3d;
        vector<cv::KeyPoint> point_2d_uv;
        vector<Eigen::Vector3d> point_ids;  // @Reloc: landmarkId, mfId, keypointIdx related to each point
        // For every KF, a map <observed_kf, weight> describing which other kfs how many MapPoints are common.
        map<KFMatcher*, int> KFcounter;

        int kf_index = -1;
        int combined_kf_index = -1;
        cv::Mat kf_image = subscriber_->readRosImage(image_msg);
        cv::Mat orig_color_image = subscriber_->getCorrespondingImage(pose_msg->header.stamp.toNSec());
        cv::Mat undistort_image;
        cv::remap(orig_color_image,
                  undistort_image,
                  params_->cam0_undistort_map_x_,
                  params_->cam0_undistort_map_y_,
                  cv::INTER_LINEAR);

        if (params_->resize_factor_ != 1.0) {
          uint16_t new_width = static_cast<uint16_t>(params_->image_width_ * params_->resize_factor_);
          uint16_t new_height = static_cast<uint16_t>(params_->image_height_ * params_->resize_factor_);
          cv::resize(undistort_image, undistort_image, cv::Size(new_width, new_height));
        }

        // TODO(bjoshi): publish debug image
        // cv::imshow("image", undistort_image);
        // cv::waitKey(1);

        // Need this because we want to update global map only if there is new loop
        // LoopClosing::addKFToPoseGraph updates keyframe to account for loop closure correction
        // Hence, need to add global points after the function.

        std::vector<uint64_t> landmark_ids;
        std::vector<double> qualities;
        std::vector<Eigen::Vector3d> colors;
        std::vector<Eigen::Vector3d> local_positions;

        Eigen::Matrix4d uber_pose_matrix = Utility::rosPoseToMatrix(uber_pose.pose);
        Eigen::Vector3d uber_position = uber_pose_matrix.block<3, 1>(0, 3);
        Eigen::Matrix3d uber_orientation = uber_pose_matrix.block<3, 3>(0, 0);

        for (unsigned int i = 0; i < point_msg->points.size(); i++) {
          double quality = point_msg->channels[i].values[3];

          cv::Point3f p_3d;
          p_3d.x = point_msg->points[i].x;
          p_3d.y = point_msg->points[i].y;
          p_3d.z = point_msg->points[i].z;

          Eigen::Vector3d point_3d_eigen;
          point_3d_eigen << p_3d.x, p_3d.y, p_3d.z;

          Eigen::Vector3d point_cam_frame = R.transpose() * (point_3d_eigen - T);
          Eigen::Vector3d point_w_f_uber = uber_orientation * point_cam_frame + uber_position;
          p_3d = cv::Point3f(point_w_f_uber(0), point_w_f_uber(1), point_w_f_uber(2));
          point_3d.push_back(p_3d);

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
          // double p_id;
          kf_index = point_msg->channels[i].values[4];
          combined_kf_index = kf_index + prim_estimator_keyframes_;
          p_2d_uv.pt.x = point_msg->channels[i].values[5];
          p_2d_uv.pt.y = point_msg->channels[i].values[6];
          p_2d_uv.size = point_msg->channels[i].values[7];
          p_2d_uv.angle = point_msg->channels[i].values[8];
          p_2d_uv.octave = point_msg->channels[i].values[9];
          p_2d_uv.response = point_msg->channels[i].values[10];
          p_2d_uv.class_id = point_msg->channels[i].values[11];

          point_2d_uv.push_back(p_2d_uv);

          // std::cout << "CV Keypoint of size 8:" << p_2d_uv.pt.x << " , " << p_2d_uv.pt.y << " size: " <<
          // p_2d_uv.size
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
            int observed_kf_index =
                point_msg->channels[i].values[sz] + prim_estimator_keyframes_;  // kf_index where this point_3d
                                                                                // has been observed
            if (observed_kf_index == combined_kf_index) {
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

        KFMatcher* keyframe;

        keyframe = new KFMatcher(pose_msg->header.stamp.toSec(),
                                 point_ids,
                                 combined_kf_index,
                                 uber_position,
                                 uber_orientation,
                                 kf_image,
                                 point_3d,
                                 point_2d_uv,
                                 KFcounter,
                                 sequence_,
                                 voc_,
                                 *params_,
                                 true);

        keyframe->setRelocalizationPCLCallback(
            std::bind(&Publisher::kfMatchedPointCloudCallback, &publisher, std::placeholders::_1));
        kfMapper_.insert(std::make_pair(combined_kf_index, keyframe));

        {
          std::lock_guard<std::mutex> l(processMutex_);
          // start_flag = 1;
          loop_closing_->addKFToPoseGraph(keyframe, params_->loop_closure_params_.loop_closure_enabled);
        }

        // sensor_msgs::PointCloud2 sparse_pointcloud_msg;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr sparse_pointcloud_(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (size_t i = 0; i < landmark_ids.size(); i++) {
          double quality = qualities.at(i);
          if (quality < params_->min_landmark_quality_) continue;

          Eigen::Vector3d pos_cam_frame = local_positions.at(i);

          if (kfMapper_.find(combined_kf_index) == kfMapper_.end()) {
            cout << "Keyframe not found" << endl;
            continue;
          }

          KFMatcher* kf = kfMapper_.find(combined_kf_index)->second;
          Eigen::Matrix3d R_w_kf;
          Eigen::Vector3d T_w_kf;
          kf->getPose(T_w_kf, R_w_kf);

          Eigen::Vector3d global_pos = R_w_kf * pos_cam_frame + T_w_kf;
          Eigen::Vector3d color = colors.at(i);
          uint64_t landmark_id = landmark_ids.at(i);

          global_map_->addLandmark(global_pos, landmark_id, quality, combined_kf_index, pos_cam_frame, color);

          // pcl::PointXYZRGB pcl_point;
          // pcl_point.x = global_pos(0);
          // pcl_point.y = global_pos(1);
          // pcl_point.z = global_pos(2);
          // pcl_point.r = (int)color(0);
          // pcl_point.g = (int)color(1);
          // pcl_point.b = (int)color(2);
          // sparse_pointcloud_->push_back(pcl_point);
        }

        if (tracking_status_ == TrackingStatus::TRACKING_VIO && primitive_estimator_odom &&
            !last_t_w_prim_.isIdentity()) {
          double time_since_last_kf = pose_msg->header.stamp.toSec() - static_cast<double>(last_keyframe_time_) * 1e-9;
          if (time_since_last_kf <= health_params.kf_wait_time) {
            double time_since_last_prim =
                primitive_estimator_odom->header.stamp.toSec() - last_primitive_estmator_time_;
            // ROS_INFO_STREAM("Time since last keyframe: " << time_since_last_kf);
            // ROS_INFO_STREAM("Time since last primitive: " << time_since_last_prim);

            vio_traj_length_ +=
                (last_t_w_svin_.inverse() * svin_body_pose).block<3, 1>(0, 3).norm() / time_since_last_kf;
            prim_traj_length_ +=
                (last_t_w_prim_.inverse() * prim_estimator_pose).block<3, 1>(0, 3).norm() / time_since_last_prim;
            scale_between_vio_prim_ = vio_traj_length_ / prim_traj_length_;
            // ROS_INFO_STREAM("Scaling factor between VIO and primitive estimator: " << scale_between_vio_prim_);
          }
        }

        frame_index_++;
        last_translation_ = T;
        last_keyframe_time_ = pose_msg->header.stamp.toNSec();
        if (kf_index > 0) last_keyframe_index = kf_index;

        last_t_w_svin_ = svin_body_pose;
        if (primitive_estimator_odom) {
          last_t_w_prim_ = prim_estimator_pose;
          last_scaled_prim_pose_ = scaled_prim_estimator_pose;
          last_primitive_estmator_time_ = primitive_estimator_odom->header.stamp.toSec();
        }
        // pcl::toROSMsg(*sparse_pointcloud_, sparse_pointcloud_msg);
        // sparse_pointcloud_msg.header.frame_id = "world";
        // sparse_pointcloud_msg.header.stamp = pose_msg->header.stamp;
        // pubSparseMap.publish(sparse_pointcloud_msg);
      }
    } else if (health_params.health_monitoring_enabled) {
      // SVIN Frontend does not pass keyframe because of not tracking
      // Check if it is just not waiting for keyframe

      if (tracking_status_ != TrackingStatus::NOT_INITIALIZED &&
          abs(subscriber_->getLatestPrimitiveEstimatorTime() - static_cast<double>(last_keyframe_time_) * 1e-9) >
              health_params.kf_wait_time) {
        std::vector<nav_msgs::OdometryConstPtr> prim_estimator_poses;
        subscriber_->getPrimitiveEstimatorPoses(last_keyframe_time_, prim_estimator_poses);

        if (!prim_estimator_poses.empty()) {
          if (tracking_status_ == TrackingStatus::TRACKING_VIO) {
            switch_svin_pose_ = last_t_w_svin_;
            switch_prim_pose_ = last_scaled_prim_pose_;
            switch_uber_pose_ = Utility::rosPoseToMatrix(uber_estimator_poses_.back().pose) *
                                params_->T_imu_cam0_.inverse() * params_->T_body_imu_.inverse();
            // cout << std::setprecision(12) << primitive_estimator_odom->header.stamp.toSec() << " " <<
            // last_keyframe_time_
            //      << endl;
            // cout << switch_svin_pose_ << endl;
            // cout << switch_prim_pose_ << endl;
            // cout << switch_uber_pose_ << endl;
            tracking_status_ = TrackingStatus::TRACKING_PRIMITIVE_ESTIMATOR;
            ROS_INFO_STREAM("!!!!!!!! Switching to Primitive Estimator !!!!!!!!!!");
            switch_file << 0 << " " << static_cast<double>(last_keyframe_time_) * 1e-9 << " "
                        << prim_estimator_poses.front()->header.stamp.toSec() << " "
                        << prim_estimator_poses.front()->header.stamp.toSec() << std::endl;
          }

          for (nav_msgs::OdometryConstPtr primitive_estimator_odom : prim_estimator_poses) {
            Eigen::Matrix4d prim_estimator_pose = init_t_w_svin_ * init_t_w_prim_.inverse() *
                                                  Utility::rosPoseToMatrix(primitive_estimator_odom->pose.pose);
            Eigen::Matrix4d scaled_prim_estimator_pose = prim_estimator_pose;

            if (scale_between_vio_prim_ != 0) {
              Eigen::Matrix4d relative_lkf_kf = last_t_w_prim_.inverse() * prim_estimator_pose;
              relative_lkf_kf.block<3, 1>(0, 3) = relative_lkf_kf.block<3, 1>(0, 3) * scale_between_vio_prim_;
              scaled_prim_estimator_pose = last_scaled_prim_pose_ * relative_lkf_kf;
            }
            geometry_msgs::PoseStamped uber_pose;
            uber_pose.header.seq = uber_estimator_poses_.size() + 1;
            uber_pose.header.frame_id = "world";
            uber_pose.header.stamp = primitive_estimator_odom->header.stamp;
            uber_pose.pose =
                Utility::matrixToRosPose(switch_uber_pose_ * switch_prim_pose_.inverse() * scaled_prim_estimator_pose *
                                         params_->T_body_imu_ * params_->T_imu_cam0_);
            uber_estimator_poses_.push_back(uber_pose);
            nav_msgs::Odometry uber_odom;
            uber_odom.pose.pose = uber_pose.pose;
            uber_odom.header = uber_pose.header;

            updatePrimiteEstimatorTrajectory(primitive_estimator_odom);
            nav_msgs::Odometry primtive_odometry;
            primtive_odometry.pose.pose = primitive_estimator_poses_.back().pose;
            primtive_odometry.header = primitive_estimator_poses_.back().header;
            publisher.publishOdometry(primtive_odometry, publisher.pub_prim_odometry_);

            publisher.publishOdometry(uber_odom, publisher.pub_uber_odometry_);
            publisher.publishPath(uber_estimator_poses_, publisher.pub_uber_path_);

            prim_estimator_keyframes_++;
            Eigen::Matrix4d updated_transform = Utility::rosPoseToMatrix(uber_pose.pose);
            Eigen::Vector3d T = updated_transform.block<3, 1>(0, 3);
            Eigen::Matrix3d R = updated_transform.block<3, 3>(0, 0);
            int kf_index = last_keyframe_index + prim_estimator_keyframes_;
            map<KFMatcher*, int> kf_counter;
            KFMatcher* keyframe =
                new KFMatcher(uber_pose.header.stamp.toSec(), kf_index, T, R, kf_counter, sequence_, *params_, false);
            keyframe->setRelocalizationPCLCallback(
                std::bind(&Publisher::kfMatchedPointCloudCallback, &publisher, std::placeholders::_1));
            kfMapper_.insert(std::make_pair(kf_index, keyframe));

            {
              std::lock_guard<std::mutex> l(processMutex_);
              // start_flag = 1;
              loop_closing_->addKFToPoseGraph(keyframe, false);
            }
            last_t_w_prim_ = prim_estimator_pose;
            last_scaled_prim_pose_ = scaled_prim_estimator_pose;
          }
        }
        consecutive_tracking_successes_ = 0;
      }
    }

    switch_file.close();
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
    Eigen::Vector3d color = point_landmark.color_;
    double quality = point_landmark.quality_;

    if (quality > params_->min_landmark_quality_) {
      pcl::PointXYZRGB point;
      point.x = global_pos(0);
      point.y = global_pos(1);
      point.z = global_pos(2);
      // float intensity = std::min(0.025, quality) / 0.025;
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

      // if (kf_quality > quality) {
      //   point_3d = R_kf_w * local_pos + T_kf_w;
      //   color = local_color;
      //   quality = kf_quality;
      // }

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

void PoseGraphOptimization::updatePrimiteEstimatorTrajectory(const nav_msgs::OdometryConstPtr& pose_msg) {
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = pose_msg->header;
  pose_stamped.header.seq = primitive_estimator_poses_.size() + 1;
  pose_stamped.pose = Utility::matrixToRosPose(init_t_w_svin_ * init_t_w_prim_.inverse() *
                                               Utility::rosPoseToMatrix(pose_msg->pose.pose) * params_->T_body_imu_ *
                                               params_->T_imu_cam0_);
  primitive_estimator_poses_.push_back(pose_stamped);
}

bool PoseGraphOptimization::healthCheck(const okvis_ros::SvinHealthConstPtr& health_msg, std::string& error_msg) {
  // ROS_INFO_STREAM(Utility::healthMsgToString(health_msg));
  std::stringstream ss;
  std::setprecision(5);

  HealthParams health_params = params_->health_params_;
  uint32_t total_triangulated_keypoints = health_msg->numTrackedKps;

  if (total_triangulated_keypoints < health_params.min_tracked_keypoints) {
    ss << "Not enough triangulated keypoints: " << total_triangulated_keypoints << std::endl;
    error_msg = ss.str();
    return false;
  }

  std::vector<int> keypoints_per_quadrant = health_msg->kpsPerQuadrant;

  bool quadrant_check = std::all_of(keypoints_per_quadrant.begin(), keypoints_per_quadrant.end(), [&](int kp_count) {
    return kp_count >= health_params.kps_per_quadrant;
  });

  if (!quadrant_check && *std::max_element(keypoints_per_quadrant.begin(), keypoints_per_quadrant.end()) <=
                             10.0 * health_params.kps_per_quadrant) {
    ss << "Not enough keypoints per quadrant:  [" << keypoints_per_quadrant[0] << ", " << keypoints_per_quadrant[1]
       << ", " << keypoints_per_quadrant[2] << ", " << keypoints_per_quadrant[3] << "]" << std::endl;
    error_msg = ss.str();
    return false;
  }

  uint32_t new_detected_keypoints_kf = health_msg->newKps;
  float new_detected_keypoints_ratio =
      static_cast<float>(new_detected_keypoints_kf) / static_cast<float>(total_triangulated_keypoints);

  if (new_detected_keypoints_ratio >= 0.75) {
    ss << "Too many new keypoints: " << new_detected_keypoints_ratio << std::endl;
    error_msg = ss.str();
    return false;
  }

  double average_response =
      std::accumulate(health_msg->responseStrengths.begin(), health_msg->responseStrengths.end(), double(0.0)) /
      static_cast<double>(health_msg->responseStrengths.size());
  float fraction_with_low_detector_response =
      std::count_if(health_msg->responseStrengths.begin(),
                    health_msg->responseStrengths.end(),
                    [&](double response) { return response < average_response; }) /
      static_cast<float>(health_msg->responseStrengths.size());

  if (fraction_with_low_detector_response >= 0.85) {
    ss << "Too many detectors with low response: " << fraction_with_low_detector_response << std::endl;
    error_msg = ss.str();
    return false;
  }

  // double average_quality = std::accumulate(health_msg->quality.begin(), health_msg->quality.end(), double(0.0)) /
  //                          static_cast<double>(health_msg->quality.size());
  // float fraction_with_low_quality = std::count_if(health_msg->quality.begin(),
  //                                                 health_msg->quality.end(),
  //                                                 [&](double quality) { return quality < average_quality; }) /
  //                                   static_cast<float>(health_msg->quality.size());
  // if (fraction_with_low_quality > 0.90) {
  //   ss << "Too many landmarks with low quality: " << fraction_with_low_quality << std::endl;
  //   error_msg = ss.str();
  //   return false;
  // }

  return true;
}

void PoseGraphOptimization::setupOutputLogDirectories() {
  std::string pacakge_path = ros::package::getPath("pose_graph");

  std::string output_dir = pacakge_path + "/output_logs/loop_candidates/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directory(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = pacakge_path + "/output_logs/descriptor_matched/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directory(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = pacakge_path + "/output_logs/pnp_verified/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directory(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = pacakge_path + "/output_logs/loop_closure/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directory(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  output_dir = pacakge_path + "/output_logs/geometric_verification/";
  if (!boost::filesystem::is_directory(output_dir) || !boost::filesystem::exists(output_dir)) {
    boost::filesystem::create_directory(output_dir);
  }
  for (const auto& entry : boost::filesystem::directory_iterator(output_dir)) {
    boost::filesystem::remove_all(entry.path());
  }

  std::string loop_closure_file = pacakge_path + "/output_logs/loop_closure.txt";
  if (boost::filesystem::exists(loop_closure_file)) {
    boost::filesystem::remove(loop_closure_file);
  }
  std::ofstream loop_path_file(loop_closure_file, ios::out);
  loop_path_file << "cur_kf_id"
                 << " "
                 << "cur_kf_ts"
                 << " "
                 << "matched_kf_id"
                 << " "
                 << "matched_kf_ts"
                 << " "
                 << "relative_tx"
                 << " "
                 << "relative_ty"
                 << " "
                 << "relative_tz"
                 << " "
                 << "relative_qx"
                 << " "
                 << "relative_qy"
                 << " "
                 << "relative_qz"
                 << " "
                 << "relative_qw" << endl;
  loop_path_file.close();

  std::string switch_info_file = pacakge_path + "/output_logs/switch_info.txt";
  if (boost::filesystem::exists(switch_info_file)) {
    boost::filesystem::remove(switch_info_file);
  }
  std::ofstream switch_info_file_stream(switch_info_file, ios::out);
  switch_info_file_stream << "type"
                          << " "
                          << "vio_stamp"
                          << " "
                          << "prim_stamp"
                          << " "
                          << "uber_stamp" << endl;
  switch_info_file_stream.close();
}
