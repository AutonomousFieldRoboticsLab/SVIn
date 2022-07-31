#include "pose_graph/LoopClosing.h"

#include <list>
#include <map>
#include <set>
#include <string>

#include "pose_graph/Pose3DError.h"

LoopClosing::LoopClosing() {
  posegraph_visualization = new CameraPoseVisualization(1.0, 0.0, 1.0, 1.0);
  posegraph_visualization->setScale(0.1);
  posegraph_visualization->setLineWidth(0.01);
  earliest_loop_index = -1;
  t_drift = Eigen::Vector3d(0, 0, 0);
  yaw_drift = 0;
  r_drift = Eigen::Matrix3d::Identity();
  w_t_svin = Eigen::Vector3d(0, 0, 0);
  w_r_svin = Eigen::Matrix3d::Identity();
  global_index = 0;
  sequence_cnt = 0;
  sequence_loop.push_back(0);
  base_sequence = 1;
}

LoopClosing::~LoopClosing() { t_optimization.join(); }

void LoopClosing::set_svin_results_file(const std::string& svin_output_file) { svin_output_file_ = svin_output_file; }
void LoopClosing::set_fast_relocalization(const bool fast_relocalization) {
  is_fast_localization_ = fast_relocalization;
}

void LoopClosing::setPublishers(ros::NodeHandle& nh) {
  pubPoseGraphPath = nh.advertise<nav_msgs::Path>("pose_graph_path", 1000);
  pubBasePath = nh.advertise<nav_msgs::Path>("base_path", 1000);
  pubPoseGraph = nh.advertise<visualization_msgs::MarkerArray>("pose_graph", 1000);
  for (int i = 1; i < 10; i++) pubPath[i] = nh.advertise<nav_msgs::Path>("path_" + std::to_string(i), 1000);
}

void LoopClosing::setBriefVocAndDB(BriefVocabulary* vocabulary, BriefDatabase database) {
  voc = vocabulary;
  db = database;
}

void LoopClosing::startOptimizationThread(bool vio_only_optimization) {
  if (vio_only_optimization) {
    t_optimization = std::thread(&LoopClosing::optimize4DoFPoseGraph, this);
  } else {
    t_optimization = std::thread(&LoopClosing::optimize6DoFPoseGraph, this);
  }
}

void LoopClosing::addKFToPoseGraph(KFMatcher* cur_kf, bool flag_detect_loop) {
  // shift to base frame
  Eigen::Vector3d svin_P_cur;
  Eigen::Matrix3d svin_R_cur;
  if (sequence_cnt != cur_kf->sequence) {
    sequence_cnt++;
    sequence_loop.push_back(0);
    w_t_svin = Eigen::Vector3d(0, 0, 0);
    w_r_svin = Eigen::Matrix3d::Identity();

    {
      std::lock_guard<std::mutex> l(driftMutex_);
      t_drift = Eigen::Vector3d(0, 0, 0);
      r_drift = Eigen::Matrix3d::Identity();
    }
  }

  cur_kf->getSVInPose(svin_P_cur, svin_R_cur);
  svin_P_cur = w_r_svin * svin_P_cur + w_t_svin;
  svin_R_cur = w_r_svin * svin_R_cur;
  cur_kf->updateSVInPose(svin_P_cur, svin_R_cur);
  cur_kf->index = global_index;
  global_index++;
  std::set<KFMatcher*> loopCandidates;
  int loop_index = -1;

  if (flag_detect_loop) {  // at least 20 KF has been passed
    TicToc tmp_t;
    loop_index = detectLoop(cur_kf, cur_kf->index);
  } else {
    addKeyFrameIntoVoc(cur_kf);
  }

  if (loop_index != -1) {
    KFMatcher* old_kf = getKFPtr(loop_index);
    if (cur_kf->findConnection(old_kf)) {
      std::cout << "FOUND Loop Connection!!!!" << std::endl;

      if (earliest_loop_index > loop_index || earliest_loop_index == -1) earliest_loop_index = loop_index;

      Eigen::Vector3d w_P_old, w_P_cur, svin_P_cur;
      Eigen::Matrix3d w_R_old, w_R_cur, svin_R_cur;
      old_kf->getSVInPose(w_P_old, w_R_old);  // old_kf replaced by min_loop_kf
      cur_kf->getSVInPose(svin_P_cur, svin_R_cur);

      Eigen::Vector3d relative_t;
      Eigen::Quaterniond relative_q;
      relative_t = cur_kf->getLoopRelativeT();
      relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
      w_P_cur = w_R_old * relative_t + w_P_old;
      w_R_cur = w_R_old * relative_q;
      double shift_yaw;
      Eigen::Matrix3d shift_r;
      Eigen::Vector3d shift_t;
      shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(svin_R_cur).x();
      shift_r = Utility::ypr2R(Eigen::Vector3d(shift_yaw, 0, 0));
      shift_t = w_P_cur - w_R_cur * svin_R_cur.transpose() * svin_P_cur;
      // shift svin pose of whole sequence to the world frame
      if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0) {
        w_r_svin = shift_r;
        w_t_svin = shift_t;
        svin_P_cur = w_r_svin * svin_P_cur + w_t_svin;
        svin_R_cur = w_r_svin * svin_R_cur;
        cur_kf->updateSVInPose(svin_P_cur, svin_R_cur);
        std::list<KFMatcher*>::iterator it = keyframelist.begin();
        for (; it != keyframelist.end(); it++) {
          if ((*it)->sequence == cur_kf->sequence) {
            Eigen::Vector3d svin_P_cur;
            Eigen::Matrix3d svin_R_cur;
            (*it)->getSVInPose(svin_P_cur, svin_R_cur);
            svin_P_cur = w_r_svin * svin_P_cur + w_t_svin;
            svin_R_cur = w_r_svin * svin_R_cur;
            (*it)->updateSVInPose(svin_P_cur, svin_R_cur);
          }
        }
        sequence_loop[cur_kf->sequence] = 1;
      }
      std::lock_guard<std::mutex> l(optimizationMutex_);
      optimizationBuffer_.push(cur_kf->index);
    }
  }

  {
    std::lock_guard<std::mutex> l(kflistMutex_);
    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    cur_kf->getSVInPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Eigen::Quaterniond Q{R};
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = cur_kf->time_stamp;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x();
    pose_stamped.pose.position.y = P.y();
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    if (SAVE_LOOP_PATH) {
      std::ofstream loop_path_file(svin_output_file_, std::ios::app);
      loop_path_file.setf(std::ios::fixed, std::ios::floatfield);
      loop_path_file.precision(9);
      loop_path_file << cur_kf->time_stamp << " ";
      loop_path_file << P.x() << " " << P.y() << " " << P.z() << " " << Q.x() << " " << Q.y() << " " << Q.z() << " "
                     << Q.w() << std::endl;
      loop_path_file.close();
    }
    // draw local connection
    if (SHOW_S_EDGE) {
      std::list<KFMatcher*>::reverse_iterator rit = keyframelist.rbegin();
      for (int i = 0; i < 4; i++) {
        if (rit == keyframelist.rend()) break;
        Eigen::Vector3d conncected_P;
        Eigen::Matrix3d connected_R;
        if ((*rit)->sequence == cur_kf->sequence) {
          (*rit)->getPose(conncected_P, connected_R);
          posegraph_visualization->add_edge(P, conncected_P);
        }
        rit++;
      }
    }
    if (SHOW_L_EDGE) {
      if (cur_kf->has_loop) {
        // printf("has loop \n");
        KFMatcher* connected_KF = getKFPtr(cur_kf->loop_index);
        Eigen::Vector3d connected_P, P0;
        Eigen::Matrix3d connected_R, R0;
        connected_KF->getPose(connected_P, connected_R);
        // cur_kf->getSVInPose(P0, R0);
        cur_kf->getPose(P0, R0);
        if (cur_kf->sequence > 0) {
          // printf("add loop into visual \n");
          // std::cout<< "Drawing loop edge for " << cur_kf->index << " and "<< cur_kf->loop_index <<std::endl;
          posegraph_visualization->add_loopedge(P0, connected_P);
        }
      }
    }

    keyframelist.push_back(cur_kf);
    publish();
  }
}

KFMatcher* LoopClosing::getKFPtr(int index) {
  std::list<KFMatcher*>::iterator it = keyframelist.begin();
  for (; it != keyframelist.end(); it++) {
    if ((*it)->index == index) break;
  }
  if (it != keyframelist.end())
    return *it;
  else
    return NULL;
}

int LoopClosing::detectLoop(KFMatcher* keyframe, int frame_index) {
  cv::Mat compressed_image;

  if (keyframe->bowVec.empty() || keyframe->featVec.empty()) {
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    voc->transform(keyframe->brief_descriptors, keyframe->bowVec);
  }

  float min_score = 1.0;
  for (std::map<KFMatcher*, int>::iterator mit = keyframe->mConnectedKeyFrameWeights.begin();
       mit != keyframe->mConnectedKeyFrameWeights.end();
       mit++) {
    // BowVector neigh_vec;

    if (mit->first->bowVec.empty() || mit->first->featVec.empty())
      voc->transform(mit->first->brief_descriptors, mit->first->bowVec);

    float score = voc->score(keyframe->bowVec, mit->first->bowVec);
    // std::cout << "Score in neigh frames: " << score << " with Id: " << mit->first->index << std::endl;
    if (score < min_score) min_score = score;
  }

  // std::cout<< "Min BoW Score: "<< min_score << std::endl;

  TicToc tmp_t;
  // first query; then add this frame into database!
  DBoW2::QueryResults ret;
  TicToc t_query;
  db.query(keyframe->bowVec, ret, 4, frame_index - 50);
  // printf("query time: %f", t_query.toc());

  TicToc t_add;
  db.add(keyframe->brief_descriptors);

  bool find_loop = false;
  cv::Mat loop_result;

  for (unsigned int i = 0; i < ret.size(); i++) {
    if (ret[i].Score > 0.60 * min_score) {
      find_loop = true;
      // std::cout<< "Query KF: "<< frame_index<< " candidate kf: "<< ret[i].Id << std::endl;
      int tmp_index = ret[i].Id;
    }
  }

  // Note:  Returns depending on the highest score
  if (find_loop && frame_index > 50) {
    int best_index = -1;
    float best_score = 0.0;
    for (unsigned int i = 0; i < ret.size(); i++) {
      if (best_index == -1 || (ret[i].Score > best_score && ret[i].Score > 0.60 * min_score)) {
        best_index = ret[i].Id;
        best_score = ret[i].Score;
      }
    }
    return best_index;
  } else {
    return -1;
  }
}

void LoopClosing::addKeyFrameIntoVoc(KFMatcher* keyframe) {
  cv::Mat compressed_image;
  db.add(keyframe->brief_descriptors);
}

void LoopClosing::optimize4DoFPoseGraph() {
  while (true) {
    int cur_index = -1;
    int first_looped_index = -1;

    {
      std::lock_guard<std::mutex> l(optimizationMutex_);
      while (!optimizationBuffer_.empty()) {
        cur_index = optimizationBuffer_.front();
        first_looped_index = earliest_loop_index;
        optimizationBuffer_.pop();
      }
    }
    if (cur_index != -1) {
      TicToc tmp_t;
      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction* loss_function;
      loss_function = new ceres::HuberLoss(0.1);

      kflistMutex_.lock();
      KFMatcher* cur_kf = getKFPtr(cur_index);

      int max_length = cur_index + 1;

      double t_array[max_length][3];
      Eigen::Quaterniond q_array[max_length];  // NOLINT
      double euler_array[max_length][3];       // NOLINT
      double sequence_array[max_length];       // NOLINT

      ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

      std::list<KFMatcher*>::iterator it;

      int i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
        if ((*it)->index < first_looped_index) continue;
        (*it)->local_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getSVInPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i][0] = tmp_t(0);
        t_array[i][1] = tmp_t(1);
        t_array[i][2] = tmp_t(2);
        q_array[i] = tmp_q;

        Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
        euler_array[i][0] = euler_angle.x();
        euler_array[i][1] = euler_angle.y();
        euler_array[i][2] = euler_angle.z();

        sequence_array[i] = (*it)->sequence;

        problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
        problem.AddParameterBlock(t_array[i], 3);

        if ((*it)->index == first_looped_index || (*it)->sequence == 0) {
          problem.SetParameterBlockConstant(euler_array[i]);
          problem.SetParameterBlockConstant(t_array[i]);
        }

        // add edge
        // adding sequential egde. Fixed sized window of length 4 serves as covisibility
        for (int j = 1; j < 5; j++) {
          if (i - j >= 0 && sequence_array[i] == sequence_array[i - j]) {
            Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i - j].toRotationMatrix());
            Eigen::Vector3d relative_t(t_array[i][0] - t_array[i - j][0],
                                       t_array[i][1] - t_array[i - j][1],
                                       t_array[i][2] - t_array[i - j][2]);
            relative_t = q_array[i - j].inverse() * relative_t;
            double relative_yaw = euler_array[i][0] - euler_array[i - j][0];
            ceres::CostFunction* cost_function = FourDOFError::Create(relative_t.x(),
                                                                      relative_t.y(),
                                                                      relative_t.z(),
                                                                      relative_yaw,
                                                                      euler_conncected.y(),
                                                                      euler_conncected.z());
            problem.AddResidualBlock(
                cost_function, NULL, euler_array[i - j], t_array[i - j], euler_array[i], t_array[i]);
          }
        }

        // add loop edge

        if ((*it)->has_loop) {
          assert((*it)->loop_index >= first_looped_index);
          int connected_index = getKFPtr((*it)->loop_index)->local_index;
          Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
          Eigen::Vector3d relative_t;
          relative_t = (*it)->getLoopRelativeT();
          double relative_yaw = (*it)->getLoopRelativeYaw();
          ceres::CostFunction* cost_function = FourDOFWeightError::Create(
              relative_t.x(), relative_t.y(), relative_t.z(), relative_yaw, euler_conncected.y(), euler_conncected.z());
          problem.AddResidualBlock(cost_function,
                                   loss_function,
                                   euler_array[connected_index],
                                   t_array[connected_index],
                                   euler_array[i],
                                   t_array[i]);
        }

        if ((*it)->index == cur_index) break;
        i++;
      }
      kflistMutex_.unlock();

      ceres::Solve(options, &problem, &summary);

      {
        std::lock_guard<std::mutex> l(kflistMutex_);
        i = 0;
        for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
          if ((*it)->index < first_looped_index) continue;
          Eigen::Quaterniond tmp_q;
          tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
          Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
          Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
          (*it)->updatePose(tmp_t, tmp_r);

          if ((*it)->index == cur_index) break;
          i++;
        }

        Eigen::Vector3d cur_t, svin_t;
        Eigen::Matrix3d cur_r, svin_r;
        cur_kf->getPose(cur_t, cur_r);
        cur_kf->getSVInPose(svin_t, svin_r);
        {
          std::lock_guard<std::mutex> l(driftMutex_);
          yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(svin_r).x();
          r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
          t_drift = cur_t - r_drift * svin_t;
        }

        it++;
        for (; it != keyframelist.end(); it++) {
          Eigen::Vector3d P;
          Eigen::Matrix3d R;
          (*it)->getSVInPose(P, R);
          P = r_drift * P + t_drift;
          R = r_drift * R;
          (*it)->updatePose(P, R);
        }
      }
      updatePath();

      loop_closure_optimization_callback_(ros::Time::now().toNSec());
    }

    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
}

void LoopClosing::optimize6DoFPoseGraph() {
  while (true) {
    int cur_index = -1;
    int first_looped_index = -1;

    {
      std::lock_guard<std::mutex> l(optimizationMutex_);
      while (!optimizationBuffer_.empty()) {
        cur_index = optimizationBuffer_.front();
        first_looped_index = earliest_loop_index;
        optimizationBuffer_.pop();
      }
    }
    if (cur_index != -1) {
      // clang-format off
      Eigen::Matrix<double, 6, 6> relative_pose_sqrt_information, loop_closure_sqrt_information;
      relative_pose_sqrt_information << 20.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 20.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 20.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 57.3;

      loop_closure_sqrt_information << 20.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 20.0, 0.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 20.0, 0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 100.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0, 0.0, 100.0, 0.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 100.0;
      // clang-format on

      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction* loss_function = new ceres::HuberLoss(0.1);

      kflistMutex_.lock();
      KFMatcher* cur_kf = getKFPtr(cur_index);

      int kMaxLength = cur_index + 1;

      Eigen::Vector3d t_array[kMaxLength];
      Eigen::Quaterniond q_array[kMaxLength];  // NOLINT
      double sequence_array[kMaxLength];       // NOLINT

      ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

      std::list<KFMatcher*>::iterator it;

      int i = 0;
      for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
        if ((*it)->index < first_looped_index) continue;
        (*it)->local_index = i;
        Eigen::Quaterniond tmp_q;
        Eigen::Matrix3d tmp_r;
        Eigen::Vector3d tmp_t;
        (*it)->getSVInPose(tmp_t, tmp_r);
        tmp_q = tmp_r;
        t_array[i] = tmp_t;
        q_array[i] = tmp_q;
        sequence_array[i] = (*it)->sequence;

        problem.AddParameterBlock(q_array[i].coeffs().data(), 4, quaternion_local_parameterization);
        problem.AddParameterBlock(t_array[i].data(), 3);

        if ((*it)->index == first_looped_index || (*it)->sequence == 0) {
          problem.SetParameterBlockConstant(t_array[i].data());
          problem.SetParameterBlockConstant(q_array[i].coeffs().data());
        }

        // add edge
        // adding sequential egde. Fixed sized window of length 4 serves as covisibility
        for (int j = 1; j < 5; j++) {
          if (i - j >= 0 && sequence_array[i] == sequence_array[i - j]) {
            Eigen::Quaterniond relative_q = q_array[i - j].inverse() * q_array[i];
            Eigen::Vector3d relative_t = q_array[i - j].inverse() * (t_array[i] - t_array[i - j]);
            ceres::Pose3d relative_pose(relative_t, relative_q);
            ceres::CostFunction* cost_function =
                ceres::PoseGraph3dErrorTerm::Create(relative_pose, relative_pose_sqrt_information);

            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     t_array[i - j].data(),
                                     q_array[i - j].coeffs().data(),
                                     t_array[i].data(),
                                     q_array[i].coeffs().data());

            // problem.SetParameterization(q_array[i - j].coeffs().data(), quaternion_local_parameterization);
            // problem.SetParameterization(q_array[i].coeffs().data(), quaternion_local_parameterization);
          }
        }

        // add loop edge

        if ((*it)->has_loop) {
          assert((*it)->loop_index >= first_looped_index);
          Eigen::Vector3d relative_t = (*it)->getLoopRelativeT();
          Eigen::Quaterniond relative_q = (*it)->getLoopRelativeQ();
          ceres::Pose3d relative_pose(relative_t, relative_q);
          ceres::CostFunction* cost_function =
              ceres::PoseGraph3dErrorTerm::Create(relative_pose, loop_closure_sqrt_information);

          int connected_index = getKFPtr((*it)->loop_index)->local_index;
          problem.AddResidualBlock(cost_function,
                                   loss_function,
                                   t_array[connected_index].data(),
                                   q_array[connected_index].coeffs().data(),
                                   t_array[i].data(),
                                   q_array[i].coeffs().data());

          // problem.SetParameterization(q_array[connected_index].coeffs().data(), quaternion_local_parameterization);
          // problem.SetParameterization(q_array[i].coeffs().data(), quaternion_local_parameterization);
        }

        if ((*it)->index == cur_index) break;
        i++;
      }
      kflistMutex_.unlock();

      ceres::Solve(options, &problem, &summary);

      {
        std::lock_guard<std::mutex> l(kflistMutex_);
        i = 0;
        for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
          if ((*it)->index < first_looped_index) continue;
          (*it)->updatePose(t_array[i], q_array[i].toRotationMatrix());

          if ((*it)->index == cur_index) break;
          i++;
        }

        Eigen::Vector3d cur_t, svin_t;
        Eigen::Matrix3d cur_r, svin_r;
        cur_kf->getPose(cur_t, cur_r);
        cur_kf->getSVInPose(svin_t, svin_r);
        {
          std::lock_guard<std::mutex> l(driftMutex_);
          r_drift = cur_r.transpose() * svin_r;
          yaw_drift = Utility::R2ypr(r_drift).x();
          t_drift = cur_t - r_drift * svin_t;
        }

        it++;
        for (; it != keyframelist.end(); it++) {
          Eigen::Vector3d P;
          Eigen::Matrix3d R;
          (*it)->getSVInPose(P, R);
          P = r_drift * P + t_drift;
          R = r_drift * R;
          (*it)->updatePose(P, R);
        }
      }
      updatePath();

      loop_closure_optimization_callback_(ros::Time::now().toNSec());
    }

    std::chrono::milliseconds dura(2000);
    std::this_thread::sleep_for(dura);
  }
}

void LoopClosing::updatePath() {
  std::lock_guard<std::mutex> l(kflistMutex_);

  std::list<KFMatcher*>::iterator it;
  for (int i = 1; i <= sequence_cnt; i++) {
    path[i].poses.clear();
  }
  base_path.poses.clear();
  posegraph_visualization->reset();

  if (SAVE_LOOP_PATH) {
    std::ofstream loop_path_file_tmp(svin_output_file_, std::ios::out);
    loop_path_file_tmp.close();
  }

  for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    (*it)->getPose(P, R);
    Eigen::Quaterniond Q;
    Q = R;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = (*it)->time_stamp;
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = P.x();
    pose_stamped.pose.position.y = P.y();
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    if ((*it)->sequence == 0) {
      base_path.poses.push_back(pose_stamped);
      base_path.header = pose_stamped.header;
    } else {
      path[(*it)->sequence].poses.push_back(pose_stamped);
      path[(*it)->sequence].header = pose_stamped.header;
    }

    if (SAVE_LOOP_PATH) {
      std::ofstream loop_path_file(svin_output_file_, std::ios::app);
      loop_path_file.setf(std::ios::fixed, std::ios::floatfield);

      loop_path_file.precision(9);
      loop_path_file << (*it)->time_stamp << " ";
      loop_path_file << P.x() << " " << P.y() << " " << P.z() << " " << Q.x() << " " << Q.y() << " " << Q.z() << " "
                     << Q.w() << std::endl;
      loop_path_file.close();
    }
    // draw local connection
    if (SHOW_S_EDGE) {
      std::list<KFMatcher*>::reverse_iterator rit = keyframelist.rbegin();
      std::list<KFMatcher*>::reverse_iterator lrit;
      for (; rit != keyframelist.rend(); rit++) {
        if ((*rit)->index == (*it)->index) {
          lrit = rit;
          lrit++;
          for (int i = 0; i < 4; i++) {
            if (lrit == keyframelist.rend()) break;
            if ((*lrit)->sequence == (*it)->sequence) {
              Eigen::Vector3d conncected_P;
              Eigen::Matrix3d connected_R;
              (*lrit)->getPose(conncected_P, connected_R);
              posegraph_visualization->add_edge(P, conncected_P);
            }
            lrit++;
          }
          break;
        }
      }
    }
    if (SHOW_L_EDGE) {
      if ((*it)->has_loop && (*it)->sequence == sequence_cnt) {
        KFMatcher* connected_KF = getKFPtr((*it)->loop_index);
        Eigen::Vector3d connected_P;
        Eigen::Matrix3d connected_R;
        connected_KF->getPose(connected_P, connected_R);
        (*it)->getPose(P, R);
        if ((*it)->sequence > 0) {
          posegraph_visualization->add_loopedge(P, connected_P);
        }
      }
    }
  }
  publish();
}
void LoopClosing::publish() {
  for (int i = 1; i <= sequence_cnt; i++) {
    if (i == base_sequence) {
      pubPoseGraphPath.publish(path[i]);
      pubPath[i].publish(path[i]);
      posegraph_visualization->publish_by(pubPoseGraph, path[sequence_cnt].header);
    }
  }
  pubBasePath.publish(base_path);
}

void LoopClosing::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1>& _loop_info) {
  KFMatcher* kf = getKFPtr(index);
  kf->updateLoop(_loop_info);
  if (abs(_loop_info(7)) < 30.0 && Eigen::Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0) {
    if (is_fast_localization_) {
      KFMatcher* old_kf = getKFPtr(kf->loop_index);
      Eigen::Vector3d w_P_old, w_P_cur, svin_P_cur;
      Eigen::Matrix3d w_R_old, w_R_cur, svin_R_cur;
      old_kf->getPose(w_P_old, w_R_old);
      kf->getSVInPose(svin_P_cur, svin_R_cur);

      Eigen::Vector3d relative_t;
      Eigen::Quaterniond relative_q;
      relative_t = kf->getLoopRelativeT();
      relative_q = (kf->getLoopRelativeQ()).toRotationMatrix();
      w_P_cur = w_R_old * relative_t + w_P_old;
      w_R_cur = w_R_old * relative_q;
      double shift_yaw;
      Eigen::Matrix3d shift_r;
      Eigen::Vector3d shift_t;
      shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(svin_R_cur).x();
      shift_r = Utility::ypr2R(Eigen::Vector3d(shift_yaw, 0, 0));
      shift_t = w_P_cur - w_R_cur * svin_R_cur.transpose() * svin_P_cur;

      {
        std::lock_guard<std::mutex> l(driftMutex_);
        yaw_drift = shift_yaw;
        r_drift = shift_r;
        t_drift = shift_t;
      }
    }
  }
}


void LoopClosing::registerLoopClosureOptimizationCallback(const EventCallback& optimization_finish_callback) {
  loop_closure_optimization_callback_ = optimization_finish_callback;
}
