#include "pose_graph/PoseGraph.h"

#include <list>
#include <map>
#include <set>
#include <string>

#include "pose_graph/Pose3DError.h"

PoseGraph::PoseGraph() {
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
  is_fast_localization_ = true;
}

PoseGraph::~PoseGraph() { t_optimization.join(); }

void PoseGraph::set_fast_relocalization(const bool fast_relocalization) { is_fast_localization_ = fast_relocalization; }

void PoseGraph::setBriefVocAndDB(BriefVocabulary* vocabulary, BriefDatabase database) {
  voc = vocabulary;
  db = database;
}

void PoseGraph::startOptimizationThread(bool vio_only_optimization) {
  t_optimization = std::thread(&PoseGraph::optimize4DoFPoseGraph, this);
}

void PoseGraph::addKFToPoseGraph(Keyframe* cur_kf, bool flag_detect_loop) {
  // shift to base frame
  Eigen::Vector3d svin_P_cur;
  Eigen::Matrix3d svin_R_cur;
  if (sequence_cnt != cur_kf->sequence) {
    sequence_cnt++;
    sequence_loop.push_back(0);
    w_t_svin = Eigen::Vector3d(0, 0, 0);
    w_r_svin = Eigen::Matrix3d::Identity();

    driftMutex_.lock();
    t_drift = Eigen::Vector3d(0, 0, 0);
    r_drift = Eigen::Matrix3d::Identity();
    driftMutex_.unlock();
  }

  cur_kf->getSVInPose(svin_P_cur, svin_R_cur);
  svin_P_cur = w_r_svin * svin_P_cur + w_t_svin;
  svin_R_cur = w_r_svin * svin_R_cur;
  cur_kf->updateSVInPose(svin_P_cur, svin_R_cur);
  cur_kf->index = global_index;
  global_index++;
  std::set<Keyframe*> loopCandidates;
  int loop_index = -1;

  if (flag_detect_loop) {  // at least 20 KF has been passed
    loop_index = detectLoop(cur_kf, cur_kf->index);
  } else {
    db.add(cur_kf->brief_descriptors);
  }

  if (loop_index != -1) {
    Keyframe* old_kf = getKFPtr(loop_index);
    if (cur_kf->findConnection(old_kf)) {
      if (earliest_loop_index > loop_index || earliest_loop_index == -1) earliest_loop_index = loop_index;
      VLOG(1) << "Found Loop  with keyframe: " << loop_index << " and " << cur_kf->index;
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
        std::list<Keyframe*>::iterator it = keyframelist.begin();
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

      optimizationMutex_.lock();
      optimizationBuffer_.push(cur_kf->index);
      optimizationMutex_.unlock();
    }
  }

  Eigen::Vector3d P;
  Eigen::Matrix3d R;
  std::pair<Eigen::Vector3d, Eigen::Vector3d> loop_info;
  loop_info.first = Eigen::Vector3d::Zero();
  loop_info.second = Eigen::Vector3d::Zero();

  kflistMutex_.lock();
  std::pair<Timestamp, Eigen::Matrix4d> pose;
  cur_kf->getSVInPose(P, R);
  P = r_drift * P + t_drift;
  R = r_drift * R;
  cur_kf->updatePose(P, R);
  Eigen::Quaterniond Q{R};

  if (cur_kf->has_loop) {
    Keyframe* connected_KF = getKFPtr(cur_kf->loop_index);
    Eigen::Vector3d connected_P, P0;
    Eigen::Matrix3d connected_R, R0;
    connected_KF->getPose(connected_P, connected_R);
    cur_kf->getPose(P0, R0);
    loop_info = {P0, connected_P};
  }

  keyframelist.push_back(cur_kf);
  pose.first = cur_kf->time_stamp;
  kflistMutex_.unlock();

  pose.second.block<3, 3>(0, 0) = R;
  pose.second.block<3, 1>(0, 3) = P;
  CHECK(keyframe_pose_callback_);
  keyframe_pose_callback_(pose, loop_info);
  VLOG(10) << "Called Keyframe Callback";
}

Keyframe* PoseGraph::getKFPtr(int index) {
  std::list<Keyframe*>::iterator it = keyframelist.begin();
  for (; it != keyframelist.end(); it++) {
    if ((*it)->index == index) break;
  }
  if (it != keyframelist.end())
    return *it;
  else
    return NULL;
}

int PoseGraph::detectLoop(Keyframe* keyframe, int frame_index) {
  cv::Mat compressed_image;

  if (keyframe->bowVec.empty() || keyframe->featVec.empty()) {
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    voc->transform(keyframe->brief_descriptors, keyframe->bowVec);
  }

  float min_score = 1.0;
  for (std::map<Keyframe*, int>::iterator mit = keyframe->mConnectedKeyFrameWeights.begin();
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

  // first query; then add this frame into database!
  DBoW2::QueryResults ret;
  db.query(keyframe->bowVec, ret, 4, frame_index - 50);
  db.add(keyframe->brief_descriptors);

  bool find_loop = false;
  cv::Mat loop_result;

  for (unsigned int i = 0; i < ret.size(); i++) {
    if (ret[i].Score > 0.60 * min_score) {
      find_loop = true;
      // std::cout<< "Query KF: "<< frame_index<< " candidate kf: "<< ret[i].Id << std::endl;
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

void PoseGraph::optimize4DoFPoseGraph() {
  while (true) {
    int cur_index = -1;
    int first_looped_index = -1;

    optimizationMutex_.lock();
    while (!optimizationBuffer_.empty()) {
      cur_index = optimizationBuffer_.front();
      first_looped_index = earliest_loop_index;
      optimizationBuffer_.pop();
    }
    optimizationMutex_.unlock();

    if (cur_index != -1) {
      ceres::Problem problem;
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
      options.max_num_iterations = 5;
      ceres::Solver::Summary summary;
      ceres::LossFunction* loss_function;
      loss_function = new ceres::HuberLoss(0.1);

      kflistMutex_.lock();
      Keyframe* cur_kf = getKFPtr(cur_index);

      if (cur_kf == NULL) {
        LOG(ERROR) << "Current Keyframe is NULL";
      }

      int max_length = cur_index + 1;

      double t_array[max_length][3];
      Eigen::Quaterniond q_array[max_length];  // NOLINT
      double euler_array[max_length][3];       // NOLINT
      double sequence_array[max_length];       // NOLINT

      ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

      std::list<Keyframe*>::iterator it;

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

        if ((*it)->index == first_looped_index) {
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

      kflistMutex_.lock();

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
      Timestamp current_stamp = cur_kf->time_stamp;
      cur_kf->getPose(cur_t, cur_r);
      cur_kf->getSVInPose(svin_t, svin_r);

      driftMutex_.lock();
      yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(svin_r).x();
      r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
      t_drift = cur_t - r_drift * svin_t;
      driftMutex_.unlock();

      it++;
      for (; it != keyframelist.end(); it++) {
        Eigen::Vector3d P;
        Eigen::Matrix3d R;
        (*it)->getSVInPose(P, R);
        P = r_drift * P + t_drift;
        R = r_drift * R;
        (*it)->updatePose(P, R);
      }
      kflistMutex_.unlock();
      updatePath();
      if (loop_closure_optimization_callback_) {
        loop_closure_optimization_callback_(current_stamp);
      }
    }
    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
  }
}

void PoseGraph::updatePath() {
  kflistMutex_.lock();

  std::list<Keyframe*>::iterator it;

  std::vector<std::pair<Timestamp, Eigen::Matrix4d>> loop_closure_path;
  std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> loop_closure_edges;

  for (it = keyframelist.begin(); it != keyframelist.end(); it++) {
    Eigen::Vector3d P;
    Eigen::Matrix3d R;
    (*it)->getPose(P, R);
    Eigen::Quaterniond Q{R};

    std::pair<Timestamp, Eigen::Matrix4d> pose;
    pose.first = (*it)->time_stamp;
    pose.second.block<3, 3>(0, 0) = R;
    pose.second.block<3, 1>(0, 3) = P;
    loop_closure_path.push_back(pose);

    if ((*it)->has_loop) {
      Keyframe* connected_KF = getKFPtr((*it)->loop_index);
      Eigen::Vector3d connected_P;
      Eigen::Matrix3d connected_R;
      connected_KF->getPose(connected_P, connected_R);
      (*it)->getPose(P, R);
      loop_closure_edges.push_back({P, connected_P});
    }
  }

  kflistMutex_.unlock();

  CHECK(loop_closure_callback_);
  loop_closure_callback_(loop_closure_path, loop_closure_edges);
}

void PoseGraph::updateKeyFrameLoop(int index, Eigen::Matrix<double, 8, 1>& _loop_info) {
  Keyframe* kf = getKFPtr(index);
  kf->updateLoop(_loop_info);
  if (abs(_loop_info(7)) < 30.0 && Eigen::Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0) {
    if (is_fast_localization_) {
      Keyframe* old_kf = getKFPtr(kf->loop_index);
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

void PoseGraph::setLoopClosureOptimizationCallback(const EventCallback& optimization_finish_callback) {
  loop_closure_optimization_callback_ = optimization_finish_callback;
}

void PoseGraph::setKeyframePoseCallback(const KeframeWithLoopClosureCallback& keyframe_pose_callback) {
  keyframe_pose_callback_ = keyframe_pose_callback;
}

void PoseGraph::setLoopClosureCallback(const PathWithLoopClosureCallback& loop_closure_callback) {
  loop_closure_callback_ = loop_closure_callback;
}
