#include "KFMatcher.h"

const int KFMatcher::TH_HIGH = 100;
const int KFMatcher::TH_LOW = 50;
const size_t KFMatcher::briskDetectionOctaves_ = 0;                ///< The set number of brisk octaves.
const double KFMatcher::briskDetectionThreshold_ = 40.0;           ///< The set BRISK detection threshold.
const double KFMatcher::briskDetectionAbsoluteThreshold_ = 800;    ///< The set BRISK absolute detection threshold.
const size_t KFMatcher::briskDetectionMaximumKeypoints_ = 300;     ///< The set maximum number of keypoints.
const bool KFMatcher::briskDescriptionRotationInvariance_ = true;  ///< The set rotation invariance setting.
const bool KFMatcher::briskDescriptionScaleInvariance_ = false;    ///< The set scale invariance setting.
const double KFMatcher::briskMatchingThreshold_ = 80.0;            ///< The set BRISK matching threshold.

template <typename Derived>
static void reduceVector(vector<Derived>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}

KFMatcher::KFMatcher(double _time_stamp,
                     vector<Eigen::Vector3d>& _point_ids,
                     int _index,
                     Vector3d& _svin_T_w_i,
                     Matrix3d& _svin_R_w_i,
                     cv::Mat& _image,
                     vector<cv::Point3f>& _point_3d,
                     vector<cv::KeyPoint>& _point_2d_uv,
                     map<KFMatcher*, int>& KFcounter,
                     int _sequence,
                     BriefVocabulary* vocBrief) {
  time_stamp = _time_stamp;

  // @Reloc
  point_ids_ = _point_ids;

  index = _index;
  svin_T_w_i = _svin_T_w_i;
  svin_R_w_i = _svin_R_w_i;
  T_w_i = svin_T_w_i;
  R_w_i = svin_R_w_i;
  origin_svin_T = svin_T_w_i;
  origin_svin_R = svin_R_w_i;
  image = _image.clone();
  point_3d = _point_3d;
  point_2d_uv = _point_2d_uv;

  has_loop = false;
  loop_index = -1;
  has_fast_point = false;
  loop_info << 0, 0, 0, 0, 0, 0, 0, 0;
  sequence = _sequence;
  computeWindowBRIEFPoint();
  voc = vocBrief;
  computeBoW();
  KFcounter_ = KFcounter;  // for Covisibility graph
  updateConnections();     // for Covisibility graph

  computeBRIEFPoint();
}

double KFMatcher::brisk_distance(const cv::Mat& a, const cv::Mat& b) {
  const unsigned char* pa = a.ptr<unsigned char>();
  const unsigned char* pb = b.ptr<unsigned char>();
  // number_of_128_bit_words or number_of_col, L = 48
  return (double)brisk::Hamming::PopcntofXORed(pa, pb, 3 /*48 / 16*/);
}
void KFMatcher::computeBRISKPoint() {
  // for searchByDescriptor to create new BRISK keypoints and descriptors
  std::shared_ptr<cv::FeatureDetector> detector(
      new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(KFMatcher::briskDetectionThreshold_,
                                                                         KFMatcher::briskDetectionOctaves_,
                                                                         KFMatcher::briskDetectionAbsoluteThreshold_,
                                                                         KFMatcher::briskDetectionMaximumKeypoints_));

  std::shared_ptr<cv::DescriptorExtractor> extractor(new brisk::BriskDescriptorExtractor(
      KFMatcher::briskDescriptionRotationInvariance_, KFMatcher::briskDescriptionScaleInvariance_));

  detector->detect(image, brisk_keypoints);

  if (!window_keypoints.empty())
    extractor->compute(image, window_keypoints, window_brisk_descriptors);
  else {
    std::cout << "window keypoints are empty. This is a problem!!" << std::endl;
  }
  extractor->compute(image, brisk_keypoints, brisk_descriptors);

  std::cout << "Size of Brisk keypoints: " << brisk_keypoints.size() << std::endl;
}

void KFMatcher::computeBoW() {
  if (bowVec.empty() || featVec.empty()) {
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    voc->transform(brief_descriptors, bowVec);
  }
}

void KFMatcher::updateConnections() {
  if (KFcounter_.empty()) {
    std::cout << "KFcounter is empty for KF: " << index << " This SHOULDN't be happening except 1st frame."
              << std::endl;
    return;
  }

  // std::cout<<"Weights for observed keyframes in Kf: "<< this->index << std::endl;
  int th_weight = 20;  // TODO Sharmin: Move it to the Config file
  for (map<KFMatcher*, int>::iterator mit = KFcounter_.begin(); mit != KFcounter_.end(); mit++) {
    if (mit->second > th_weight) {
      mConnectedKeyFrameWeights.insert(std::make_pair(mit->first, mit->second));
      std::cout << "Observed Kf: " << mit->first->index << " with weight(common MapPoint): " << mit->second
                << std::endl;
    }
  }
}

// Note Keypoints found by okvis_estimator
void KFMatcher::computeWindowBRIEFPoint() {
  BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());

  window_keypoints = point_2d_uv;

  extractor(image, window_keypoints, window_brief_descriptors);

  for (int i = 0; i < (int)window_keypoints.size(); i++) {
    Eigen::Vector3d tmp_p;

    project_normal(Eigen::Vector2d(window_keypoints[i].pt.x, window_keypoints[i].pt.y), tmp_p);

    cv::KeyPoint tmp_norm;
    tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
    window_keypoints_norm.push_back(tmp_norm);
  }
}

void KFMatcher::project_normal(Eigen::Vector2d kp, Eigen::Vector3d& point3d) const {
  const float invfx = 1.0f / p_fx;
  const float invfy = 1.0f / p_fy;

  const float u = kp[0];
  const float v = kp[1];
  point3d[0] = (u - p_cx) * invfx;
  point3d[1] = (v - p_cy) * invfy;
  point3d[2] = 1.0;
}

void KFMatcher::computeBRIEFPoint() {
  BriefExtractor extractor(BRIEF_PATTERN_FILE.c_str());
  const int fast_th = 20;  // corner detector response threshold
  if (1)
    cv::FAST(image, keypoints, fast_th, true);
  else {
    vector<cv::Point2f> tmp_pts;
    cv::goodFeaturesToTrack(image, tmp_pts, 500, 0.01, 10);
    for (int i = 0; i < (int)tmp_pts.size(); i++) {
      cv::KeyPoint key;
      key.pt = tmp_pts[i];
      keypoints.push_back(key);
    }
  }
  extractor(image, keypoints, brief_descriptors);

  for (int i = 0; i < (int)keypoints.size(); i++) {
    Eigen::Vector3d tmp_p;

    project_normal(Eigen::Vector2d(keypoints[i].pt.x, keypoints[i].pt.y), tmp_p);

    cv::KeyPoint tmp_norm;
    tmp_norm.pt = cv::Point2f(tmp_p.x() / tmp_p.z(), tmp_p.y() / tmp_p.z());
    keypoints_norm.push_back(tmp_norm);
  }
}

void BriefExtractor::operator()(const cv::Mat& im,
                                vector<cv::KeyPoint>& keys,
                                vector<BRIEF::bitset>& descriptors) const {
  m_brief.compute(im, keys, descriptors);
}

bool KFMatcher::matchBrisk(const cv::Mat& window_descriptor,
                           const cv::Mat& descriptors_old,
                           const std::vector<cv::KeyPoint>& keypoints_old,
                           cv::Point2f& best_match) {
  cv::Point2f best_pt;
  int bestDist = 256;
  int bestIndex = -1;
  for (size_t i = 0; i < descriptors_old.rows; i++) {
    double dis = brisk_distance(window_descriptor, descriptors_old.row(i));
    if (dis < bestDist) {
      bestDist = dis;
      bestIndex = i;
    }
  }
  // printf("best dist %d", bestDist);
  if (bestIndex != -1 && bestDist < briskMatchingThreshold_) {
    best_match = keypoints_old[bestIndex].pt;
    return true;
  } else
    return false;
}

void KFMatcher::searchByBRISKDescriptor(std::vector<cv::Point2f>& matched_2d_old,
                                        std::vector<uchar>& status,
                                        const cv::Mat& descriptors_old,
                                        const std::vector<cv::KeyPoint>& keypoints_old) {
  for (size_t i = 0; i < window_brisk_descriptors.rows; i++) {
    cv::Point2f pt(0.f, 0.f);
    if (matchBrisk(window_brisk_descriptors.row(i), descriptors_old, keypoints_old, pt))
      status.push_back(1);
    else
      status.push_back(0);
    matched_2d_old.push_back(pt);
  }
}

bool KFMatcher::searchInAera(const BRIEF::bitset window_descriptor,
                             const std::vector<BRIEF::bitset>& descriptors_old,
                             const std::vector<cv::KeyPoint>& keypoints_old,
                             const std::vector<cv::KeyPoint>& keypoints_old_norm,
                             cv::Point2f& best_match,
                             cv::Point2f& best_match_norm) {
  cv::Point2f best_pt;
  int bestDist = 128;
  int bestIndex = -1;
  for (int i = 0; i < (int)descriptors_old.size(); i++) {
    int dis = HammingDis(window_descriptor, descriptors_old[i]);
    if (dis < bestDist) {
      bestDist = dis;
      bestIndex = i;
    }
  }
  // printf("best dist %d", bestDist);
  if (bestIndex != -1 && bestDist < 80) {
    best_match = keypoints_old[bestIndex].pt;
    best_match_norm = keypoints_old_norm[bestIndex].pt;
    return true;
  } else
    return false;
}
void KFMatcher::searchByBRIEFDes(std::vector<cv::Point2f>& matched_2d_old,
                                 std::vector<cv::Point2f>& matched_2d_old_norm,
                                 std::vector<uchar>& status,
                                 const std::vector<BRIEF::bitset>& descriptors_old,
                                 const std::vector<cv::KeyPoint>& keypoints_old,
                                 const std::vector<cv::KeyPoint>& keypoints_old_norm) {
  for (int i = 0; i < (int)window_brief_descriptors.size(); i++) {
    cv::Point2f pt(0.f, 0.f);
    cv::Point2f pt_norm(0.f, 0.f);
    if (searchInAera(window_brief_descriptors[i], descriptors_old, keypoints_old, keypoints_old_norm, pt, pt_norm))
      status.push_back(1);
    else
      status.push_back(0);
    matched_2d_old.push_back(pt);
    matched_2d_old_norm.push_back(pt_norm);
  }
}

void KFMatcher::PnPRANSAC(const vector<cv::Point2f>& matched_2d_old_norm,
                          const std::vector<cv::Point3f>& matched_3d,
                          std::vector<uchar>& status,
                          Eigen::Vector3d& PnP_T_old,
                          Eigen::Matrix3d& PnP_R_old) {
  cv::Mat r, rvec, t, D, tmp_r;
  cv::Mat K = (cv::Mat_<double>(3, 3) << 1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0);
  Matrix3d R_inital;
  Vector3d P_inital;

  Matrix3d R_w_c = origin_svin_R;
  Vector3d T_w_c = origin_svin_T;

  R_inital = R_w_c.inverse();
  P_inital = -(R_inital * T_w_c);

  cv::eigen2cv(R_inital, tmp_r);
  cv::Rodrigues(tmp_r, rvec);
  cv::eigen2cv(P_inital, t);

  cv::Mat inliers;
  TicToc t_pnp_ransac;

  // bjoshi
  // Temporary fix for https://github.com/opencv/opencv/issues/17799
  // This is a bug in opencv. The bug is fixed in opencv master branch.
  try {
    if (CV_MAJOR_VERSION < 3)
      solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, false, 100, 10.0 / 230, 100, inliers);
    else {
      if (CV_MINOR_VERSION < 2)
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, false, 100, sqrt(10.0 / 230), 0.99, inliers);
      else
        solvePnPRansac(matched_3d, matched_2d_old_norm, K, D, rvec, t, false, 100, 10.0 / 230, 0.99, inliers);
    }
  } catch (cv::Exception e) {
    // std::cout << "Caught exception in PnPRANSAC:" << e.what() << std::endl;
    inliers.setTo(cv::Scalar(0));
  }

  for (int i = 0; i < (int)matched_2d_old_norm.size(); i++) status.push_back(0);

  for (int i = 0; i < inliers.rows; i++) {
    int n = inliers.at<int>(i);
    status[n] = 1;
  }

  cv::Rodrigues(rvec, r);
  Matrix3d R_pnp, R_w_c_old;
  cv::cv2eigen(r, R_pnp);
  R_w_c_old = R_pnp.transpose();
  Vector3d T_pnp, T_w_c_old;
  cv::cv2eigen(t, T_pnp);
  T_w_c_old = R_w_c_old * (-T_pnp);

  PnP_R_old = R_w_c_old;
  PnP_T_old = T_w_c_old;
}

/*
int KFMatcher::searchByBoW(KFMatcher* old_kf, vector<cv::Point3f> &vpMatches12, vector<bool> &vbMatched2){
        const vector<cv::KeyPoint> &vKeys2 = old_kf->point_2d_uv;
        const DBoW2::FeatureVector &vFeatVec2 = old_kf->featVec;
        const vector<cv::Point3f> &vMapPoints2 = old_kf->point_3d;
        const vector<BRIEF::bitset> &vDescriptor2 = old_kf->window_brief_descriptors;

        vpMatches12 = vector<cv::Point3f>(point_3d.size(), cv::Point3f(-1000.0, -1000.0, -1000.0));  // FIXME SHarmin:
Better way to initialize vbMatched2 = vector<bool>(vMapPoints2.size(),false);

        int nmatches = 0;

        DBoW2::FeatureVector::const_iterator f1it = featVec.begin();
        DBoW2::FeatureVector::const_iterator f2it = vFeatVec2.begin();
        DBoW2::FeatureVector::const_iterator f1end = featVec.end();
        DBoW2::FeatureVector::const_iterator f2end = vFeatVec2.end();

        cv::Mat gray_img, loop_match_img;
        cv::Mat old_img = old_kf->image;
        cv::hconcat(image, old_img, gray_img);
        cvtColor(gray_img, loop_match_img, CV_GRAY2RGB);
        for(int i = 0; i< (int)point_2d_uv.size(); i++)
        {
                cv::Point2f cur_pt = point_2d_uv[i];
                cv::circle(loop_match_img, cur_pt, 5, cv::Scalar(0, 255, 0));
        }
        for(int i = 0; i< (int)old_kf->window_keypoints.size(); i++)
        {
                cv::Point2f old_pt = old_kf->window_keypoints[i].pt;
                old_pt.x += COL;
                cv::circle(loop_match_img, old_pt, 5, cv::Scalar(0, 255, 0));
        }

        while(f1it != f1end && f2it != f2end)
        {
                if(f1it->first == f2it->first)
                {
                        for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
                        {
                                const size_t idx1 = f1it->second[i1];

                                cv::Point3f pMP1 = point_3d[idx1]; // TODO Sharmin: doulbe check if produces correct
result const BRIEF::bitset &d1 = window_brief_descriptors[idx1];

                                int bestDist1=256;
                                int bestIdx2 =-1 ;
                                int bestDist2=256;

                                for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
                                {
                                        const size_t idx2 = f2it->second[i2];

                                        cv::Point3f pMP2 = vMapPoints2[idx2];

                                        if(vbMatched2[idx2])
                                                continue;

                                        const BRIEF::bitset &d2 = vDescriptor2[idx2];

                                        int dist = HammingDis(d1,d2);

                                        if(dist<bestDist1)
                                        {
                                                bestDist2=bestDist1;
                                                bestDist1=dist;
                                                bestIdx2=idx2;
                                        }
                                        else if(dist<bestDist2)
                                        {
                                                bestDist2=dist;
                                        }
                                }

                                if(bestDist1<TH_LOW)
                                {
                                        //if(static_cast<float>(bestDist1)<0.75*static_cast<float>(bestDist2))  // TODO
Sharmin: Decide the ratio: 0.75
                                        {
                                                vpMatches12[idx1]=vMapPoints2[bestIdx2];
                                                vbMatched2[bestIdx2]=true;
                                                nmatches++;

                                                // Debug
                                                cv::Point2f old_pt = old_kf->window_keypoints[bestIdx2].pt;
                                                old_pt.x +=  COL;
                                                cv::line(loop_match_img, point_2d_uv[idx1], old_pt, cv::Scalar(0, 255,
0), 1, 8, 0);
                                        }
                                }
                        }

                        f1it++;
                        f2it++;
                }
                else if(f1it->first < f2it->first)
                {
                        f1it = featVec.lower_bound(f2it->first);
                }
                else
                {
                        f2it = vFeatVec2.lower_bound(f1it->first);
                }
        }

        ostringstream path;
        path << "/media/afrl/Elements/Sharmin/svin_loop/loop_image/"
                        << index << "-"
                        << old_kf->index << ".jpg";
        cv::imwrite( path.str().c_str(), loop_match_img);

        return nmatches;
}*/

bool KFMatcher::findConnection(KFMatcher* old_kf) {
  TicToc tmp_t;

  vector<cv::KeyPoint> matched_2d_cur;
  vector<cv::Point2f> matched_2d_old;
  vector<cv::Point2f> matched_2d_old_norm;
  vector<cv::Point3f> matched_3d;
  vector<Eigen::Vector3d> matched_ids;  // Reloc
  vector<uchar> status;

  matched_3d = point_3d;
  matched_2d_cur = point_2d_uv;
  matched_ids = point_ids_;

  searchByBRIEFDes(matched_2d_old,
                   matched_2d_old_norm,
                   status,
                   old_kf->brief_descriptors,
                   old_kf->keypoints,
                   old_kf->keypoints_norm);
  // searchByBRISKDescriptor(matched_2d_old, status, old_kf->brisk_descriptors, old_kf->brisk_keypoints);
  reduceVector(matched_2d_old, status);
  reduceVector(matched_3d, status);
  reduceVector(matched_2d_cur, status);
  reduceVector(matched_2d_old_norm, status);
  reduceVector(matched_ids, status);

  std::cout << "Size Before RANSAC: " << matched_2d_cur.size() << std::endl;

  Eigen::Vector3d PnP_T_old;
  Eigen::Matrix3d PnP_R_old;
  Eigen::Vector3d relative_t;
  Quaterniond relative_q;
  double relative_yaw;

  if ((int)matched_2d_cur.size() > MIN_LOOP_NUM) {
    status.clear();
    PnPRANSAC(matched_2d_old_norm, matched_3d, status, PnP_T_old, PnP_R_old);
    reduceVector(matched_2d_cur, status);
    reduceVector(matched_2d_old, status);
    reduceVector(matched_2d_old_norm, status);
    reduceVector(matched_3d, status);
    reduceVector(matched_ids, status);
  }

  // std::cout<< "Size after RANSAC "<< matched_2d_cur.size() << std::endl;

  if ((int)matched_2d_cur.size() > MIN_LOOP_NUM) {
    relative_t = PnP_R_old.transpose() * (origin_svin_T - PnP_T_old);
    relative_q = PnP_R_old.transpose() * origin_svin_R;
    relative_yaw = Utility::normalizeAngle(Utility::R2ypr(origin_svin_R).x() - Utility::R2ypr(PnP_R_old).x());

    if (abs(relative_yaw) < 30.0 && relative_t.norm() < 20.0) {
      has_loop = true;
      loop_index = old_kf->index;
      loop_info << relative_t.x(), relative_t.y(), relative_t.z(), relative_q.w(), relative_q.x(), relative_q.y(),
          relative_q.z(), relative_yaw;

      std::cout << index << " has Loop with: " << loop_index << std::endl;
      if (FAST_RELOCALIZATION) {
        sensor_msgs::PointCloud msg_match_points;
        msg_match_points.header.stamp = ros::Time(time_stamp);

        // Note that this PointCloud msg is not for visualization
        for (int i = 0; i < (int)matched_ids.size(); i++) {
          // landmarkId, mfId/poseId, keypointIdx for Every Matched 3d points in Current frame
          geometry_msgs::Point32 p;
          p.x = matched_ids[i].x();
          p.y = matched_ids[i].y();
          p.z = matched_ids[i].z();
          msg_match_points.points.push_back(p);
        }
        // T, R in Old frame
        Eigen::Vector3d T = old_kf->T_w_i;
        Eigen::Matrix3d R = old_kf->R_w_i;
        Quaterniond Q(R);
        sensor_msgs::ChannelFloat32 t_q_index;
        t_q_index.values.push_back(T.x());
        t_q_index.values.push_back(T.y());
        t_q_index.values.push_back(T.z());
        t_q_index.values.push_back(Q.w());
        t_q_index.values.push_back(Q.x());
        t_q_index.values.push_back(Q.y());
        t_q_index.values.push_back(Q.z());

        msg_match_points.channels.push_back(t_q_index);
        pubMatchedPoints.publish(msg_match_points);
      }
      return true;
    }
  }

  return false;
}

int KFMatcher::HammingDis(const BRIEF::bitset& a, const BRIEF::bitset& b) {
  BRIEF::bitset xor_of_bitset = a ^ b;
  int dis = xor_of_bitset.count();
  return dis;
}

void KFMatcher::getSVInPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i) {
  _T_w_i = svin_T_w_i;
  _R_w_i = svin_R_w_i;
}

void KFMatcher::getPose(Eigen::Vector3d& _T_w_i, Eigen::Matrix3d& _R_w_i) {
  _T_w_i = T_w_i;
  _R_w_i = R_w_i;
}

void KFMatcher::updatePose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i) {
  T_w_i = _T_w_i;
  R_w_i = _R_w_i;
}

void KFMatcher::updateSVInPose(const Eigen::Vector3d& _T_w_i, const Eigen::Matrix3d& _R_w_i) {
  svin_T_w_i = _T_w_i;
  svin_R_w_i = _R_w_i;
  T_w_i = svin_T_w_i;
  R_w_i = svin_R_w_i;
}

Eigen::Vector3d KFMatcher::getLoopRelativeT() { return Eigen::Vector3d(loop_info(0), loop_info(1), loop_info(2)); }

Eigen::Quaterniond KFMatcher::getLoopRelativeQ() {
  return Eigen::Quaterniond(loop_info(3), loop_info(4), loop_info(5), loop_info(6));
}

double KFMatcher::getLoopRelativeYaw() { return loop_info(7); }

void KFMatcher::updateLoop(Eigen::Matrix<double, 8, 1>& _loop_info) {
  if (abs(_loop_info(7)) < 30.0 && Vector3d(_loop_info(0), _loop_info(1), _loop_info(2)).norm() < 20.0) {
    // printf("update loop info\n");
    loop_info = _loop_info;
  }
}

BriefExtractor::BriefExtractor(const std::string& pattern_file) {
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage fs(pattern_file.c_str(), cv::FileStorage::READ);
  if (!fs.isOpened()) throw string("Could not open file ") + pattern_file;

  vector<int> x1, y1, x2, y2;
  fs["x1"] >> x1;
  fs["x2"] >> x2;
  fs["y1"] >> y1;
  fs["y2"] >> y2;

  m_brief.importPairs(x1, y1, x2, y2);
}
