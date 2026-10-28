#include "rgbd_slam/graph_optimizer.hpp"

#include <algorithm>

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace rgbd_slam
{

static gtsam::noiseModel::Diagonal::shared_ptr mk(const std::array<double, 6> & s)
{
  return gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << s[0], s[1], s[2], s[3], s[4], s[5]).finished());
}

GraphOptimizer::GraphOptimizer(const gtsam::Pose3 & x0, const NoiseParams & np)
{
  n_prior_ = mk(np.prior);
  n_odom_  = mk(np.odometry);
  n_vo_    = mk(np.vo);
  n_bow_   = mk(np.loop_bow);
  n_prox_  = mk(np.loop_prox);
  n_bow_use_ = n_bow_;
  n_prox_use_ = n_prox_;
  n_odom_use_ = n_odom_;

  isam_params_.relinearizeSkip = 1;             // 매 업데이트마다 재선형화
  isam_ = gtsam::ISAM2(isam_params_);

  graph_increment_.addPrior(1, x0, n_prior_);
  graph_batch_.addPrior(1, x0, n_prior_);
  initial_.insert(1, x0);
}

void GraphOptimizer::setBodyToSensor(const gtsam::Pose3 & x_vi)
{
  // Σ_s = Ad(x_iv) diag(σ_b²) Ad(x_iv)^T. GTSAM 의 AdjointMap 은 [rot; trans] 순서라 시그마 순서와 같다.
  const gtsam::Matrix6 Ad = x_vi.inverse().AdjointMap();
  const gtsam::Vector sig = n_odom_->sigmas();
  const gtsam::Matrix6 Sb = (sig.array().square()).matrix().asDiagonal();
  gtsam::Matrix6 Ss = Ad * Sb * Ad.transpose();
  Ss = 0.5 * (Ss + Ss.transpose());              // 대칭 강제 (수치오차)
  n_odom_use_ = gtsam::noiseModel::Gaussian::Covariance(Ss);
}

void GraphOptimizer::addDrNode(int from, int to,
                               const gtsam::Pose3 & dT, const gtsam::Pose3 & init_to)
{
  initial_.insert(to, init_to);
  graph_increment_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from, to, dT, n_odom_use_);
  graph_batch_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from, to, dT, n_odom_use_);
  max_key_ = std::max(max_key_, to);
}

void GraphOptimizer::addVo(int from, int to, const gtsam::Pose3 & dT)
{
  graph_increment_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from, to, dT, n_vo_);
  graph_batch_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from, to, dT, n_vo_);
  pending_ = true;
}

void GraphOptimizer::setLoopRobust(bool on, double huber_k)
{
  if (!on) {
    n_bow_use_ = n_bow_;
    n_prox_use_ = n_prox_;
    return;
  }
  const auto est = gtsam::noiseModel::mEstimator::Huber::Create(huber_k);
  n_bow_use_ = gtsam::noiseModel::Robust::Create(est, n_bow_);
  n_prox_use_ = gtsam::noiseModel::Robust::Create(est, n_prox_);
}

void GraphOptimizer::addLoop(int from, int to, const gtsam::Pose3 & dT, int type)
{
  const auto nm = (type == 1) ? n_bow_use_ : n_prox_use_;
  graph_increment_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from, to, dT, nm);
  graph_batch_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(from, to, dT, nm);
  pending_ = true;
  loop_pending_ = true;                         // 이 틱에 배치 LM 이 돈다
}

bool GraphOptimizer::optimize(std::vector<gtsam::Pose3> & poses)
{
  if (!pending_) {return false;}
  last_batch_ = false;

  // 1) iSAM2 증분 -----------------------------------------------------
  isam_.update(graph_increment_, initial_);
  const gtsam::Values inc = isam_.calculateEstimate();

  graph_increment_ = gtsam::NonlinearFactorGraph();
  initial_ = gtsam::Values();
  pending_ = false;

  poses.assign(max_key_ + 1, gtsam::Pose3());
  for (int k = 1; k <= max_key_; ++k) {
    if (inc.exists(k)) {poses[k] = inc.at<gtsam::Pose3>(k);}
  }

  if (!loop_pending_) {return true;}          // 기본은 증분 결과를 그대로 쓴다
  loop_pending_ = false;
  last_batch_ = true;

  // 2) 배치 LM — 루프가 들어온 틱에만 누적 그래프 전체를 다시 푼다 -----
  gtsam::Values batch_init;
  for (int k = 1; k <= max_key_; ++k) {
    if (inc.exists(k)) {batch_init.insert(k, inc.at<gtsam::Pose3>(k));}
  }
  const gtsam::Values result =
    gtsam::LevenbergMarquardtOptimizer(graph_batch_, batch_init).optimize();

  for (int k = 1; k <= max_key_; ++k) {
    if (result.exists(k)) {poses[k] = result.at<gtsam::Pose3>(k);}
  }

  // 3) iSAM2 재구축 — LM 해를 새 선형화점으로 전체 그래프를 다시 등록한다.
  //    이게 없으면 다음 증분 틱의 calculateEstimate() 가 LM 이전 상태를 돌려주어
  //    루프 보정이 한 틱 만에 되돌아간다. 루프 틱에만 도는 O(N) 이라 배치 LM 과 같은 예산.
  isam_ = gtsam::ISAM2(isam_params_);
  isam_.update(graph_batch_, result);
  return true;
}

}  // namespace rgbd_slam
