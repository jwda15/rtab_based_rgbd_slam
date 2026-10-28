#pragma once

// GTSAM 팩터그래프.
//   graph_increment_ : iSAM2 용, 업데이트마다 비움
//   graph_batch_     : 누적 그래프, 루프 틱의 배치 LM 용
//
// ★ 배치 LM 은 루프 엣지가 들어온 틱에만 돈다. 평소에는 iSAM2 증분 결과를 그대로 쓴다.
// ★ 배치 LM 뒤에는 iSAM2 를 전체 그래프 + LM 해로 재구축한다. 안 하면 다음 증분 틱에서
//   iSAM2 가 LM 이전 추정치를 돌려주어 루프로 펴진 궤적이 되돌아간다.

#include <array>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace rgbd_slam
{

/// 팩터별 시그마 [rx, ry, rz (rad), tx, ty, tz (m)]. slam_node 가 param.yaml 의 noise_* 로 채운다.
struct NoiseParams
{
  std::array<double, 6> prior{};      ///< 1번 노드 고정
  std::array<double, 6> odometry{};   ///< DR (차체축 기준 — setBodyToSensor 가 광학프레임으로 바꾼다)
  std::array<double, 6> vo{};         ///< rtabmap 이웃링크
  std::array<double, 6> loop_bow{};   ///< rtabmap 루프 (type 1)
  std::array<double, 6> loop_prox{};  ///< rtabmap 근접 루프 (type 2)
};

class GraphOptimizer
{
public:
  /// x0 : 1번 노드의 초기 포즈 (센서 절대포즈)
  GraphOptimizer(const gtsam::Pose3 & x0, const NoiseParams & np);

  /// DR 노드 추가. between 팩터와 to 노드 초기값을 함께 넣는다.
  void addDrNode(int from, int to, const gtsam::Pose3 & dT, const gtsam::Pose3 & init_to);

  /// VO 엣지 추가. 이게 들어와야 최적화가 돈다.
  void addVo(int from, int to, const gtsam::Pose3 & dT);

  /// 루프 엣지 추가 (type 1 = 루프, 그 외 = 근접). 이 틱에는 배치 LM 이 돈다.
  void addLoop(int from, int to, const gtsam::Pose3 & dT, int type);

  /// 루프 팩터에 Huber 커널. 가짜 루프 하나가 궤적을 접는 걸 막고, 꼬리가 선형이라 큰 드리프트를
  /// 닫는 진짜 루프는 살아남는다. k 는 표준화 잔차 단위.
  void setLoopRobust(bool on, double huber_k);

  /// DR 노이즈(차체축)를 그래프 변수(광학 포즈)의 상대포즈 공분산으로 바꾼다.
  ///   ΔS = x_iv ΔB x_vi  →  Σ_s = Ad(x_iv) Σ_b Ad(x_iv)^T  (틸트·레버암 때문에 축이 섞인다)
  void setBodyToSensor(const gtsam::Pose3 & x_vi);

  /// 최적화 대기 중인지 (VO/루프 엣지가 하나라도 붙었는지)
  bool pending() const {return pending_;}

  /// iSAM2 증분 (+ 루프가 들어온 틱이면 배치 LM). poses 는 1-base 로 채워진다.
  bool optimize(std::vector<gtsam::Pose3> & poses);

  /// 직전 optimize() 가 배치 LM 까지 돌았는지
  bool lastRunWasBatch() const {return last_batch_;}

private:
  gtsam::noiseModel::Diagonal::shared_ptr n_prior_, n_odom_, n_vo_, n_bow_, n_prox_;
  gtsam::noiseModel::Base::shared_ptr n_odom_use_;               ///< 광학프레임 변환본 (addDrNode 가 쓴다)
  gtsam::noiseModel::Base::shared_ptr n_bow_use_, n_prox_use_;   ///< 강건 커널 적용본 (addLoop 가 쓴다)
  gtsam::ISAM2Params isam_params_;

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_increment_;
  gtsam::NonlinearFactorGraph graph_batch_;
  gtsam::Values initial_;
  int max_key_ = 1;
  bool pending_ = false;
  bool loop_pending_ = false;
  bool last_batch_ = false;
};

}  // namespace rgbd_slam
