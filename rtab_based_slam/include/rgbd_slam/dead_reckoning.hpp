#pragma once

// 추측항법 (DR) — 휠 전진속도 + 자이로 적분.
// 차체프레임은 FRD. 전진이 +x 인 건 FLU/FRD 공통이고 각속도 적분도 w 가 차체프레임이기만 하면
// 규약과 무관하므로 여기엔 좌표계 분기가 없다.

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

namespace rgbd_slam
{

/// SVD 로 SO(3) 재투영. GTSAM Pose3 생성자는 회전을 SO(3) 로 투영하지 않아서, 이걸 빼면
/// 비정규직교 드리프트가 최적화 피드백을 타고 발산한다.
Eigen::Matrix3d so3Project(const Eigen::Matrix3d & R);

/// 각도 증분 벡터 w*dt [rad] 의 지수사상.
Eigen::Matrix3d expmap(const Eigen::Vector3d & w_dt);

/// 비홀로노믹 전진 적분: p_new = p + R_abs * [v*dt; 0; 0], 자세는 R_abs 로 교체.
gtsam::Pose3 propagateX(const gtsam::Pose3 & pose, double v, const Eigen::Matrix3d & R_abs, double dt);

/// DR 상태 두 트랙.
///   보정 트랙 (x_gv) : 최적화 결과를 받아 이어가는 트랙 — 그래프 노드와 출력의 원천
///   무보정 트랙 (x_dw): 피드백 없는 순수 DR — 종료 요약의 비교 기준
class DeadReckoning
{
public:
  /// x_vi : 차체(FRD) -> 카메라 광학 변환
  explicit DeadReckoning(const gtsam::Pose3 & x_vi);

  /// IMU 한 틱 전파. v [m/s] 휠 전진속도, w [rad/s] 자이로 3축
  void step(double v, const Eigen::Vector3d & w, double dt);

  /// 최적화 결과에서 보정 트랙 재개. 직전 step() 의 회전 증분을 이어받는다.
  void resetFromOptimized(const gtsam::Pose3 & x_gi_opt, double v, double dt);

  const gtsam::Pose3 & xGi() const {return x_gi_;}   ///< 보정 트랙, 카메라 광학 절대포즈 (그래프 노드)
  const gtsam::Pose3 & xGv() const {return x_gv_;}   ///< 보정 트랙, 차체 절대포즈 (출력)
  const gtsam::Pose3 & xDr() const {return x_dr_;}   ///< 무보정 트랙, 카메라 광학 절대포즈
  const gtsam::Pose3 & xIv() const {return x_iv_;}   ///< 카메라 광학 -> 차체

private:
  gtsam::Pose3 x_vi_, x_iv_;
  gtsam::Pose3 x_gv_, x_dw_, x_gi_, x_dr_;
  Eigen::Matrix3d R_att_, R_gv_, dR_;
};

}  // namespace rgbd_slam
