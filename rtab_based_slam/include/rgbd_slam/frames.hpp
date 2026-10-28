#pragma once

// 좌표계 규약
// ============================================================================
// 이 패키지의 내부 계산은 전부 FRD/NED 로 통일한다. 예외는 없다.
//
//   차체 V (body)    : FRD — x 전방, y 우측, z 하방
//   글로벌 G         : 차체프레임의 시작 시점 축과 원점. 차체가 FRD 이므로
//                      G 도 FRD(=국소 NED 형태)가 된다.
//   센서 I (optical) : RDF — x 우, y 하, z 전. rtabmap 이 만드는 프레임이라
//                      건드리지 않는다. /odom, /mapGraph 의 상대포즈가 전부 이것이다.
//
// 그래프 노드는 x_gi (G -> I) 이고 팩터의 측정치는 센서프레임 상대포즈인데,
// 상대포즈는 G 를 어떻게 잡든 불변이다. 그래서 FLU -> FRD 전환에서 실제로 바뀌는 건
//   (1) 외부파라미터 x_vi  (2) 차체프레임으로 들어오는 입력  (3) 나가는 출력
// 이 셋뿐이다.
//
// FLU <-> FRD 는 x축 180도 회전 S = diag(1,-1,-1) 하나로 끝난다.
// det(S)=+1 이라 진짜 회전이고, S = S^T = S^-1 이라 자기 자신이 역변환이다.
//   차체프레임 벡터           : v_frd = S v_flu       (입력 자이로·휠, enu_input)
//   G->V 포즈 (양쪽 다 차체계) : T_frd = S T S         (출력 차체 포즈, enu_output)
//
// rpy 는 항공 표준 ZYX(yaw-pitch-roll) 오일러다. FRD 는 z 가 아래를 향하므로
// pitch 양수가 기수 상향이고, 45도 하방 틸트 카메라는 pitch = -45 다.

#include <cmath>

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>

namespace rgbd_slam
{

/// FLU <-> FRD 축 변환 S = diag(1,-1,-1). 자기 자신이 역변환.
inline const Eigen::Matrix3d & fluFrdFlip()
{
  static const Eigen::Matrix3d S =
    Eigen::DiagonalMatrix<double, 3>(1.0, -1.0, -1.0).toDenseMatrix();
  return S;
}

/// 차체프레임 벡터(자이로 각속도, 휠 twist 등)의 FLU <-> FRD 변환.
inline Eigen::Vector3d flipVec(const Eigen::Vector3d & v)
{
  return fluFrdFlip() * v;
}

/// 출발·도착이 둘 다 차체계인 포즈 (x_gv 등 차체 절대포즈). T' = S T S
inline gtsam::Pose3 flipPoseBody(const gtsam::Pose3 & T)
{
  const Eigen::Matrix3d & S = fluFrdFlip();
  return gtsam::Pose3(
    gtsam::Rot3(S * T.rotation().matrix() * S),
    gtsam::Point3(S * T.translation()));
}

/// 항공 표준 ZYX(yaw-pitch-roll) 오일러 -> 회전행렬 [rad].
/// 반환값은 '마운트 축을 차체 축으로 옮기는' 행렬 (열이 마운트 축의 차체좌표 표현).
inline Eigen::Matrix3d rpyToMatrix(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

/// 카메라 광학(RDF) 축 -> 차체(FRD) 축 정렬 회전.
///   광학 x(우) -> 차체 +y,  광학 y(하) -> 차체 +z,  광학 z(전) -> 차체 +x
inline Eigen::Matrix3d opticalToBodyFrd()
{
  Eigen::Matrix3d R;
  R << 0, 0, 1,
    1, 0, 0,
    0, 1, 0;
  return R;
}

/// 차체(FRD) -> 카메라 광학(RDF) 외부파라미터 x_vi.
///   rpy_deg : 마운트 자세 [roll, pitch, yaw] deg. 하방 틸트가 음수다.
///   xyz     : 레버암 (차체원점 -> 카메라 광학원점) [m], 차체 FRD.
inline gtsam::Pose3 makeCamExtrinsic(const double rpy_deg[3], const double xyz[3])
{
  constexpr double D2R = M_PI / 180.0;
  const Eigen::Matrix3d R_mount =
    rpyToMatrix(rpy_deg[0] * D2R, rpy_deg[1] * D2R, rpy_deg[2] * D2R);
  return gtsam::Pose3(
    gtsam::Rot3(R_mount * opticalToBodyFrd()),
    gtsam::Point3(xyz[0], xyz[1], xyz[2]));
}

}  // namespace rgbd_slam
