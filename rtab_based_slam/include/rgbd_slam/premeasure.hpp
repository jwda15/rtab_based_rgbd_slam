#pragma once

// 시작 직후 정지 구간에서 재는 사전 측정 (ROS 무관).
//   - 자이로 바이어스 (gyro)  : 정지 동안 자이로 평균
//   - 카메라 장착 (mount)      : 정지 동안 depth 에 바닥 평면 하나만 보이면 roll, pitch, 카메라 높이
//
// 규칙
//   첫 판정 틱(휠 수신 시작)부터 still_sec 동안 정지(|v| ≤ max_v ∧ |ω| < max_w)가 이어지면 측정을 확정한다.
//   그 전에 한 틱이라도 움직이면 둘 다 포기하고 사전값을 쓴다. 판정은 시작 직후 한 번뿐이다.
//   장착은 평면이 "하나뿐"일 때만 쓴다: 처리한 모든 프레임의 평면 인라이어 비율 ≥ inlier_min,
//   프레임 간 법선·높이 흔들림이 작고, 사전값과의 차이가 허용 범위 안일 것. 아니면 사전값 유지.
//   평면으로는 roll, pitch, 높이만 보인다. lever x, y 와 yaw 는 사전값 그대로다.

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace rgbd_slam
{

class Premeasure
{
public:
  struct Params
  {
    // --- slam_node 가 param.yaml 값으로 채운다 (여기 기본값은 '꺼짐' 상태일 뿐) ---
    bool gyro = false;               ///< 자이로 바이어스 측정
    bool mount = false;              ///< 카메라 roll·pitch·높이 측정
    double still_sec = 0.0;          ///< 필요한 정지 시간 [s]
    double max_v = 0.0;              ///< 정지 판정 휠 속도 [m/s]
    double max_w = 0.0;              ///< 정지 판정 각속도 [rad/s] (바이어스 빼기 전)
    double inlier_thr = 0.0;         ///< 평면 거리 임계 [m]
    double inlier_min = 0.0;         ///< 프레임마다 인라이어 비율 하한 — 하나라도 미달이면 단일 평면 아님
    double base_height = 0.0;        ///< 차체원점의 지면 높이 [m]
    double max_dev_deg = 0.0;        ///< 사전값과 roll·pitch 차이 허용 (0 = 검사 안 함)
    double max_dev_z = 0.0;          ///< 사전값과 lever z 차이 허용 [m] (0 = 검사 안 함)
    double prior_roll_deg = 0.0, prior_pitch_deg = 0.0, prior_lever_z = 0.0;   ///< 사전값 (차체 FRD)

    // --- 내부 상수 ---
    double zmin = 0.3, zmax = 3.0;   ///< 쓰는 depth 범위 [m]
    int pixel_step = 4;              ///< 픽셀 간격
    int ransac_iters = 300;
    int min_frames = 10;             ///< 확정에 필요한 프레임 수
    int max_frames = 30;             ///< 정지 구간에 고르게 이만큼만 처리
    double max_spread_deg = 1.0;     ///< 프레임 간 법선 흔들림 허용
    double max_spread_h = 0.01;      ///< 프레임 간 높이 흔들림 허용 [m]
  };

  struct Result
  {
    bool gyro_ok = false;
    Eigen::Vector3d gyro_bias = Eigen::Vector3d::Zero();   ///< [rad/s] 차체 FRD
    bool mount_ok = false;
    double roll_deg = 0.0, pitch_deg = 0.0;   ///< cam_rpy 에 넣을 값 (차체 FRD, 하방 = pitch 음수)
    double lever_z = 0.0;                     ///< lever z (차체 FRD, 위쪽 = 음수)
    double height = 0.0;                      ///< 카메라 → 바닥 수직거리 [m]
    double inlier_mean = 0.0, inlier_min = 0.0;
    int frames = 0;
    std::vector<std::string> log;             ///< 사람이 읽을 판정 기록
  };

  Premeasure();
  explicit Premeasure(const Params & p);

  /// 측정 중인가 (켜져 있고 아직 끝나지 않음)
  bool running() const {return (p_.gyro || p_.mount) && !done_;}

  /// IMU 틱마다. w_raw = 바이어스 빼기 전 각속도(FRD), v = 휠 전진속도.
  /// 이번 호출로 측정이 끝났으면(확정이든 포기든) true.
  bool addImu(double t, const Eigen::Vector3d & w_raw, double v, bool wheel_fresh);

  /// depth 프레임마다. enc = "16UC1"(mm) 또는 "32FC1"(m), K = camera_info.k (3x3 행 우선).
  void addDepth(double t, const uint8_t * data, int width, int height, int step,
                const std::string & enc, const double K[9]);

  bool wantsDepth() const {return p_.mount && !done_;}
  const Result & result() const {return res_;}
  const Params & params() const {return p_;}

private:
  struct Plane {Eigen::Vector3d n; double d; double ratio;};
  bool fitPlane(const std::vector<Eigen::Vector3d> & P, Plane & out);
  void finish(double t);

  Params p_;
  Result res_;
  bool done_ = false;
  double t0_ = -1.0;                 ///< 판정 시작 시각
  Eigen::Vector3d w_sum_ = Eigen::Vector3d::Zero();
  int w_n_ = 0;
  double last_depth_t_ = -1.0;
  int depth_seen_ = 0;
  std::vector<Plane> planes_;
  std::mt19937 rng_{0};
};

}  // namespace rgbd_slam
