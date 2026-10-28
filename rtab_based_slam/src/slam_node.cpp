// rgbd_slam slam_node — DR(휠+자이로) + rtabmap 링크(VO·루프) 팩터그래프 (GTSAM)
//
// 입력 : 휠 오돔(전진속도), IMU(자이로), rtabmap /mapGraph(키프레임 링크) + /info(키프레임 시각),
//        rgbd_odometry /vo/odom (로스트 감지 로그용)
// 출력 : /fused_odom·/current_pose (매 IMU 틱), /fused_path (배치 최적화 때), 종료 시 콘솔 요약
//
// IMU 틱마다 DR 을 적분하고 graph_idx 틱마다 그래프 노드를 만든다. 팩터는 셋:
//   DR(항상) / VO = rtabmap 이웃링크(type 0) / 루프 = rtabmap 루프링크(type 1, 잔차 게이트 통과분)
// rtabmap 의 최적화 포즈는 쓰지 않는다.
//
// 파라미터는 전부 config/param.yaml 에서 온다 (코드에 기본값 없음 — 빠지면 시작 시 에러).
// 좌표계 (include/rgbd_slam/frames.hpp): 내부는 차체 FRD, 글로벌 = 시작 시점 차체 자세.
//   광학프레임(RDF)은 rtabmap 것 그대로. enu_input = 입력이 ENU/FLU, enu_output = 출력을 ENU/FLU 로.

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rtabmap_msgs/msg/map_graph.hpp>
#include <rtabmap_msgs/msg/info.hpp>

#include <gtsam/base/Lie.h>

#include "rgbd_slam/msg/current_pose.hpp"

#include "rgbd_slam/dead_reckoning.hpp"
#include "rgbd_slam/frames.hpp"
#include "rgbd_slam/graph_optimizer.hpp"
#include "rgbd_slam/loop_source.hpp"
#include "rgbd_slam/premeasure.hpp"

namespace rgbd_slam
{

constexpr double D2R = M_PI / 180.0;
constexpr double R2D = 180.0 / M_PI;

static double stampSec(const builtin_interfaces::msg::Time & t)
{
  return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

static gtsam::Pose3 toPose3(const geometry_msgs::msg::Transform & t)
{
  const gtsam::Rot3 R = gtsam::Rot3::Quaternion(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z);
  return gtsam::Pose3(R, gtsam::Point3(t.translation.x, t.translation.y, t.translation.z));
}

static geometry_msgs::msg::Pose toPoseMsg(const gtsam::Pose3 & T)
{
  const auto q = T.rotation().toQuaternion();
  geometry_msgs::msg::Pose p;
  p.position.x = T.x();
  p.position.y = T.y();
  p.position.z = T.z();
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();
  return p;
}

class SlamNode : public rclcpp::Node
{
public:
  SlamNode()
  : rclcpp::Node("slam_node")
  {
    loadParams();
    initGraph();
    subscribe();
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>("/fused_path", 1);
    pub_pose_ = create_publisher<rgbd_slam::msg::CurrentPose>("/current_pose", 10);
    report();
  }

  ~SlamNode() {dump();}

private:
  template<typename T>
  T param(const std::string & name) {return declare_parameter<T>(name);}   // param.yaml 필수

  void loadParams();
  void subscribe();
  void report();                         ///< 시작 시 좌표계·장착 해석을 찍는다
  void initGraph();                      ///< cam_rpy_·lever_ 로 외부파라미터·DR·최적화기를 (다시) 만든다
  void applyPremeasure();                ///< 사전 측정 결과 적용 (끝난 틱에 한 번)
  void onImu(sensor_msgs::msg::Imu::SharedPtr m);
  void onVoOdom(nav_msgs::msg::Odometry::SharedPtr m);
  void makeNode(double t_now);
  void addLoopFactor(int from, int to, const gtsam::Pose3 & dT, int rtab_from, int rtab_to);
  void runOptimize(double t_now);        ///< 대기 중인 팩터가 있으면 최적화하고 x_hat_ 갱신
  void publishFused(const builtin_interfaces::msg::Time & stamp);
  void publishPath(const builtin_interfaces::msg::Time & stamp);
  void dump();                           ///< 종료 요약 (콘솔)

  gtsam::Pose3 poseAtTime(double s, int last) const;   ///< x_hat_ 를 노드 사이에서 보간
  double pathLength(int a, int b) const;               ///< x_hat_ 위치 경로길이 [a,b]
  /// 키프레임 시각(tA, tB)에 잰 상대포즈 dT 를 노드(from, to) 시각으로 옮긴다 (끝점당 ≤50 ms 보간):
  ///   T_{from<-to} = (x_hat[from]^-1 P(tA)) dT (P(tB)^-1 x_hat[to])
  gtsam::Pose3 carryToNodes(int from, int to, double tA, double tB, const gtsam::Pose3 & dT, int last) const;
  double drGapBetween(int from, int to) const;   ///< 두 노드 사이에서 VO 없이 DR 로만 간 거리 [m]
  double loopAllow(int from, int to) const;      ///< 루프 잔차 허용치 [m] (게이트 꺼짐이면 inf)
  gtsam::Pose3 outBody(const gtsam::Pose3 & T) const {return enu_output_ ? flipPoseBody(T) : T;}

  // --- 파라미터 (param.yaml) ---
  std::string imu_topic_, wheel_topic_, vo_topic_, depth_topic_, cinfo_topic_, map_frame_, base_frame_;
  std::vector<double> cam_rpy_, lever_;  ///< 차체 FRD 장착 [deg], [m]
  bool enu_input_, enu_output_;          ///< 입력·출력이 ENU/FLU 인가 (false = FRD)
  Eigen::Vector3d gyro_bias_;            ///< [rad/s] 차체 FRD
  NoiseParams np_;
  LoopSource::Params lp_;
  double loop_max_err_, loop_err_per_m_, loop_err_per_m_lost_, loop_max_rot_deg_, loop_huber_k_;
  bool loop_robust_;
  bool use_vo_, use_loops_;              ///< 팩터 on/off (DR 은 항상)
  int graph_idx_;
  double wheel_timeout_, max_gap_;

  // --- 상태 ---
  Premeasure pm_;                        ///< 시작 직후 자이로 바이어스·카메라 장착 측정
  std::unique_ptr<DeadReckoning> dr_;
  std::unique_ptr<GraphOptimizer> opt_;
  LoopSource ls_;
  gtsam::Pose3 x_vi_;                    ///< 차체 -> 카메라 광학
  double K_[9] = {0};                    ///< camera_info (mount_calib 용)
  bool have_K_ = false;
  double v_ = 0.0;                       ///< 최신 휠 전진속도
  double wheel_t_last_ = -1.0;
  double t_prev_ = -1.0;                 ///< 직전 IMU 시각
  int64_t tick_ = 0;
  int last_node_ = 1;                    ///< 마지막으로 만든 노드 번호 (1-base)
  bool pose_update_ = false;             ///< 최적화 결과를 DR 에 반영할 차례

  std::vector<double> node_time_;        ///< 1-base 노드 시각
  std::vector<gtsam::Pose3> x_hat_;      ///< 1-base 최적화 궤적 (글로벌 -> 광학)
  std::vector<gtsam::Pose3> dr_save_;    ///< 1-base 무보정 DR (요약 비교용)
  std::vector<double> dr_since_vo_;      ///< 노드별 '마지막 VO 엣지 이후 경로길이' [m]
  int last_vo_node_ = 0;
  bool vo_lost_now_ = false;
  double vo_lost_t0_ = 0.0, vo_lost_time_ = 0.0;

  // --- 통계 ---
  int n_vo_ = 0, n_loop_ = 0, n_loop_rejected_ = 0;
  int64_t n_imu_ = 0, n_gap_ = 0, n_imu_ooo_ = 0, n_wheel_ = 0, n_wheel_stale_ = 0;
  int64_t n_odom_ = 0, n_odom_lost_ = 0, vo_lost_frames_ = 0, n_vo_lost_events_ = 0;
  double gap_sum_ = 0.0, odom_t0_ = -1.0, odom_t1_ = -1.0;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wheel_, sub_vo_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_cinfo_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapGraph>::SharedPtr sub_graph_;
  rclcpp::Subscription<rtabmap_msgs::msg::Info>::SharedPtr sub_info_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<rgbd_slam::msg::CurrentPose>::SharedPtr pub_pose_;
};

// ===================================================================== 초기화

void SlamNode::loadParams()
{
  // 0. 입력 토픽
  imu_topic_   = param<std::string>("imu_topic");
  wheel_topic_ = param<std::string>("wheel_odom_topic");
  param<std::string>("rgb_topic");                       // slam_node 는 안 씀 (launch 가 rtabmap 쪽에 넘김)
  depth_topic_ = param<std::string>("depth_topic");      // mount_calib 때만 씀
  cinfo_topic_ = param<std::string>("camera_info_topic");
  vo_topic_    = param<std::string>("vo_odom_topic");    // launch 가 넣는다 (/vo/odom)

  // 1. 차체 <-> 카메라
  cam_rpy_ = param<std::vector<double>>("cam_rpy_deg");
  lever_   = param<std::vector<double>>("lever_arm");
  if (cam_rpy_.size() != 3) {throw std::runtime_error("cam_rpy_deg 는 [roll, pitch, yaw] 3개");}
  if (lever_.size() != 3) {throw std::runtime_error("lever_arm 은 [x, y, z] 3개");}
  const double base_height = param<double>("base_height");

  // 2. 입력 / 출력 좌표계
  enu_input_  = param<bool>("enu_input");
  enu_output_ = param<bool>("enu_output");

  // 3. 초반 자동 측정
  Premeasure::Params pp;
  pp.gyro        = param<bool>("gyro_bias_calib");
  pp.mount       = param<bool>("mount_calib");
  pp.still_sec   = param<double>("premeasure_sec");
  pp.max_v       = param<double>("premeasure_max_v");
  pp.max_w       = param<double>("premeasure_max_w_deg") * D2R;
  pp.inlier_min  = param<double>("mount_inlier_min");
  pp.inlier_thr  = param<double>("mount_inlier_thr");
  pp.max_dev_deg = param<double>("mount_max_dev_deg");
  pp.max_dev_z   = param<double>("mount_max_dev_z");
  pp.base_height = base_height;
  pp.prior_roll_deg  = cam_rpy_[0];
  pp.prior_pitch_deg = cam_rpy_[1];
  pp.prior_lever_z   = lever_[2];
  pm_ = Premeasure(pp);
  const auto gb = param<std::vector<double>>("gyro_bias");
  if (gb.size() != 3) {throw std::runtime_error("gyro_bias 는 [x, y, z] 3개");}
  gyro_bias_ << gb[0], gb[1], gb[2];

  // 4. 팩터별 노이즈 [roll, pitch, yaw (deg), x, y, z (m)]
  auto sigma = [this](const std::string & name) {
      const auto v = param<std::vector<double>>(name);
      if (v.size() != 6) {throw std::runtime_error(name + " 는 [roll, pitch, yaw (deg), x, y, z (m)] 6개");}
      return std::array<double, 6>{v[0] * D2R, v[1] * D2R, v[2] * D2R, v[3], v[4], v[5]};
    };
  np_.prior     = sigma("noise_prior");
  np_.odometry  = sigma("noise_odometry");
  np_.vo        = sigma("noise_vo");
  np_.loop_bow  = sigma("noise_loop");
  np_.loop_prox = np_.loop_bow;          // 근접 루프(type 2)는 안 쓴다

  // 5. 팩터 on/off (DR 은 그래프의 뼈대라 항상 켜져 있다)
  use_vo_    = param<bool>("use_vo");
  use_loops_ = param<bool>("use_loops");

  // 6. 루프 잔차 게이트
  loop_max_err_        = param<double>("loop_max_error");
  loop_err_per_m_      = param<double>("loop_err_per_m");
  loop_err_per_m_lost_ = param<double>("loop_err_per_m_lost");
  loop_max_rot_deg_    = param<double>("loop_max_rot_deg");
  loop_robust_         = param<bool>("loop_robust");
  loop_huber_k_        = param<double>("loop_huber_k");
  lp_.min_gap          = param<int>("loop_min_node_gap");
  lp_.loop_min_id_gap  = param<int>("loop_min_id_gap");

  // 7. 그래프 / 입력 처리
  graph_idx_     = param<int>("graph_idx");
  lp_.max_dt     = param<double>("kf_max_dt");
  wheel_timeout_ = param<double>("wheel_timeout");
  max_gap_       = param<double>("max_imu_gap");
  map_frame_     = param<std::string>("map_frame");
  base_frame_    = param<std::string>("base_frame");
  if (graph_idx_ <= 0) {throw std::runtime_error("graph_idx 는 1 이상");}

  lp_.use_vo = use_vo_;
  lp_.use_loops = use_loops_;
  lp_.use_prox = false;          // rtabmap 근접 루프(type 2)는 안 쓴다 (launch 에서도 ProximityBySpace false)
  ls_.setParams(lp_);
  node_time_.assign(2, 0.0);     // 1-base (인덱스 1 = 첫 노드)
}

void SlamNode::subscribe()
{
  const auto qos_imu = rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();
  const auto qos_odom = rclcpp::QoS(rclcpp::KeepLast(500)).best_effort();

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic_, qos_imu, std::bind(&SlamNode::onImu, this, std::placeholders::_1));

  sub_wheel_ = create_subscription<nav_msgs::msg::Odometry>(
    wheel_topic_, qos_odom,
    [this](nav_msgs::msg::Odometry::SharedPtr m) {
      Eigen::Vector3d vb(m->twist.twist.linear.x, m->twist.twist.linear.y, m->twist.twist.linear.z);
      if (enu_input_) {vb = flipVec(vb);}
      const double ts = stampSec(m->header.stamp);
      if (ts < wheel_t_last_) {return;}          // 역순 메시지
      v_ = vb.x();
      wheel_t_last_ = ts;
      ++n_wheel_;
    });

  sub_vo_ = create_subscription<nav_msgs::msg::Odometry>(
    vo_topic_, qos_odom, std::bind(&SlamNode::onVoOdom, this, std::placeholders::_1));

  // rtabmap: /mapGraph = 키프레임 링크 (type 0 이웃, 1 루프), /info = 키프레임 id -> 시각
  sub_graph_ = create_subscription<rtabmap_msgs::msg::MapGraph>(
    "/mapGraph", 10,
    [this](rtabmap_msgs::msg::MapGraph::SharedPtr m) {
      for (const auto & L : m->links) {ls_.addLink(L.from_id, L.to_id, L.type, toPose3(L.transform));}
    });
  sub_info_ = create_subscription<rtabmap_msgs::msg::Info>(
    "/info", 50,
    [this](rtabmap_msgs::msg::Info::SharedPtr m) {ls_.addNodeStamp(m->ref_id, stampSec(m->header.stamp));});

  // 장착 측정용 depth — mount_calib 일 때만. 측정이 끝나면 applyPremeasure 가 구독을 푼다
  if (pm_.wantsDepth()) {
    sub_cinfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      cinfo_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::SharedPtr m) {
        std::copy(m->k.begin(), m->k.end(), K_);
        have_K_ = true;
      });
    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::SharedPtr m) {
        if (!have_K_ || !pm_.wantsDepth()) {return;}
        pm_.addDepth(stampSec(m->header.stamp), m->data.data(), static_cast<int>(m->width),
                     static_cast<int>(m->height), static_cast<int>(m->step), m->encoding, K_);
      });
  }
}

void SlamNode::report()
{
  // 부호 실수는 안 죽고 궤적만 틀어지므로 광축 방향까지 찍어 눈으로 확인한다
  const Eigen::Vector3d z_opt = x_vi_.rotation().matrix().col(2);   // 광축의 차체좌표 표현
  const double down_deg = std::asin(std::clamp(z_opt.z(), -1.0, 1.0)) * R2D;
  const auto & pp = pm_.params();
  RCLCPP_INFO(get_logger(),
    "\n===== rgbd_slam =====\n"
    "graph_idx=%d  kf_max_dt=%.2f  잔차 게이트 %.2f m + %.2f m/m\n"
    "enu_input=%s / enu_output=%s (내부는 차체 FRD)\n"
    "카메라 rpy=[%.2f, %.2f, %.2f] deg  lever=[%.3f, %.3f, %.3f] m\n"
    "  → 광축이 차체좌표에서 (%.3f, %.3f, %.3f), 하향 %.2f deg\n"
    "자이로 바이어스(FRD) (%+.4f, %+.4f, %+.4f) deg/s\n"
    "사전 측정: 자이로=%s 장착=%s (시작 후 %.1f s 정지 시)\n"
    "팩터: DR=on VO=%s 루프=%s",
    graph_idx_, lp_.max_dt, loop_max_err_, loop_err_per_m_,
    enu_input_ ? "true" : "false", enu_output_ ? "true" : "false",
    cam_rpy_[0], cam_rpy_[1], cam_rpy_[2], lever_[0], lever_[1], lever_[2],
    z_opt.x(), z_opt.y(), z_opt.z(), down_deg,
    gyro_bias_.x() * R2D, gyro_bias_.y() * R2D, gyro_bias_.z() * R2D,
    pp.gyro ? "on" : "off", pp.mount ? "on" : "off", pp.still_sec,
    use_vo_ ? "on" : "off", use_loops_ ? "on" : "off");
}

void SlamNode::initGraph()
{
  const double rpy[3] = {cam_rpy_[0], cam_rpy_[1], cam_rpy_[2]};
  const double xyz[3] = {lever_[0], lever_[1], lever_[2]};
  x_vi_ = makeCamExtrinsic(rpy, xyz);
  dr_ = std::make_unique<DeadReckoning>(x_vi_);
  opt_ = std::make_unique<GraphOptimizer>(dr_->xGi(), np_);
  opt_->setLoopRobust(loop_robust_, loop_huber_k_);
  opt_->setBodyToSensor(x_vi_);          // DR 노이즈를 차체축 기준으로 준다

  // 이미 만든 노드(사전 측정 중엔 정지라 전부 시작 포즈)를 새 외부파라미터로 다시 세운다
  x_hat_.assign(last_node_ + 1, dr_->xGi());
  dr_save_.assign(last_node_ + 1, dr_->xDr());
  for (int k = 2; k <= last_node_; ++k) {opt_->addDrNode(k - 1, k, gtsam::Pose3(), dr_->xGi());}
  pose_update_ = false;
}

void SlamNode::applyPremeasure()
{
  const auto & r = pm_.result();
  for (const auto & s : r.log) {RCLCPP_INFO(get_logger(), "[사전측정] %s", s.c_str());}
  if (r.gyro_ok) {gyro_bias_ = r.gyro_bias;}
  if (r.mount_ok) {
    if (n_vo_ + n_loop_ > 0) {
      RCLCPP_WARN(get_logger(), "[사전측정] 측정 중 이미 VO·루프 엣지가 들어와 장착값은 적용하지 않음");
    } else {
      cam_rpy_[0] = r.roll_deg;
      cam_rpy_[1] = r.pitch_deg;
      lever_[2] = r.lever_z;
      initGraph();
      RCLCPP_INFO(get_logger(), "[사전측정] 장착 적용: cam_rpy=[%.2f, %.2f, %.2f] deg  lever=[%.3f, %.3f, %.4f] m (노드 %d개 재구성)",
        cam_rpy_[0], cam_rpy_[1], cam_rpy_[2], lever_[0], lever_[1], lever_[2], last_node_);
    }
  }
  sub_depth_.reset();       // 장착 측정이 끝났으니 depth 구독 해제
  sub_cinfo_.reset();
}

// ===================================================================== 입력

void SlamNode::onVoOdom(nav_msgs::msg::Odometry::SharedPtr m)
{
  // 로스트 감지 로그에만 쓴다 (그래프에는 rtabmap 이웃링크가 들어간다).
  // rgbd_odometry 는 로스트 시 항등포즈 + 공분산 9999 를 낸다.
  ++n_odom_;
  const double ts = stampSec(m->header.stamp);
  if (odom_t0_ < 0.0) {odom_t0_ = ts;}
  odom_t1_ = ts;
  const auto & c = m->pose.covariance;
  const bool lost = std::max({c[0], c[7], c[14]}) >= 9000.0;
  if (lost && !vo_lost_now_) {
    vo_lost_t0_ = ts;
    vo_lost_frames_ = 0;
    ++n_vo_lost_events_;
    RCLCPP_WARN(get_logger(), "[VO] 로스트 시작  t=%.2f (bag +%.1f s)  누적 %ld회",
      ts, ts - odom_t0_, n_vo_lost_events_);
  } else if (!lost && vo_lost_now_) {
    vo_lost_time_ += ts - vo_lost_t0_;
    RCLCPP_WARN(get_logger(), "[VO] 복귀         t=%.2f (bag +%.1f s)  로스트 %.2f s / %ld 프레임",
      ts, ts - odom_t0_, ts - vo_lost_t0_, vo_lost_frames_);
  }
  vo_lost_now_ = lost;
  if (lost) {++vo_lost_frames_; ++n_odom_lost_;}
}

void SlamNode::onImu(sensor_msgs::msg::Imu::SharedPtr m)
{
  const double t_now = stampSec(m->header.stamp);
  ++n_imu_;

  if (t_prev_ < 0.0) {                   // 첫 틱 = 1번 노드 시각
    t_prev_ = t_now;
    node_time_[1] = t_now;
    return;
  }
  const double dt = t_now - t_prev_;
  if (dt <= 0.0) {++n_imu_ooo_; return;}  // 역순·중복은 기준 시각을 안 건드리고 버린다
  t_prev_ = t_now;
  if (dt > max_gap_) {
    ++n_gap_;
    gap_sum_ += dt;
    if (dt > 3600.0) {   // 시계가 섞였다 — 거의 항상 이전 실행의 bag play 가 안 죽은 경우
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
        "[IMU] dt=%.0f s — 시계가 섞였다. %s 에 publisher 가 둘인지 확인 "
        "(ros2 topic info %s / pkill -9 -f 'bag play')", dt, imu_topic_.c_str(), imu_topic_.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[IMU] %.3f s 공백 — 이 구간 운동을 버림 (누적 %ld회 / %.2f s)", dt, n_gap_, gap_sum_);
    }
    return;
  }

  Eigen::Vector3d w_raw(m->angular_velocity.x, m->angular_velocity.y, m->angular_velocity.z);
  if (enu_input_) {w_raw = flipVec(w_raw);}          // 이 아래는 전부 차체 FRD

  // 휠 속도: 마지막 수신이 wheel_timeout 보다 오래면 0 으로 본다
  const bool wheel_fresh = (wheel_t_last_ >= 0.0 && t_now - wheel_t_last_ <= wheel_timeout_);
  const double v_use = wheel_fresh ? v_ : 0.0;
  if (!wheel_fresh && wheel_t_last_ >= 0.0) {
    ++n_wheel_stale_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "[휠] %.2f s 무수신 — 속도 0 으로 간주 (누적 %ld틱)", t_now - wheel_t_last_, n_wheel_stale_);
  }

  // 사전 측정 (자이로 바이어스·카메라 장착). 끝난 틱에 결과를 적용한다.
  if (pm_.running() && pm_.addImu(t_now, w_raw, v_, wheel_fresh)) {applyPremeasure();}

  dr_->step(v_use, w_raw - gyro_bias_, dt);
  if (pose_update_) {
    pose_update_ = false;
    dr_->resetFromOptimized(x_hat_[last_node_], v_use, dt);
  }

  ++tick_;
  if (tick_ % graph_idx_ == 0) {makeNode(t_now);}
  publishFused(m->header.stamp);
}

// ===================================================================== 그래프

void SlamNode::makeNode(double t_now)
{
  const int a = last_node_, b = last_node_ + 1;

  // 노드 등록 + DR 팩터
  node_time_.resize(b + 1);
  x_hat_.resize(b + 1);
  dr_save_.resize(b + 1);
  node_time_[b] = t_now;
  x_hat_[b] = dr_->xGi();
  dr_save_[b] = dr_->xDr();
  const gtsam::Pose3 dT = x_hat_[a].between(dr_->xGi());
  opt_->addDrNode(a, b, dT, x_hat_[a].compose(dT));

  // VO 엣지 = rtabmap 이웃링크. 키프레임 시각 → 노드 시각 보정 후 넣는다.
  for (const auto & e : ls_.harvestVo(node_time_, b)) {
    opt_->addVo(e.from, e.to, carryToNodes(e.from, e.to, e.from_stamp, e.to_stamp, e.dT, b));
    ++n_vo_;
    last_vo_node_ = std::max(last_vo_node_, e.to);
  }

  // 루프 엣지 = rtabmap 루프링크
  for (const auto & L : ls_.harvestLoop(node_time_, b)) {
    addLoopFactor(L.from, L.to, carryToNodes(L.from, L.to, L.from_stamp, L.to_stamp, L.dT, b), L.from_id, L.to_id);
  }

  last_node_ = b;
  // 마지막 VO 엣지 이후 경로길이 (루프 잔차 허용의 VO 공백 항에 쓴다)
  dr_since_vo_.resize(b + 1, 0.0);
  dr_since_vo_[b] = pathLength(std::max(1, last_vo_node_), b);

  runOptimize(t_now);
}

void SlamNode::addLoopFactor(int from, int to, const gtsam::Pose3 & dT, int rtab_from, int rtab_to)
{
  // 잔차 게이트: 현재 추정과 너무 어긋난 루프는 버린다
  const gtsam::Pose3 res = x_hat_[from].between(x_hat_[to]).between(dT);
  const double e_t = res.translation().norm();
  const double e_r = gtsam::Rot3::Logmap(res.rotation()).norm() * R2D;
  const double allow = loopAllow(from, to);
  if (e_t > allow || (loop_max_rot_deg_ > 0.0 && e_r > loop_max_rot_deg_)) {
    ++n_loop_rejected_;
    RCLCPP_WARN(get_logger(), "[루프거부] 노드 %d<->%d (rtab %d<->%d) 잔차 %.3f m (허용 %.3f) 회전 %.1f deg (허용 %.1f)",
      from, to, rtab_from, rtab_to, e_t, allow, e_r, loop_max_rot_deg_);
    return;
  }
  opt_->addLoop(from, to, dT, 1);
  ++n_loop_;
  RCLCPP_INFO(get_logger(), "[루프] 노드 %d<->%d (rtab %d<->%d, 간격 %d)  |t|=%.3f m  잔차 %.3f m  누적 %d",
    from, to, rtab_from, rtab_to, to - from, dT.translation().norm(), e_t, n_loop_);
}

void SlamNode::runOptimize(double t_now)
{
  std::vector<gtsam::Pose3> poses;
  if (!opt_->pending() || !opt_->optimize(poses)) {return;}
  for (size_t k = 1; k < poses.size() && k < x_hat_.size(); ++k) {x_hat_[k] = poses[k];}
  pose_update_ = true;
  if (opt_->lastRunWasBatch()) {       // 루프로 배치 최적화가 돈 때만 경로 재발행
    builtin_interfaces::msg::Time st;
    st.sec = static_cast<int32_t>(t_now);
    st.nanosec = static_cast<uint32_t>((t_now - st.sec) * 1e9);
    publishPath(st);
  }
}

gtsam::Pose3 SlamNode::poseAtTime(double s, int last) const
{
  last = std::min(last, static_cast<int>(x_hat_.size()) - 1);
  if (last < 2 || s <= node_time_[1]) {return x_hat_[1];}
  if (s >= node_time_[last]) {return x_hat_[last];}
  const auto it = std::upper_bound(node_time_.begin() + 1, node_time_.begin() + last + 1, s);
  const int k = static_cast<int>(it - node_time_.begin()) - 1;      // node_time_[k] <= s < node_time_[k+1]
  if (k < 1) {return x_hat_[1];}
  if (k >= last) {return x_hat_[last];}
  const double den = node_time_[k + 1] - node_time_[k];
  const double a = den > 1e-9 ? (s - node_time_[k]) / den : 0.0;
  return gtsam::interpolate<gtsam::Pose3>(x_hat_[k], x_hat_[k + 1], a);
}

gtsam::Pose3 SlamNode::carryToNodes(int from, int to, double tA, double tB,
                                    const gtsam::Pose3 & dT, int last) const
{
  if (tA <= 0.0 || tB <= 0.0) {return dT;}
  if (from < 1 || to < 1 || from > last || to > last) {return dT;}
  const gtsam::Pose3 PA = poseAtTime(tA, last);
  const gtsam::Pose3 PB = poseAtTime(tB, last);
  return x_hat_[from].between(PA).compose(dT).compose(PB.between(x_hat_[to]));
}

double SlamNode::pathLength(int a, int b) const
{
  if (a > b) {std::swap(a, b);}
  double L = 0.0;
  for (int k = std::max(1, a); k < b && k + 1 < static_cast<int>(x_hat_.size()); ++k) {
    L += (x_hat_[k + 1].translation() - x_hat_[k].translation()).norm();
  }
  return L;
}

double SlamNode::drGapBetween(int from, int to) const
{
  // dr_since_vo_ 는 VO 공백 동안만 늘어나므로 그 증가분 합 = 구간 안에서 DR 로만 간 거리
  if (from > to) {std::swap(from, to);}
  const int n = static_cast<int>(dr_since_vo_.size());
  double g = 0.0;
  for (int k = std::max(2, from + 1); k <= to && k < n; ++k) {
    g += std::max(0.0, dr_since_vo_[k] - dr_since_vo_[k - 1]);
  }
  return g;
}

double SlamNode::loopAllow(int from, int to) const
{
  if (loop_max_err_ <= 0.0 && loop_err_per_m_ <= 0.0 && loop_err_per_m_lost_ <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return loop_max_err_ + loop_err_per_m_ * pathLength(from, to) +
         loop_err_per_m_lost_ * drGapBetween(from, to);
}

// ===================================================================== 출력

void SlamNode::publishFused(const builtin_interfaces::msg::Time & stamp)
{
  const gtsam::Pose3 T = outBody(dr_->xGv());     // 차체 절대포즈
  nav_msgs::msg::Odometry o;
  o.header.stamp = stamp;
  o.header.frame_id = map_frame_;
  o.child_frame_id = base_frame_;
  o.pose.pose = toPoseMsg(T);
  o.twist.twist.linear.x = v_;
  pub_odom_->publish(o);

  // 같은 포즈를 위치 m / 자세 deg (ZYX 오일러) 로
  const gtsam::Vector3 rpy = T.rotation().rpy();
  rgbd_slam::msg::CurrentPose cp;
  cp.header = o.header;
  cp.x = T.x();
  cp.y = T.y();
  cp.z = T.z();
  cp.roll  = rpy(0) * R2D;
  cp.pitch = rpy(1) * R2D;
  cp.yaw   = rpy(2) * R2D;
  pub_pose_->publish(cp);
}

void SlamNode::publishPath(const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = map_frame_;
  for (size_t k = 1; k < x_hat_.size(); ++k) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path.header;
    ps.pose = toPoseMsg(outBody(x_hat_[k].compose(dr_->xIv())));   // 광학 -> 차체
    path.poses.push_back(ps);
  }
  pub_path_->publish(path);
}

void SlamNode::dump()
{
  if (x_hat_.size() < 2) {return;}
  const size_t N = x_hat_.size();

  // 시작→종점 변위와 경로길이 (차체 원점)
  auto endErr = [&](const std::vector<gtsam::Pose3> & P, double & h, double & d, double & len) {
      const gtsam::Pose3 iv = dr_->xIv();
      const auto e = P.back().compose(iv).translation() - P[1].compose(iv).translation();
      h = std::hypot(e.x(), e.y());
      d = e.norm();
      len = 0.0;
      for (size_t k = 2; k < P.size(); ++k) {
        len += (P[k].compose(iv).translation() - P[k - 1].compose(iv).translation()).norm();
      }
    };
  double hf, df, lf, hd, dd, ld;
  endErr(x_hat_, hf, df, lf);
  endErr(dr_save_, hd, dd, ld);

  RCLCPP_INFO(get_logger(),
    "\n===== 시작→종점 변위 [차체 원점] =====\n"
    "(시작점으로 돌아오는 주행에서만 폐합 오차다)\n"
    "노드 %zu / VO엣지 %d / 루프엣지 %d\n"
    "DR    수평 %.3f m / 3D %.3f m   경로 %.3f m\n"
    "Fused 수평 %.3f m / 3D %.3f m   경로 %.3f m\n"
    "-- 입력 --\n"
    "IMU 수신 %ld  처리틱 %ld  공백폐기 %ld회/%.2fs  역순폐기 %ld\n"
    "휠 수신 %ld  무수신으로 v=0 %ld틱\n"
    "/odom 수신 %ld  그중 로스트 %ld (%.1f%%)  로스트 %ld회 / 누적 %.1f s\n"
    "-- rtabmap 링크 매핑 --\n"
    "이웃링크 받음 %zu -> 그래프 %d, 대기 %zu   루프 대기 %zu   루프 잔차거부 %d\n"
    "키프레임 %zu개 (id %d~%d), 노드로 매핑 %zu   링크 id %d~%d\n"
    "시각 없는 링크 id %zu   역행 %zu   타입거부 %zu   id간격거부 %zu   그외폐기 %zu",
    N - 1, n_vo_, n_loop_,
    hd, dd, ld, hf, df, lf,
    n_imu_, tick_, n_gap_, gap_sum_, n_imu_ooo_,
    n_wheel_, n_wheel_stale_,
    n_odom_, n_odom_lost_,
    n_odom_ ? 100.0 * static_cast<double>(n_odom_lost_) / static_cast<double>(n_odom_) : 0.0,
    n_vo_lost_events_, vo_lost_time_ + (vo_lost_now_ ? odom_t1_ - vo_lost_t0_ : 0.0),
    ls_.voLinksSeen(), n_vo_, ls_.pendingVo(), ls_.pendingLoop(), n_loop_rejected_,
    ls_.kfSeen(), ls_.kfIdMin(), ls_.kfIdMax(), ls_.kfMapped(), ls_.linkIdMin(), ls_.linkIdMax(),
    ls_.unmappedLinkIds(), ls_.nonMonotonic(), ls_.rejectedType(), ls_.rejectedIdGap(), ls_.droppedCount());
}

}  // namespace rgbd_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rgbd_slam::SlamNode>());
  rclcpp::shutdown();
  return 0;
}
