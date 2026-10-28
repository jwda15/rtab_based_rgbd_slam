#include "rgbd_slam/premeasure.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace rgbd_slam
{

namespace
{
constexpr double R2D = 180.0 / M_PI;

std::string fmt(const char * f, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof(buf), f, ap);
  va_end(ap);
  return buf;
}
}  // namespace

Premeasure::Premeasure()
: Premeasure(Params()) {}

Premeasure::Premeasure(const Params & p)
: p_(p)
{
  planes_.reserve(static_cast<size_t>(std::max(1, p_.max_frames)));
}

bool Premeasure::addImu(double t, const Eigen::Vector3d & w_raw, double v, bool wheel_fresh)
{
  if (!running() || !wheel_fresh) {return false;}   // 휠이 들어오기 전엔 판정을 미룬다
  if (t0_ < 0.0) {t0_ = t;}

  const bool still = std::fabs(v) <= p_.max_v && w_raw.norm() < p_.max_w;
  if (!still) {
    res_.log.push_back(fmt("시작 %.1f s 만에 움직임 (필요 %.1f s, |v|=%.3f m/s |ω|=%.2f deg/s) — 사전값 유지",
      t - t0_, p_.still_sec, std::fabs(v), w_raw.norm() * R2D));
    done_ = true;
    return true;
  }
  w_sum_ += w_raw;
  ++w_n_;
  if (t - t0_ >= p_.still_sec) {
    finish(t);
    return true;
  }
  return false;
}

void Premeasure::addDepth(double t, const uint8_t * data, int width, int height, int step,
                          const std::string & enc, const double K[9])
{
  if (!wantsDepth() || t0_ < 0.0) {return;}           // 판정 시작 전·후의 프레임은 안 쓴다
  ++depth_seen_;
  if (static_cast<int>(planes_.size()) >= p_.max_frames) {return;}
  const double gap = p_.still_sec / std::max(1, p_.max_frames);   // 정지 구간에 고르게
  if (last_depth_t_ >= 0.0 && t - last_depth_t_ < gap) {return;}

  const bool u16 = (enc == "16UC1" || enc == "mono16");
  if (!u16 && enc != "32FC1") {
    if (depth_seen_ == 1) {res_.log.push_back("지원 안 하는 depth 인코딩: " + enc);}
    return;
  }
  const double fx = K[0], fy = K[4], cx = K[2], cy = K[5];
  if (fx <= 0.0 || fy <= 0.0) {return;}
  last_depth_t_ = t;

  std::vector<Eigen::Vector3d> P;
  P.reserve(static_cast<size_t>((width / p_.pixel_step + 1) * (height / p_.pixel_step + 1)));
  for (int r = 0; r < height; r += p_.pixel_step) {
    const uint8_t * row = data + static_cast<size_t>(r) * static_cast<size_t>(step);
    for (int c = 0; c < width; c += p_.pixel_step) {
      double z;
      if (u16) {
        uint16_t mm;
        std::memcpy(&mm, row + 2 * c, 2);
        z = mm * 1e-3;
      } else {
        float m;
        std::memcpy(&m, row + 4 * c, 4);
        z = m;
      }
      if (!std::isfinite(z) || z <= p_.zmin || z >= p_.zmax) {continue;}
      P.emplace_back((c - cx) * z / fx, (r - cy) * z / fy, z);
    }
  }
  if (P.size() < 500) {return;}   // 너무 가깝거나 멀어 점이 모자란 프레임

  Plane pl;
  if (fitPlane(P, pl)) {planes_.push_back(pl);}
}

bool Premeasure::fitPlane(const std::vector<Eigen::Vector3d> & P, Plane & out)
{
  const int N = static_cast<int>(P.size());
  std::uniform_int_distribution<int> pick(0, N - 1);
  int best = -1;
  Eigen::Vector3d bn;
  double bd = 0.0;
  for (int it = 0; it < p_.ransac_iters; ++it) {
    const Eigen::Vector3d & a = P[pick(rng_)], & b = P[pick(rng_)], & c = P[pick(rng_)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    const double nn = n.norm();
    if (nn < 1e-9) {continue;}
    n /= nn;
    const double d = -n.dot(a);
    int cnt = 0;
    for (const auto & q : P) {if (std::fabs(n.dot(q) + d) < p_.inlier_thr) {++cnt;}}
    if (cnt > best) {best = cnt; bn = n; bd = d;}
  }
  if (best < 3) {return false;}

  // 인라이어로 최소자승 재추정 (공분산 최소 고유벡터)
  Eigen::Vector3d c = Eigen::Vector3d::Zero();
  int m = 0;
  for (const auto & q : P) {if (std::fabs(bn.dot(q) + bd) < p_.inlier_thr) {c += q; ++m;}}
  c /= m;
  Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
  for (const auto & q : P) {
    if (std::fabs(bn.dot(q) + bd) < p_.inlier_thr) {C += (q - c) * (q - c).transpose();}
  }
  Eigen::Vector3d n = Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(C).eigenvectors().col(0);
  double d = -n.dot(c);
  if (d < 0.0) {n = -n; d = -d;}   // 카메라가 평면 위쪽(양의 거리)이 되게

  int in = 0;
  for (const auto & q : P) {if (std::fabs(n.dot(q) + d) < p_.inlier_thr) {++in;}}
  out = {n, d, static_cast<double>(in) / N};
  return true;
}

void Premeasure::finish(double t)
{
  done_ = true;
  const double dur = t - t0_;

  if (p_.gyro) {
    res_.gyro_bias = w_sum_ / static_cast<double>(w_n_);
    res_.gyro_ok = true;
    res_.log.push_back(fmt("자이로: 정지 %.1f s / %d 샘플 평균 (%+.4f, %+.4f, %+.4f) deg/s 적용",
      dur, w_n_, res_.gyro_bias.x() * R2D, res_.gyro_bias.y() * R2D, res_.gyro_bias.z() * R2D));
  }
  if (!p_.mount) {return;}

  const int F = static_cast<int>(planes_.size());
  res_.frames = F;
  auto fail = [&](const std::string & why) {res_.log.push_back("장착: " + why + " — 사전값 유지");};
  if (F < p_.min_frames) {
    fail(fmt("평면 프레임 %d개 < %d (depth 수신 %d — 토픽·범위 확인)", F, p_.min_frames, depth_seen_));
    return;
  }

  double rmin = 1.0, rsum = 0.0;
  Eigen::Vector3d nsum = Eigen::Vector3d::Zero();
  double hsum = 0.0;
  for (const auto & pl : planes_) {
    rmin = std::min(rmin, pl.ratio);
    rsum += pl.ratio;
    nsum += pl.n;
    hsum += pl.d;
  }
  res_.inlier_min = rmin;
  res_.inlier_mean = rsum / F;
  const Eigen::Vector3d n = nsum.normalized();
  const double h = hsum / F;
  double ang = 0.0, hdev = 0.0;
  for (const auto & pl : planes_) {
    ang = std::max(ang, std::acos(std::clamp(pl.n.dot(n), -1.0, 1.0)) * R2D);
    hdev = std::max(hdev, std::fabs(pl.d - h));
  }

  // 광학프레임(x 우, y 하, z 전)에서 본 바닥 법선 -> 마운트 각
  const double pitch_cam = std::asin(std::clamp(-n.z(), -1.0, 1.0)) * R2D;   // 광축 하향각
  const double roll_cam = std::atan2(n.x(), -n.y()) * R2D;
  res_.roll_deg = -roll_cam;
  res_.pitch_deg = -pitch_cam;
  res_.height = h;
  res_.lever_z = -(h - p_.base_height);

  const std::string meas = fmt("roll %+.2f pitch %+.2f deg, lever z %+.4f m (높이 %.4f m), "
    "인라이어 평균 %.1f%% 최소 %.1f%%, 프레임 %d개, 흔들림 %.2f deg / %.4f m",
    res_.roll_deg, res_.pitch_deg, res_.lever_z, h, 100 * res_.inlier_mean, 100 * rmin, F, ang, hdev);

  if (rmin < p_.inlier_min) {
    fail(fmt("단일 평면 아님 (인라이어 최소 %.1f%% < %.1f%%) [%s]", 100 * rmin, 100 * p_.inlier_min, meas.c_str()));
    return;
  }
  if (ang > p_.max_spread_deg || hdev > p_.max_spread_h) {
    fail(fmt("프레임 간 평면이 흔들림 [%s]", meas.c_str()));
    return;
  }
  const double dr = std::fabs(res_.roll_deg - p_.prior_roll_deg);
  const double dp = std::fabs(res_.pitch_deg - p_.prior_pitch_deg);
  const double dz = std::fabs(res_.lever_z - p_.prior_lever_z);
  if ((p_.max_dev_deg > 0.0 && (dr > p_.max_dev_deg || dp > p_.max_dev_deg)) ||
      (p_.max_dev_z > 0.0 && dz > p_.max_dev_z))
  {
    fail(fmt("사전값과 차이가 큼 (roll %.2f, pitch %.2f deg, z %.4f m) — 바닥이 아닌 평면일 수 있음 [%s]",
      dr, dp, dz, meas.c_str()));
    return;
  }
  res_.mount_ok = true;
  res_.log.push_back("장착: " + meas + " 적용");
}

}  // namespace rgbd_slam
