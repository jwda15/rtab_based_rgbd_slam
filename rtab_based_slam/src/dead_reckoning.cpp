#include "rgbd_slam/dead_reckoning.hpp"

#include <gtsam/geometry/Rot3.h>

namespace rgbd_slam
{

Eigen::Matrix3d so3Project(const Eigen::Matrix3d & R)
{
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d Rp = svd.matrixU() * svd.matrixV().transpose();
  if (Rp.determinant() < 0) {                 // 반사 성분 제거
    Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
    D(2, 2) = -1.0;
    Rp = svd.matrixU() * D * svd.matrixV().transpose();
  }
  return Rp;
}

Eigen::Matrix3d expmap(const Eigen::Vector3d & w_dt)
{
  return gtsam::Rot3::Expmap(w_dt).matrix();
}

gtsam::Pose3 propagateX(const gtsam::Pose3 & pose,
                        double v,
                        const Eigen::Matrix3d & R_abs,
                        double dt)
{
  const Eigen::Vector3d p_new =
    pose.translation() + R_abs * Eigen::Vector3d(v * dt, 0.0, 0.0);
  return gtsam::Pose3(gtsam::Rot3(R_abs), gtsam::Point3(p_new));
}

DeadReckoning::DeadReckoning(const gtsam::Pose3 & x_vi)
: x_vi_(x_vi),
  x_iv_(x_vi.inverse()),
  x_gv_(gtsam::Pose3()),
  x_dw_(gtsam::Pose3()),
  R_att_(Eigen::Matrix3d::Identity()),
  R_gv_(Eigen::Matrix3d::Identity()),
  dR_(Eigen::Matrix3d::Identity())
{
  x_gi_ = x_gv_.compose(x_vi_);
  x_dr_ = x_dw_.compose(x_vi_);
}

void DeadReckoning::step(double v, const Eigen::Vector3d & w, double dt)
{
  const Eigen::Matrix3d R_att_prev = R_att_;
  R_att_ = so3Project(R_att_ * expmap(w * dt));      // 무보정 자세
  dR_    = so3Project(R_att_prev.transpose() * R_att_);  // 이번 스텝 증분

  R_gv_  = so3Project(R_gv_ * dR_);                  // 보정 반영 자세

  x_dw_ = propagateX(x_dw_, v, R_att_, dt);          // 무보정 baseline
  x_gv_ = propagateX(x_gv_, v, R_gv_,  dt);          // 보정 트랙
  x_dr_ = x_dw_.compose(x_vi_);
  x_gi_ = x_gv_.compose(x_vi_);
}

void DeadReckoning::resetFromOptimized(const gtsam::Pose3 & x_gi_opt, double v, double dt)
{
  const gtsam::Pose3 T_v = x_gi_opt.compose(x_iv_);
  R_gv_ = so3Project(T_v.rotation().matrix() * dR_); // 최적화된 회전에서 재개
  x_gv_ = propagateX(T_v, v, R_gv_, dt);
  x_gi_ = x_gv_.compose(x_vi_);
}

}  // namespace rgbd_slam
