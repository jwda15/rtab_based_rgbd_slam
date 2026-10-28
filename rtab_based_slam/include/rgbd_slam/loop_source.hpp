#pragma once

// rtabmap /mapGraph 링크를 우리 DR 노드 번호로 번역한다.
//   type 0     = Neighbor          -> VO 엣지
//   type 1     = GlobalClosure     -> 루프
//   type 2     = LocalSpaceClosure -> 근접 루프
//   그 외(3~9) = 버림
//
// rtabmap 키프레임 id 와 우리 노드 번호는 별개 체계라 시각(stamp)으로 잇는다.
//   /info     : 프레임마다 (키프레임 id, stamp) -> id2t_
//   /mapGraph : 링크 목록 (매번 전체 재발행)  -> emitted_ 로 링크당 1회만 방출
//
// ★ 매핑 불변조건 셋. 하나라도 어기면 링크가 중복되거나 엉뚱한 노드쌍에 붙는다.
//   1) 키프레임->노드 매핑은 한 번 정해지면 동결한다. 매번 최근접을 다시 구하면 아직
//      노드가 안 생긴 미래 키프레임이 마지막 노드에 붙었다가 나중에 또 붙는다.
//   2) 노드 시각을 아직 안 지난 키프레임은 매핑을 보류한다.
//   3) 한 노드쌍에는 링크 하나만. 후보를 모아 시간정합 오차가 최소인 것만 방출한다.

#include <map>
#include <set>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

namespace rgbd_slam
{

/// 우리 노드 번호로 번역된 그래프 엣지.
struct GraphEdge
{
  int from = 0;             ///< 우리 노드 번호 (1-base, GTSAM 키)
  int to = 0;
  gtsam::Pose3 dT;          ///< from -> to, 카메라 광학프레임
  int type = 0;             ///< 0=Neighbor(VO), 1=GlobalClosure, 2=LocalSpaceClosure
  int from_id = 0;          ///< 진단용 — 방향 정규화 후의 rtabmap 키프레임 id
  int to_id = 0;
  double t_err = 0.0;       ///< 진단용 — 키프레임↔노드 시간정합 오차 합 [s]
  /// 키프레임 시각. 노드 시각과 달라서 slam_node 가 dT 를 노드 시각으로 옮기는 데 쓴다. 0 이면 모름.
  double from_stamp = 0.0;
  double to_stamp = 0.0;
};

class LoopSource
{
public:
  /// slam_node 가 param.yaml 값으로 채운다.
  struct Params
  {
    double max_dt = 0.0;         ///< 키프레임 시각 <-> 노드 시각 허용차 [s] (kf_max_dt)
    int min_gap = 0;             ///< 루프 양끝 노드 간격 하한, 0 = 끔 (loop_min_node_gap)
    int loop_min_id_gap = 0;     ///< 루프 양끝 키프레임 id 간격 하한, 0 = 끔 (loop_min_id_gap)
    bool use_vo = true;          ///< type 0 을 VO 엣지로 쓸지
    bool use_loops = false;      ///< type 1 (use_loops)
    bool use_prox = false;       ///< type 2 (use_prox_loops)
    size_t id_table_max = 20000; ///< id->stamp 표 최대 크기 (내부 상수)
  };

  void setParams(const Params & p) {p_ = p;}

  /// /info 콜백 — rtabmap 키프레임 id 와 그 시각을 기록
  void addNodeStamp(int rtab_id, double stamp);

  /// /mapGraph 콜백 — 링크 적재. 타입/id 간격 필터가 여기서 걸린다
  void addLink(int from_id, int to_id, int type, const gtsam::Pose3 & dT);

  /// 방출 가능한 VO(type 0) 엣지. node_time 은 1-base, odo_idx 는 방금 만든 노드 번호
  std::vector<GraphEdge> harvestVo(const std::vector<double> & node_time, int odo_idx);

  /// 방출 가능한 루프(type 1/2) 엣지
  std::vector<GraphEdge> harvestLoop(const std::vector<double> & node_time, int odo_idx);

  // --- 진단 (종료 요약용) ---
  size_t pendingVo() const {return vo_links_.size();}
  size_t pendingLoop() const {return loop_links_.size();}
  size_t droppedCount() const {return dropped_;}
  size_t rejectedType() const {return rejected_type_;}
  size_t rejectedIdGap() const {return rejected_idgap_;}
  size_t kfSeen() const {return id2t_.size();}
  size_t kfMapped() const {return kf2node_.size();}
  size_t unmappedLinkIds() const;   ///< 대기 링크가 참조하는 id 중 /info 로 시각을 못 받은 것
  int kfIdMin() const {return id2t_.empty() ? -1 : id2t_.begin()->first;}
  int kfIdMax() const {return id2t_.empty() ? -1 : id2t_.rbegin()->first;}
  int linkIdMin() const {return link_id_min_;}
  int linkIdMax() const {return link_id_max_;}
  size_t voLinksSeen() const {return vo_seen_total_;}
  /// id 가 커졌는데 stamp 가 과거인 경우의 개수. 0 이 아니면 /info 의 id↔stamp 대응을 믿으면 안 된다.
  size_t nonMonotonic() const {return nonmono_;}

private:
  struct RawLink
  {
    int from_id, to_id, type;
    gtsam::Pose3 dT;
  };

  /// 아직 매핑 안 된 키프레임을 노드에 붙인다. 한 번 정해지면 다시 안 바꾼다
  void refreshMapping(const std::vector<double> & node_time, int odo_idx);

  std::vector<GraphEdge> harvest(std::map<std::string, RawLink> & pool,
                                 std::set<std::string> & usedpair,
                                 const std::vector<double> & node_time,
                                 int odo_idx,
                                 bool apply_min_gap);

  Params p_;
  std::map<int, double> id2t_;                   ///< rtabmap id -> stamp
  std::map<int, int> kf2node_;                   ///< rtabmap id -> 우리 노드 (동결)
  std::map<std::string, RawLink> vo_links_;      ///< 미방출 type 0
  std::map<std::string, RawLink> loop_links_;    ///< 미방출 type 1/2
  std::set<std::string> emitted_;
  std::set<std::string> usedpair_vo_;            ///< 노드쌍 중복 체크는 VO/루프 각각 따로
  std::set<std::string> usedpair_loop_;
  size_t dropped_ = 0;
  size_t rejected_type_ = 0;
  size_t rejected_idgap_ = 0;
  size_t vo_seen_total_ = 0;      ///< 적재된 distinct type 0 링크 수
  size_t nonmono_ = 0;
  int last_id_ = 0;
  double last_t_ = 0.0;
  int link_id_min_ = -1;
  int link_id_max_ = -1;
};

}  // namespace rgbd_slam
