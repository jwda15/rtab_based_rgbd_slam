#include "rgbd_slam/loop_source.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rgbd_slam
{

static std::string linkKey(int a, int b, int t)
{
  return std::to_string(a) + "_" + std::to_string(b) + "_" + std::to_string(t);
}

static std::string pairKey(int a, int b)
{
  return std::to_string(std::min(a, b)) + "_" + std::to_string(std::max(a, b));
}

void LoopSource::addNodeStamp(int rtab_id, double stamp)
{
  if (rtab_id <= 0) {return;}
  // id 는 시간순으로 커져야 한다. 어기면 /info 를 stamp 원천으로 쓸 수 없다.
  if (rtab_id > last_id_) {
    if (last_id_ != 0 && stamp < last_t_) {++nonmono_;}
    last_id_ = rtab_id;
    last_t_ = stamp;
  }
  id2t_.emplace(rtab_id, stamp);      // 이미 있으면 덮지 않는다 (첫 보고 시각 유지)
  while (id2t_.size() > p_.id_table_max) {
    id2t_.erase(id2t_.begin());
  }
}

size_t LoopSource::unmappedLinkIds() const
{
  std::set<int> miss;
  for (const auto * pool : {&vo_links_, &loop_links_}) {
    for (const auto & kv : *pool) {
      if (!id2t_.count(kv.second.from_id)) {miss.insert(kv.second.from_id);}
      if (!id2t_.count(kv.second.to_id)) {miss.insert(kv.second.to_id);}
    }
  }
  return miss.size();
}

void LoopSource::addLink(int from_id, int to_id, int type, const gtsam::Pose3 & dT)
{
  if (link_id_min_ < 0) {link_id_min_ = std::min(from_id, to_id);}
  link_id_min_ = std::min({link_id_min_, from_id, to_id});
  link_id_max_ = std::max({link_id_max_, from_id, to_id});

  std::map<std::string, RawLink> * pool = nullptr;

  if (type == 0) {
    if (!p_.use_vo) {return;}
    pool = &vo_links_;
  } else if (type == 1 || type == 2) {
    // type 1/2 만 쓴다. 3=LocalTime, 4=User, 5=Virtual, 6=NeighborMerged,
    // 7=PosePrior, 8=Landmark, 9=Gravity 는 전부 버린다.
    if (!p_.use_loops) {return;}
    if (type == 2 && !p_.use_prox) {++rejected_type_; return;}
    if (p_.loop_min_id_gap > 0 &&
        std::abs(from_id - to_id) < p_.loop_min_id_gap)
    {
      ++rejected_idgap_;
      return;
    }
    pool = &loop_links_;
  } else {
    ++rejected_type_;
    return;
  }

  const std::string k = linkKey(from_id, to_id, type);
  const std::string kr = linkKey(to_id, from_id, type);
  if (emitted_.count(k) || emitted_.count(kr)) {return;}
  if (pool->count(k) || pool->count(kr)) {return;}   // 양방향 중복
  (*pool)[k] = RawLink{from_id, to_id, type, dT};
  if (type == 0) {++vo_seen_total_;}
}

void LoopSource::refreshMapping(const std::vector<double> & node_time, int odo_idx)
{
  if (odo_idx < 1 || odo_idx >= static_cast<int>(node_time.size())) {return;}
  const double t_last = node_time[odo_idx];
  const double t_first = node_time[1];

  for (const auto & kv : id2t_) {
    const int id = kv.first;
    const double t = kv.second;
    if (kf2node_.count(id)) {continue;}     // 이미 정해짐 — 다시 계산하지 않는다
    if (t > t_last) {continue;}             // 아직 시각이 안 지남 — 보류
    if (t <= t_first) {                     // 첫 노드보다 이른 키프레임(시작 정지구간)
      kf2node_[id] = 1;
      continue;
    }
    int best = 0;
    double bd = std::numeric_limits<double>::infinity();
    for (int n = 1; n <= odo_idx; ++n) {
      const double d = std::fabs(node_time[n] - t);
      if (d < bd) {bd = d; best = n;}
    }
    if (best != 0 && bd <= p_.max_dt) {kf2node_[id] = best;}
  }
}

std::vector<GraphEdge> LoopSource::harvest(std::map<std::string, RawLink> & pool,
                                           std::set<std::string> & usedpair,
                                           const std::vector<double> & node_time,
                                           int odo_idx,
                                           bool apply_min_gap)
{
  std::vector<GraphEdge> out;
  if (pool.empty() || odo_idx < 2) {return out;}

  struct Best
  {
    std::string key;
    double err;
    int na, nb;
  };
  std::map<std::string, Best> best;
  std::vector<std::string> done;

  // --- 1단계: 노드쌍별로 시간정합이 가장 좋은 링크만 남긴다 -------------
  for (const auto & kv : pool) {
    const RawLink & L = kv.second;

    const auto ia = kf2node_.find(L.from_id);
    const auto ib = kf2node_.find(L.to_id);
    if (ia == kf2node_.end() || ib == kf2node_.end()) {continue;}   // 아직 매핑 불가

    const int na = ia->second;
    const int nb = ib->second;
    if (na == nb) {done.push_back(kv.first); ++dropped_; continue;}
    if (apply_min_gap && p_.min_gap > 0 && std::abs(na - nb) < p_.min_gap) {
      done.push_back(kv.first); ++dropped_; continue;
    }

    const std::string pk = pairKey(na, nb);
    if (usedpair.count(pk)) {done.push_back(kv.first); ++dropped_; continue;}

    const double err = std::fabs(id2t_.at(L.from_id) - node_time[na]) +
      std::fabs(id2t_.at(L.to_id) - node_time[nb]);

    auto it = best.find(pk);
    if (it == best.end()) {
      best.emplace(pk, Best{kv.first, err, na, nb});
    } else if (it->second.err <= err) {
      done.push_back(kv.first); ++dropped_;                 // 기존 게 더 좋음
    } else {
      done.push_back(it->second.key); ++dropped_;           // 새 게 더 좋음
      it->second = Best{kv.first, err, na, nb};
    }
  }

  // --- 2단계: 살아남은 것만 방출 ---------------------------------------
  for (const auto & bkv : best) {
    const RawLink & L = pool.at(bkv.second.key);
    const int na = bkv.second.na;
    const int nb = bkv.second.nb;

    GraphEdge e;
    e.type = L.type;
    e.t_err = bkv.second.err;
    if (na <= nb) {
      e.from = na; e.to = nb; e.dT = L.dT;
      e.from_id = L.from_id; e.to_id = L.to_id;
    } else {                                    // rtabmap 은 from>to 로 저장하는 일이 많음
      e.from = nb; e.to = na; e.dT = L.dT.inverse();
      e.from_id = L.to_id; e.to_id = L.from_id;
    }
    out.push_back(e);
    out.back().from_stamp = id2t_.count(e.from_id) ? id2t_.at(e.from_id) : 0.0;
    out.back().to_stamp   = id2t_.count(e.to_id) ? id2t_.at(e.to_id) : 0.0;

    usedpair.insert(bkv.first);
    done.push_back(bkv.second.key);
  }

  for (const auto & k : done) {
    emitted_.insert(k);
    pool.erase(k);
  }
  return out;
}

std::vector<GraphEdge> LoopSource::harvestVo(const std::vector<double> & node_time, int odo_idx)
{
  refreshMapping(node_time, odo_idx);
  return harvest(vo_links_, usedpair_vo_, node_time, odo_idx, /*apply_min_gap=*/false);
}

std::vector<GraphEdge> LoopSource::harvestLoop(const std::vector<double> & node_time, int odo_idx)
{
  refreshMapping(node_time, odo_idx);
  return harvest(loop_links_, usedpair_loop_, node_time, odo_idx, /*apply_min_gap=*/true);
}

}  // namespace rgbd_slam
