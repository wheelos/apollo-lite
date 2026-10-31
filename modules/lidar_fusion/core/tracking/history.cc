// Copyright 2026 WheelOS. All Rights Reserved.

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

#include "modules/lidar_fusion/core/tracking/temporal_internal.h"

namespace apollo {
namespace lidar_fusion {
using temporal_detail::CellKey;
using temporal_detail::CellState;
using temporal_detail::Mot;
using temporal_detail::Set;
using temporal_detail::TemporalPrior;
using temporal_detail::Track;
using temporal_detail::TransformBox;
using temporal_detail::V;
using temporal_detail::Vector;
using temporal_detail::Vote;

void StatefulFusion::Impl::Reset() {
  if (tracker) {
    tracker->Reset();
  }
  history.clear();
  votes.clear();
  last_regions.clear();
  active = false;
}

TemporalPrior StatefulFusion::Impl::Prior(double now,
                                          const Eigen::Isometry3d& t) const {
  TemporalPrior prior;
  if (!active || !config.temporal().feedback()) {
    return prior;
  }
  const auto inverse = t.inverse();
  for (const auto& track : tracker->tracks()) {
    const auto& p = *track.last_observation;
    if (p.unknown || track.hits < config.temporal().confirmation_hits() ||
        now - track.last_observed_time > config.temporal().lost_seconds()) {
      continue;
    }
    ObjectCandidate object;
    Box predicted = p.box;
    Set(track.state.mean.head<3>() +
            track.state.mean.tail<3>() * (now - track.state_time),
        predicted.mutable_center());
    *object.mutable_box() = TransformBox(predicted, inverse);
    object.set_track_id(track.id);
    const auto vote = votes.find(track.id);
    const auto cls = vote != votes.end() ? vote->second.current : CLASS_UNKNOWN;
    object.set_unified_class(cls);
    object.set_temporal_class(cls);
    object.set_class_conflict(p.conflict);
    object.set_classification_use(p.conflict || cls == CLASS_UNKNOWN
                                      ? IDENTITY_UNCERTAIN
                                      : p.classification_use);
    if (p.conflict) {
      object.add_reason(DET_CLASS_CONFLICT);
    }
    object.set_group(THING);
    object.set_orientation_reliable(false);
    prior.objects.push_back(std::move(object));
  }
  for (const auto& item : history) {
    if (!item.second.ground ||
        now - item.second.ground_stamp > config.temporal().history_seconds()) {
      continue;
    }
    const double r = config.resolution();
    Vec3 p;
    Set(inverse * Vector((item.first.first + 0.5) * r,
                         (item.first.second + 0.5) * r, item.second.ground_z),
        &p);
    prior.ground.push_back(p);
  }
  return prior;
}

void StatefulFusion::Impl::Map(const Eigen::Isometry3d& t, FusionOutput* out) {
  const auto& c = config.temporal();
  if (!c.static_map()) {
    return;
  }
  const double now = out->header().timestamp_sec();
  const double r = config.resolution();
  std::set<CellKey> dynamic;
  std::set<uint64_t> moving_ids;
  std::map<uint64_t, std::set<CellKey>> regions;
  std::map<uint32_t, uint64_t> object_tracks;
  std::set<uint64_t> point_supported_tracks;
  std::map<CellKey, std::set<uint64_t>> unknown_tracks;
  for (const auto& object : out->object()) {
    object_tracks.emplace(object.id(), object.track_id());
  }
  for (const auto& track : tracker->tracks()) {
    if (track.last_observation->unknown &&
        track.last_observation->point_supported_footprint) {
      point_supported_tracks.insert(track.id);
    }
  }
  for (const auto& unknown : out->unknown()) {
    if (!unknown.point_supported_footprint()) {
      continue;
    }
    for (uint32_t id : unknown.point_id()) {
      if (id >= static_cast<uint32_t>(out->point_evidence_size())) {
        continue;
      }
      const auto& p = out->point_evidence(id);
      if (p.finite()) {
        const CellKey cell{std::floor((p.position().x() - config.min_x()) / r),
                           std::floor((p.position().y() - config.min_y()) / r)};
        unknown_tracks[cell].insert(unknown.track_id());
      }
    }
  }
  auto key = [r](const Vector& p) {
    return CellKey{std::floor(p.x() / r), std::floor(p.y() / r)};
  };
  // Unknowns and unconfirmed tracks cannot become trusted static history.
  // Predicted moving regions also erase prior history to avoid trails.
  for (const auto& track : out->track()) {
    const bool moving = track.unknown() || !track.confirmed() ||
                        V(track.velocity()).norm() > c.dynamic_speed();
    if (moving) {
      moving_ids.insert(track.id());
      const auto previous = last_regions.find(track.id());
      if (previous != last_regions.end()) {
        dynamic.insert(previous->second.begin(), previous->second.end());
      }
    }
    if (point_supported_tracks.count(track.id()) != 0) {
      // A predicted extent cannot invent support in an unsampled interior.
      // Missed tracks retain their last actual footprint until expiry.
      const auto previous = last_regions.find(track.id());
      if (!track.observed() && previous != last_regions.end()) {
        regions[track.id()] = previous->second;
      }
      continue;
    }
    const auto& b = track.box();
    const Vector extent = Eigen::Quaterniond(b.qw(), b.qx(), b.qy(), b.qz())
                              .toRotationMatrix()
                              .cwiseAbs() *
                          V(b.size()) * 0.5;
    const Vector low = V(b.center()) - extent - Vector::Constant(r);
    const Vector high = V(b.center()) + extent + Vector::Constant(r);
    for (double x = std::max(config.min_x(), low.x());
         x <= std::min(config.max_x(), high.x()); x += r) {
      for (double y = std::max(config.min_y(), low.y());
           y <= std::min(config.max_y(), high.y()); y += r) {
        const auto k = key(t * Vector(x, y, b.center().z()));
        regions[track.id()].insert(k);
        if (moving) {
          dynamic.insert(k);
        }
      }
    }
  }
  for (const auto& cell : out->space_map().cell()) {
    const Vector base(config.min_x() + (cell.x_index() + 0.5) * r,
                      config.min_y() + (cell.y_index() + 0.5) * r,
                      cell.ground_z());
    const auto k = key(t * base);
    if (cell.occupancy_unknown()) {
      dynamic.insert(k);
    }
    const auto unknown = unknown_tracks.find(
        {static_cast<int>(cell.x_index()), static_cast<int>(cell.y_index())});
    if (unknown != unknown_tracks.end()) {
      for (uint64_t id : unknown->second) {
        regions[id].insert(k);
        dynamic.insert(k);
      }
    }
    // The filtered box can lag a moving observation. Mask its actual current
    // footprint too, rather than accumulating those points as static.
    for (uint32_t ref : cell.object_ref()) {
      const auto track = object_tracks.find(ref);
      if (track != object_tracks.end()) {
        regions[track->second].insert(k);
        if (moving_ids.count(track->second) != 0) {
          dynamic.insert(k);
        }
      }
    }
  }
  for (const auto& k : dynamic) {
    history.erase(k);
  }
  last_regions = std::move(regions);
  std::map<CellKey, SpaceCell> observations;
  for (const auto& cell : out->space_map().cell()) {
    const Vector base(config.min_x() + (cell.x_index() + 0.5) * r,
                      config.min_y() + (cell.y_index() + 0.5) * r,
                      cell.ground_z());
    const Vector odom = t * base;
    const auto k = key(odom);
    if (dynamic.count(k) != 0 || cell.occupancy_unknown() ||
        (cell.occupancy_det() && !cell.occupancy_obs())) {
      continue;
    }
    if (cell.occupancy_obs() || cell.traversability() == FREE ||
        cell.traversability() == FREE_SOFT || cell.ground_supported()) {
      auto& current = observations[k];
      current.set_occupancy_obs(current.occupancy_obs() ||
                                cell.occupancy_obs());
      if (cell.ground_supported()) {
        current.set_ground_supported(true);
        current.set_ground_z(odom.z());
      }
      if (cell.traversability() == FREE || cell.traversability() == FREE_SOFT) {
        current.set_traversability(FREE);
      }
    }
  }
  for (const auto& observation : observations) {
    auto& state = history[observation.first];
    const auto& cell = observation.second;
    const double delta = cell.occupancy_obs()            ? c.occupied_log_odds()
                         : cell.traversability() == FREE ? c.free_log_odds()
                                                         : 0.;
    state.log_odds =
        std::clamp(state.log_odds + delta, -c.max_log_odds(), c.max_log_odds());
    state.stamp = now;
    if (cell.ground_supported()) {
      state.ground = true;
      state.ground_z = cell.ground_z();
      state.ground_stamp = now;
    }
  }
  for (auto it = history.begin(); it != history.end();) {
    if (now - it->second.stamp > c.history_seconds()) {
      it = history.erase(it);
    } else {
      ++it;
    }
  }
  while (history.size() > c.max_history_cells()) {
    const auto oldest = std::min_element(
        history.begin(), history.end(), [](const auto& a, const auto& b) {
          return a.second.stamp < b.second.stamp;
        });
    history.erase(oldest);
  }
  for (const auto& entry : history) {
    auto* cell = out->add_history_cell();
    cell->set_x_index(entry.first.first);
    cell->set_y_index(entry.first.second);
    cell->set_log_odds(entry.second.log_odds);
    cell->set_last_observed_stamp(entry.second.stamp);
    if (entry.second.ground &&
        now - entry.second.ground_stamp <= c.history_seconds()) {
      cell->set_ground_z(entry.second.ground_z);
    }
  }
}

}  // namespace lidar_fusion
}  // namespace apollo
