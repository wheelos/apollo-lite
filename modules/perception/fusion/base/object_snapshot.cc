#include "modules/perception/fusion/base/object_snapshot.h"

#include <cmath>

#include "cyber/common/log.h"
#include "modules/perception/base/object_pool_types.h"

namespace apollo {
namespace perception {
namespace fusion {

void CopySensorMeasurement(const SensorObjectConstPtr& source,
                           base::SensorObjectMeasurement* destination) {
  destination->sensor_id = source->GetSensorId();
  destination->timestamp = source->GetTimestamp();
  const auto object = source->GetBaseObject();
  destination->track_id = object->track_id;
  destination->center = object->center;
  destination->theta = object->theta;
  destination->size = object->size;
  destination->velocity = object->velocity;
  destination->type = object->type;
  if (IsCamera(source)) destination->box = object->camera_supplement.box;
}

bool AppendTrackSnapshot(double timestamp, const TrackPtr& track,
                         std::vector<base::ObjectPtr>* output) {
  if (!track || !output || !std::isfinite(timestamp) ||
      std::fabs(track->GetFusedObject()->GetTimestamp() - timestamp) > 1e-6) {
    AERROR << "Cannot publish a fusion state at an unpredicted timestamp.";
    return false;
  }
  auto object = base::ObjectPool::Instance().Get();
  *object = *track->GetFusedObject()->GetBaseObject();
  object->fusion_supplement.on_use = true;
  auto& measurements = object->fusion_supplement.measurements;
  measurements.clear();
  for (const auto* sources : {&track->GetLidarObjects(),
                             &track->GetRadarObjects(),
                             &track->GetCameraObjects()}) {
    for (const auto& item : *sources) {
      measurements.emplace_back();
      CopySensorMeasurement(item.second, &measurements.back());
    }
  }
  object->track_id = track->GetTrackId();
  object->latest_tracked_time = timestamp;
  object->tracking_time = track->GetTrackingPeriod();
  output->push_back(object);
  return true;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
