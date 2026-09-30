#pragma once

#include <vector>

#include "modules/perception/fusion/base/track.h"

namespace apollo {
namespace perception {
namespace fusion {

void CopySensorMeasurement(const SensorObjectConstPtr& source,
                           base::SensorObjectMeasurement* destination);
bool AppendTrackSnapshot(double timestamp, const TrackPtr& track,
                         std::vector<base::ObjectPtr>* output);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
