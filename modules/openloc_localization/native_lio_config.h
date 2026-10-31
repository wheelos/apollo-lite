// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include "openloc/localization/local/native_types.h"

#include "native_lio_host_config.pb.h"

namespace apollo::localization::openloc_diagnostic {

::openloc::localization::unified::LocalConfig LocalConfiguration(
    const NativeLioHostConfig& profile);
Eigen::Isometry3d LidarExtrinsic(const NativeLioHostConfig& profile);

}  // namespace apollo::localization::openloc_diagnostic
