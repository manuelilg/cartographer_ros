/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_Z_POSITION_FILTERING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_Z_POSITION_FILTERING_POINTS_PROCESSOR_H_

#include <memory>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"

namespace cartographer_ros {

// Filters all points that have a z-position (relative to the 'origin') over 'max_z'
// or under 'min_z'.
class ZPositionFilteringPointsProcessor : public cartographer::io::PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "z_pos_filter";
  ZPositionFilteringPointsProcessor(double min_z, double max_z,
                                     PointsProcessor* next);
  static std::unique_ptr<ZPositionFilteringPointsProcessor> FromDictionary(
      cartographer::common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~ZPositionFilteringPointsProcessor() override {}

  ZPositionFilteringPointsProcessor(
      const ZPositionFilteringPointsProcessor&) = delete;
  ZPositionFilteringPointsProcessor& operator=(
      const ZPositionFilteringPointsProcessor&) = delete;

  void Process(std::unique_ptr<cartographer::io::PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  const double min_z_;
  const double max_z_;
  cartographer::io::PointsProcessor* const next_;
};

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_Z_POSITION_FILTERING_POINTS_PROCESSOR_H_
