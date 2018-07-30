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

#include "cartographer_ros/z_pos_filtering_points_processor.h"

#include "absl/memory/memory.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_batch.h"

namespace cartographer_ros {

std::unique_ptr<ZPositionFilteringPointsProcessor>
ZPositionFilteringPointsProcessor::FromDictionary(
    cartographer::common::LuaParameterDictionary* const dictionary,
    PointsProcessor* const next) {
  return absl::make_unique<ZPositionFilteringPointsProcessor>(
      dictionary->GetDouble("min_z"), dictionary->GetDouble("max_z"),
      next);
}

ZPositionFilteringPointsProcessor::ZPositionFilteringPointsProcessor(
    const double min_z, const double max_z, PointsProcessor* next)
    : min_z_(min_z), max_z_(max_z), next_(next) {}

void ZPositionFilteringPointsProcessor::Process(
    std::unique_ptr<cartographer::io::PointsBatch> batch) {
  std::unordered_set<int> to_remove;
  for (size_t i = 0; i < batch->points.size(); ++i) {
    const float pos = (batch->points[i] - batch->origin).z();
    if (!(min_z_ <= pos && pos <= max_z_)) {
      to_remove.insert(i);
    }
  }
  RemovePoints(to_remove, batch.get());
  next_->Process(std::move(batch));
}

  cartographer::io::PointsProcessor::FlushResult ZPositionFilteringPointsProcessor::Flush() {
  return next_->Flush();
}

}  // namespace cartographer_ros
