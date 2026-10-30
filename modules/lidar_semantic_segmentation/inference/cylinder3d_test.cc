// Copyright 2026 WheelOS. All Rights Reserved.
// SPDX-License-Identifier: Apache-2.0
#include "modules/lidar_semantic_segmentation/inference/cylinder3d.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ATen/ATen.h>
#include <ATen/Parallel.h>
#include <ATen/core/grad_mode.h>
#ifdef USE_GPU
#include <c10/cuda/CUDAGuard.h>
#include <c10/cuda/CUDAStream.h>
#endif

#include "gtest/gtest.h"

#include "nlohmann/json.hpp"

#include "modules/lidar_semantic_segmentation/inference/cylinder_batch_norm.h"
#include "modules/lidar_semantic_segmentation/inference/cylinder_sparse.h"

namespace apollo {
namespace lidar_semantic_segmentation {
namespace {

void AddPoint(apollo::drivers::PointCloud* cloud, float x, float y, float z,
              uint32_t intensity = 255) {
  auto* point = cloud->add_point();
  point->set_x(x);
  point->set_y(y);
  point->set_z(z);
  point->set_intensity(intensity);
}

TEST(Cylinder3DTest, FusedNormalizationPreservesActivationOrderAndRounding) {
  at::NoGradGuard guard;
  const bool cuda = std::getenv("CYLINDER3D_TEST_CUDA") != nullptr;
  const auto device = at::Device(cuda ? at::kCUDA : at::kCPU);
#ifdef USE_GPU
  const auto stream =
      cuda ? c10::cuda::getStreamFromPool() : c10::cuda::getDefaultCUDAStream();
  c10::cuda::CUDAStreamGuard stream_guard(stream);
#endif
  using Activation = cylinder::BatchNormActivation;
  for (int channels : {9, 64}) {
    auto options = at::TensorOptions().dtype(at::kFloat).device(device);
    auto input = (at::arange(513 * channels, options).remainder(37) - 18)
                     .reshape({513, channels});
    input[0][0] = std::numeric_limits<float>::quiet_NaN();
    input[0][1] = std::numeric_limits<float>::infinity();
    input[0][2] = -std::numeric_limits<float>::infinity();
    auto mean = at::linspace(-2, 2, channels, options);
    auto scale = at::linspace(-1, 1, channels, options);
    auto bias = at::linspace(1, -3, channels, options);
    for (auto activation :
         {Activation::kNone, Activation::kReluAfter, Activation::kLeakyBefore,
          Activation::kSigmoidAfter}) {
      auto values = activation == Activation::kLeakyBefore
                        ? at::leaky_relu(input, 0.01)
                        : input;
      auto expected = (values - mean) * scale + bias;
      if (activation == Activation::kReluAfter) {
        expected = at::relu(expected);
      } else if (activation == Activation::kSigmoidAfter) {
        expected = at::sigmoid(expected);
      }
      auto actual = cylinder::BatchNorm(input, mean, scale, bias, activation);
      EXPECT_TRUE(at::allclose(actual, expected, 0, 0, true))
          << "channels=" << channels
          << " activation=" << static_cast<int>(activation);
      auto empty = input.narrow(0, 0, 0);
      EXPECT_EQ(
          cylinder::BatchNorm(empty, mean, scale, bias, activation).numel(), 0);
    }
  }
}

TEST(CylinderProjectionTest, MatchesTrainingFeaturesAndClipsOnlyIndices) {
  CylinderProjectionOptions options;
  options.grid = {3, 5, 3};
  options.minimum = {0, -3.1415926, -2};
  options.maximum = {4, 3.1415926, 2};
  CylinderProjector projector;
  std::string error;
  ASSERT_TRUE(projector.Init(options, &error)) << error;
  apollo::drivers::PointCloud cloud;
  AddPoint(&cloud, 2, 0, 0, 51);
  AddPoint(&cloud, 6, 0, 4);
  AddPoint(&cloud, 0, 0, 0);
  AddPoint(&cloud, std::numeric_limits<float>::quiet_NaN(), 1, 0);
  CylinderInput input;
  ASSERT_TRUE(projector.Project(cloud, 0, &input, &error)) << error;
  ASSERT_EQ(input.features.size(), 18U);
  EXPECT_EQ(input.point_indices, (std::vector<uint32_t>{0, 1}));
  EXPECT_EQ(input.coordinates[0], (CylinderCoordinate{1, 2, 1}));
  EXPECT_EQ(input.coordinates[1], (CylinderCoordinate{2, 2, 2}));
  EXPECT_FLOAT_EQ(input.features[0], -1);
  EXPECT_NEAR(input.features[1], -3.1415926 / 4.0, 1e-6);
  EXPECT_FLOAT_EQ(input.features[2], -1);
  EXPECT_FLOAT_EQ(input.features[3], 2);
  EXPECT_FLOAT_EQ(input.features[6], 2);
  EXPECT_NEAR(input.features[8], 0.2, 1e-7);
  EXPECT_FLOAT_EQ(input.features[9], 1);
  EXPECT_FLOAT_EQ(input.features[11], 1);
  EXPECT_FLOAT_EQ(input.features[12], 6);
  ASSERT_TRUE(projector.Project(cloud, 3, &input, &error));
  EXPECT_FLOAT_EQ(input.features[6], -2);
  EXPECT_EQ(input.point_indices, (std::vector<uint32_t>{0, 1}));
}

TEST(CylinderProjectionTest, RejectsInvalidConfigOverflowAndPointTruncation) {
  CylinderProjectionOptions options;
  CylinderProjector projector;
  std::string error;
  options.grid[0] = 1;
  EXPECT_FALSE(projector.Init(options, &error));
  options.grid = {480, 360, 32};
  options.intensity_scale = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(projector.Init(options, &error));
  options.intensity_scale = 1;
  options.max_points = 1;
  ASSERT_TRUE(projector.Init(options, &error));
  apollo::drivers::PointCloud cloud;
  AddPoint(&cloud, 1, 0, 0);
  AddPoint(&cloud, 2, 0, 0);
  CylinderInput input;
  EXPECT_FALSE(projector.Project(cloud, 0, &input, &error));
  cloud.clear_point();
  AddPoint(&cloud, std::numeric_limits<float>::max(), 1, 0);
  EXPECT_FALSE(projector.Project(cloud, 0, &input, &error));
  cloud.clear_point();
  ASSERT_TRUE(projector.Project(cloud, 0, &input, &error));
  EXPECT_TRUE(input.features.empty());
}

at::Tensor Dense(const cylinder::SparseTensor& sparse) {
  auto dense = at::zeros({1, sparse.features.size(1), sparse.shape[0],
                          sparse.shape[1], sparse.shape[2]},
                         sparse.features.options());
  for (std::size_t row = 0; row < sparse.coordinates->size(); ++row) {
    const auto& c = (*sparse.coordinates)[row];
    dense.index_put_({0, at::indexing::Slice(), c[0], c[1], c[2]},
                     sparse.features[row]);
  }
  return dense;
}

void ExpectDenseValues(const cylinder::SparseTensor& sparse,
                       const at::Tensor& dense) {
  for (std::size_t row = 0; row < sparse.coordinates->size(); ++row) {
    const auto& c = (*sparse.coordinates)[row];
    auto expected = dense.index({0, at::indexing::Slice(), c[0], c[1], c[2]});
    EXPECT_TRUE(at::allclose(sparse.features[row], expected, 1e-5, 1e-6))
        << "row " << row;
  }
}

TEST(CylinderSparseTest, AsymmetricDownsampleAndInverseMatchDenseReference) {
  at::NoGradGuard guard;
  at::set_num_threads(1);
  auto coordinates = std::make_shared<cylinder::Coordinates>(
      cylinder::Coordinates{{0, 0, 0}, {2, 1, 1}, {3, 3, 2}});
  cylinder::SparseTensor input{
      at::arange(6, at::kFloat).reshape({3, 2}) / 10, coordinates, {4, 4, 3}};
  auto weights =
      at::arange(3 * 3 * 3 * 2 * 3, at::kFloat).reshape({3, 3, 3, 2, 3}) / 100;
  cylinder::SparseRelation relation;
  auto output = cylinder::Convolve(input, weights, {}, {2, 2, 1}, {1, 1, 1},
                                   false, &relation);
  auto expected = at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}), {},
                             {2, 2, 1}, {1, 1, 1});
  ExpectDenseValues(output, expected);
  auto mask = at::conv3d((Dense(input).abs().sum(1, true) > 0).to(at::kFloat),
                         at::ones({1, 1, 3, 3, 3}), {}, {2, 2, 1}, {1, 1, 1});
  EXPECT_EQ(output.coordinates->size(),
            static_cast<std::size_t>((mask > 0).sum().item<int64_t>()));
  auto inverse_weights =
      at::arange(3 * 3 * 3 * 3 * 2, at::kFloat).reshape({3, 3, 3, 3, 2}) / 200;
  auto restored = cylinder::InverseConvolve(output, inverse_weights, relation);
  auto inverse_expected = at::conv_transpose3d(
      Dense(output), inverse_weights.permute({3, 4, 0, 1, 2}), {}, {2, 2, 1},
      {1, 1, 1}, {1, 1, 0}, 1, {1, 1, 1});
  EXPECT_EQ(restored.coordinates, input.coordinates);
  ExpectDenseValues(restored, inverse_expected);
  auto different = output;
  different.coordinates =
      std::make_shared<cylinder::Coordinates>(*output.coordinates);
  EXPECT_THROW(cylinder::InverseConvolve(different, inverse_weights, relation),
               c10::Error);
}

TEST(CylinderSparseTest, SubmanifoldKeepsOnlyActiveCoordinatesAndBias) {
  auto coordinates = std::make_shared<cylinder::Coordinates>(
      cylinder::Coordinates{{0, 0, 0}, {0, 1, 1}, {2, 2, 2}});
  cylinder::SparseTensor input{at::ones({3, 2}), coordinates, {3, 3, 3}};
  auto weights =
      at::arange(1 * 3 * 3 * 2 * 4, at::kFloat).reshape({1, 3, 3, 2, 4}) / 100;
  auto bias = at::arange(4, at::kFloat);
  auto output =
      cylinder::Convolve(input, weights, bias, {1, 1, 1}, {0, 1, 1}, true);
  auto expected = at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}),
                             bias, {1, 1, 1}, {0, 1, 1});
  EXPECT_EQ(output.coordinates, input.coordinates);
  ExpectDenseValues(output, expected);
  input.coordinates = std::make_shared<cylinder::Coordinates>(
      cylinder::Coordinates{{0, 0, 0}, {0, 0, 0}, {2, 2, 2}});
  EXPECT_THROW(
      cylinder::Convolve(input, weights, bias, {1, 1, 1}, {0, 1, 1}, true),
      c10::Error);
}

TEST(CylinderSparseTest, CachedPairsReuseGeometryNotFeaturesOrOtherGrids) {
  at::NoGradGuard guard;
  auto coordinates = std::make_shared<cylinder::Coordinates>(
      cylinder::Coordinates{{0, 0, 0}, {0, 1, 1}, {2, 2, 2}});
  cylinder::SparseTensor input{
      at::arange(6, at::kFloat).reshape({3, 2}), coordinates, {3, 3, 3}};
  cylinder::SparseIndiceCache cache;
  auto weights =
      at::arange(9 * 2 * 4, at::kFloat).reshape({1, 3, 3, 2, 4}) / 100;
  auto first = cylinder::Convolve(input, weights, {}, {1, 1, 1}, {0, 1, 1},
                                  true, nullptr, &cache);
  ASSERT_EQ(cache.size(), 1U);
  const auto* pairs = cache.front()->pairs.data_ptr<int64_t>();
  input.features = input.features * -2 + 1;
  auto second = cylinder::Convolve(input, weights, {}, {1, 1, 1}, {0, 1, 1},
                                   true, nullptr, &cache);
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_EQ(pairs, cache.front()->pairs.data_ptr<int64_t>());
  EXPECT_FALSE(at::allclose(first.features, second.features));
  auto expected = at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}), {},
                             {1, 1, 1}, {0, 1, 1});
  ExpectDenseValues(second, expected);

  auto other_kernel = weights.reshape({3, 1, 3, 2, 4});
  auto third = cylinder::Convolve(input, other_kernel, {}, {1, 1, 1}, {1, 0, 1},
                                  true, nullptr, &cache);
  EXPECT_EQ(cache.size(), 2U);
  ExpectDenseValues(
      third, at::conv3d(Dense(input), other_kernel.permute({4, 3, 0, 1, 2}), {},
                        {1, 1, 1}, {1, 0, 1}));
  input.shape = {4, 4, 4};
  auto resized = cylinder::Convolve(input, weights, {}, {1, 1, 1}, {0, 1, 1},
                                    true, nullptr, &cache);
  EXPECT_EQ(cache.size(), 3U);
  ExpectDenseValues(
      resized, at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}), {},
                          {1, 1, 1}, {0, 1, 1}));
  input.coordinates = std::make_shared<cylinder::Coordinates>(
      cylinder::Coordinates{{1, 0, 0}, {0, 1, 2}, {2, 2, 2}});
  auto changed = cylinder::Convolve(input, weights, {}, {1, 1, 1}, {0, 1, 1},
                                    true, nullptr, &cache);
  EXPECT_EQ(cache.size(), 4U);
  ExpectDenseValues(
      changed, at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}), {},
                          {1, 1, 1}, {0, 1, 1}));
  input.shape = {100000, 100000, 3};
  auto large_grid = cylinder::Convolve(input, weights, {}, {1, 1, 1}, {0, 1, 1},
                                       true, nullptr, &cache);
  EXPECT_EQ(cache.size(), 5U);
  EXPECT_TRUE(at::allclose(large_grid.features, changed.features));
}

TEST(CylinderSparseTest, LargePairEnumerationMatchesDenseOrderAndBoundaries) {
  at::NoGradGuard guard;
  at::set_num_threads(4);
  auto coordinates = std::make_shared<cylinder::Coordinates>();
  for (int a = 0; a < 12; ++a) {
    for (int b = 0; b < 12; ++b) {
      for (int c = 0; c < 8; ++c) {
        coordinates->push_back({a, b, c});
      }
    }
  }
  cylinder::SparseTensor input{
      at::sin(at::arange(1152 * 2, at::kFloat)).reshape({1152, 2}),
      coordinates,
      {12, 12, 8}};
  auto weights =
      at::sin(at::arange(27 * 2 * 4, at::kFloat)).reshape({3, 3, 3, 2, 4});
  auto output =
      cylinder::Convolve(input, weights, {}, {1, 1, 1}, {1, 1, 1}, true);
  auto expected = at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}), {},
                             {1, 1, 1}, {1, 1, 1});
  EXPECT_EQ(output.coordinates, input.coordinates);
  EXPECT_TRUE(at::allclose(output.features,
                           expected.permute({0, 2, 3, 4, 1}).reshape({1152, 4}),
                           1e-4, 1e-5));
  at::set_num_threads(1);
}

#ifdef USE_GPU
TEST(CylinderSparseTest, SpconvMatchesDenseForwardInverseOnChangingStreams) {
  if (std::getenv("CYLINDER3D_TEST_SPCONV") == nullptr) {
    GTEST_SKIP() << "Opt in to the source-built spconv CUDA backend";
  }
  at::NoGradGuard guard;
  for (int iteration = 0; iteration < 3; ++iteration) {
    c10::cuda::CUDAStreamGuard stream(c10::cuda::getStreamFromPool());
    auto coordinates = std::make_shared<cylinder::Coordinates>(
        cylinder::Coordinates{{0, 0, 0}, {1, 0, 1}, {2, 1, 1}, {3, 3, 2}});
    auto options = at::TensorOptions().device(at::kCUDA).dtype(at::kFloat);
    cylinder::SparseTensor input{
        at::sin(at::arange(8, options) + iteration).reshape({4, 2}),
        coordinates,
        {4, 4, 3},
        true};
    cylinder::SparseIndiceCache cache;
    for (const auto& kernel :
         {CylinderCoordinate{1, 3, 3}, CylinderCoordinate{3, 1, 3},
          CylinderCoordinate{3, 1, 1}, CylinderCoordinate{1, 1, 1}}) {
      const auto count = kernel[0] * kernel[1] * kernel[2] * 2 * 4;
      auto weights = at::cos(at::arange(count, options))
                         .reshape({kernel[0], kernel[1], kernel[2], 2, 4});
      CylinderCoordinate padding{kernel[0] / 2, kernel[1] / 2, kernel[2] / 2};
      auto output =
          cylinder::Convolve(input, weights, at::ones({4}, options), {1, 1, 1},
                             padding, true, nullptr, &cache);
      ASSERT_TRUE(output.device_coordinates);
      EXPECT_EQ(output.device_coordinates->coordinates, output.coordinates);
      EXPECT_EQ(cache.back()->input_device_coordinates,
                cache.front()->input_device_coordinates);
      auto expected = at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}),
                                 at::ones({4}, options), {1, 1, 1}, padding);
      ExpectDenseValues(output, expected);
      auto again = cylinder::Convolve(input, weights, {}, {1, 1, 1}, padding,
                                      true, nullptr, &cache);
      EXPECT_TRUE(
          at::allclose(again.features + 1, output.features, 1e-5, 1e-6));
    }
    EXPECT_EQ(cache.size(), 4U);
    auto weights =
        at::sin(at::arange(27 * 2 * 3, options)).reshape({3, 3, 3, 2, 3});
    cylinder::SparseRelation relation;
    auto output = cylinder::Convolve(input, weights, {}, {2, 2, 1}, {1, 1, 1},
                                     false, &relation);
    auto next_weights =
        at::sin(at::arange(9 * 3 * 2, options)).reshape({1, 3, 3, 3, 2});
    auto next = cylinder::Convolve(output, next_weights, {}, {1, 1, 1},
                                   {0, 1, 1}, true, nullptr, &cache);
    EXPECT_EQ(next.device_coordinates, output.device_coordinates);
    ExpectDenseValues(
        next, at::conv3d(Dense(output), next_weights.permute({4, 3, 0, 1, 2}),
                         {}, {1, 1, 1}, {0, 1, 1}));
    ExpectDenseValues(
        output, at::conv3d(Dense(input), weights.permute({4, 3, 0, 1, 2}), {},
                           {2, 2, 1}, {1, 1, 1}));
    auto inverse_weights = weights.permute({0, 1, 2, 4, 3}).contiguous();
    auto restored =
        cylinder::InverseConvolve(output, inverse_weights, relation);
    EXPECT_EQ(restored.coordinates, coordinates);
    EXPECT_EQ(restored.device_coordinates, relation.input_device_coordinates);
    ExpectDenseValues(
        restored, at::conv_transpose3d(
                      Dense(output), inverse_weights.permute({3, 4, 0, 1, 2}),
                      {}, {2, 2, 1}, {1, 1, 1}, {1, 1, 0}, 1, {1, 1, 1}));
    auto different = output;
    different.coordinates =
        std::make_shared<cylinder::Coordinates>(*output.coordinates);
    EXPECT_THROW(
        cylinder::InverseConvolve(different, inverse_weights, relation),
        c10::Error);
    auto changed = input;
    changed.device_coordinates = cache.front()->input_device_coordinates;
    changed.coordinates = std::make_shared<cylinder::Coordinates>(
        cylinder::Coordinates{{0, 1, 0}, {1, 1, 1}, {2, 1, 1}, {3, 3, 2}});
    auto moved = cylinder::Convolve(changed, weights, {}, {1, 1, 1}, {1, 1, 1},
                                    true, nullptr, &cache);
    EXPECT_NE(moved.device_coordinates, changed.device_coordinates);
    ExpectDenseValues(
        moved, at::conv3d(Dense(changed), weights.permute({4, 3, 0, 1, 2}), {},
                          {1, 1, 1}, {1, 1, 1}));
    auto smaller = changed;
    smaller.shape = {1, 1, 1};
    EXPECT_THROW(cylinder::Convolve(smaller, weights, {}, {1, 1, 1}, {1, 1, 1},
                                    true, nullptr, &cache),
                 c10::Error);
    input.coordinates = std::make_shared<cylinder::Coordinates>(
        cylinder::Coordinates{{0, 0, 0}, {0, 0, 0}, {2, 1, 1}, {3, 3, 2}});
    EXPECT_THROW(
        cylinder::Convolve(input, weights, {}, {1, 1, 1}, {1, 1, 1}, true),
        c10::Error);
  }
}
#endif

TEST(Cylinder3DTest, RejectsMissingModelAndUninitializedExecution) {
  Cylinder3DExecutor executor;
  CylinderInput input;
  std::vector<float> logits;
  std::string error;
  EXPECT_FALSE(executor.Run(input, &logits, &error));
  CylinderProjectionOptions projection;
  Cylinder3DOptions options;
  options.model_path = "/nonexistent/cylinder3d.ts";
  EXPECT_FALSE(executor.Init(options, &projection, &error));
  EXPECT_FALSE(error.empty());
  Cylinder3DSegmenter segmenter;
  LidarSemanticSegmentationResult result;
  result.set_source_topic("unchanged");
  EXPECT_FALSE(segmenter.Segment({}, &result, &error));
  EXPECT_EQ(result.source_topic(), "unchanged");
}

TEST(Cylinder3DTest, UpstreamDenseReferenceAllVotesAndResultContract) {
  const char* fixture = std::getenv("CYLINDER3D_TEST_FIXTURE");
  if (fixture == nullptr) {
    GTEST_SKIP() << "Generate the upstream dense reference fixture first";
  }
  at::set_num_threads(1);
  const std::string directory(fixture);
  std::ifstream stream(directory + "/reference.json");
  ASSERT_TRUE(stream.good());
  const auto reference = nlohmann::json::parse(stream);
  Cylinder3DOptions options;
  options.model_path = directory + "/model.ts";
  options.source_topic = "/test/points";
  options.sensor_name = "test_lidar";
  options.device_id = std::getenv("CYLINDER3D_TEST_CUDA") == nullptr ? -1 : 0;
  options.use_spconv = std::getenv("CYLINDER3D_TEST_SPCONV") != nullptr;
  Cylinder3DExecutor executor;
  CylinderProjectionOptions projection;
  std::string error;
  ASSERT_TRUE(executor.Init(options, &projection, &error)) << error;
  CylinderProjector projector;
  ASSERT_TRUE(projector.Init(projection, &error));
  apollo::drivers::PointCloud cloud;
  cloud.set_frame_id("lidar");
  cloud.mutable_header()->set_timestamp_sec(123.25);
  for (const auto& point : reference.at("points")) {
    AddPoint(&cloud, point[0], point[1], point[2], point[3]);
  }
  std::vector<double> fused;
  std::vector<CylinderInput> votes;
  std::vector<double> actual_fused;
  std::vector<float> first_logits;
  for (int vote = 0; vote < 4; ++vote) {
    CylinderInput input;
    ASSERT_TRUE(projector.Project(cloud, vote, &input, &error));
    std::vector<float> logits;
    ASSERT_TRUE(executor.Run(input, &logits, &error)) << error;
    const auto expected =
        reference.at("logits")[vote].get<std::vector<float>>();
    ASSERT_EQ(logits.size(), expected.size());
    if (vote == 0) {
      fused.assign(logits.size(), 0.0);
      actual_fused.assign(logits.size(), 0.0);
      first_logits = logits;
    }
    for (std::size_t index = 0; index < logits.size(); ++index) {
      EXPECT_NEAR(logits[index], expected[index],
                  1e-5 + 2e-4 * std::abs(expected[index]))
          << "vote " << vote << ", logit " << index;
      fused[index] += expected[index];
      actual_fused[index] += logits[index];
    }
    votes.push_back(std::move(input));
  }
  auto reordered = votes.front();
  const auto original_rows = reordered.coordinates.size();
  std::reverse(reordered.coordinates.begin(), reordered.coordinates.end());
  for (std::size_t row = 0; row < original_rows; ++row) {
    std::copy_n(votes.front().features.begin() + (original_rows - row - 1) * 9,
                9, reordered.features.begin() + row * 9);
  }
  reordered.coordinates.push_back(reordered.coordinates.back());
  const std::vector<float> duplicate(reordered.features.end() - 9,
                                     reordered.features.end());
  reordered.features.insert(reordered.features.end(), duplicate.begin(),
                            duplicate.end());
  reordered.point_indices.push_back(static_cast<uint32_t>(original_rows));
  std::vector<float> reordered_logits;
  ASSERT_TRUE(executor.Run(reordered, &reordered_logits, &error)) << error;
  ASSERT_EQ(reordered_logits.size(), (original_rows + 1) * 20);
  for (std::size_t row = 0; row <= original_rows; ++row) {
    const auto source = row == original_rows ? 0 : original_rows - row - 1;
    for (int label = 0; label < 20; ++label) {
      EXPECT_NEAR(reordered_logits[row * 20 + label],
                  first_logits[source * 20 + label],
                  1e-5 + 2e-4 * std::abs(first_logits[source * 20 + label]));
    }
  }
  std::vector<uint32_t> decoded;
  std::vector<float> confidences;
  ASSERT_TRUE(
      executor.RunLabels({votes.front()}, &decoded, &confidences, &error))
      << error;
  ASSERT_EQ(decoded.size() * 20, first_logits.size());
  for (std::size_t row = 0; row < decoded.size(); ++row) {
    const auto begin = first_logits.begin() + row * 20;
    const auto best = std::max_element(begin, begin + 20);
    double sum = 0;
    for (auto it = begin; it != begin + 20; ++it) {
      sum += std::exp(static_cast<double>(*it) - *best);
    }
    EXPECT_EQ(decoded[row], static_cast<uint32_t>(best - begin));
    EXPECT_NEAR(confidences[row], 1.0 / sum, 1e-6);
  }
  ASSERT_TRUE(executor.RunLabels(votes, &decoded, &confidences, &error))
      << error;
  ASSERT_EQ(decoded.size() * 20, actual_fused.size());
  ASSERT_EQ(confidences.size(), decoded.size());
  for (std::size_t row = 0; row < decoded.size(); ++row) {
    const auto begin = actual_fused.begin() + row * 20;
    const auto best = std::max_element(begin, begin + 20);
    double sum = 0;
    for (auto it = begin; it != begin + 20; ++it) {
      sum += std::exp(*it - *best);
    }
    EXPECT_EQ(decoded[row], static_cast<uint32_t>(best - begin));
    EXPECT_NEAR(confidences[row], 1.0 / sum, 1e-6);
  }
  const auto saved_labels = decoded;
  const auto saved_confidences = confidences;
  auto mismatched = votes;
  ++mismatched.back().point_indices.front();
  EXPECT_FALSE(executor.RunLabels(mismatched, &decoded, &confidences, &error));
  EXPECT_EQ(decoded, saved_labels);
  EXPECT_EQ(confidences, saved_confidences);
  auto invalid_input = votes.front();
  invalid_input.coordinates.front()[0] = projection.grid[0];
  EXPECT_FALSE(
      executor.RunLabels({invalid_input}, &decoded, &confidences, &error));
  invalid_input = votes.front();
  invalid_input.features.front() = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(
      executor.RunLabels({invalid_input}, &decoded, &confidences, &error));
  EXPECT_FALSE(executor.RunLabels({}, &decoded, &confidences, &error));
  EXPECT_FALSE(
      executor.RunLabels({votes[0], votes[1]}, &decoded, &confidences, &error));
  EXPECT_EQ(decoded, saved_labels);
  EXPECT_EQ(confidences, saved_confidences);
  ASSERT_TRUE(
      executor.RunLabels({CylinderInput{}}, &decoded, &confidences, &error));
  EXPECT_TRUE(decoded.empty());
  EXPECT_TRUE(confidences.empty());
  Cylinder3DExecutor legacy;
  auto legacy_options = options;
  legacy_options.model_path = directory + "/legacy.ts";
  ASSERT_TRUE(legacy.Init(legacy_options, &projection, &error)) << error;
  CylinderInput first;
  ASSERT_TRUE(projector.Project(cloud, 0, &first, &error));
  std::vector<float> legacy_logits;
  ASSERT_TRUE(legacy.Run(first, &legacy_logits, &error)) << error;
  const auto expected_first =
      reference.at("logits")[0].get<std::vector<float>>();
  ASSERT_EQ(legacy_logits.size(), expected_first.size());
  for (std::size_t index = 0; index < legacy_logits.size(); ++index) {
    EXPECT_NEAR(legacy_logits[index], expected_first[index],
                1e-5 + 2e-4 * std::abs(expected_first[index]));
  }
  for (const char* invalid : {"incomplete.ts", "unexpected.ts"}) {
    Cylinder3DExecutor rejected;
    auto invalid_options = options;
    invalid_options.model_path = directory + "/" + invalid;
    EXPECT_FALSE(rejected.Init(invalid_options, &projection, &error));
    EXPECT_FALSE(rejected.Run(first, &legacy_logits, &error));
  }
  options.flip_tta = true;
  Cylinder3DSegmenter segmenter;
  ASSERT_TRUE(segmenter.Init(options, &error)) << error;
  const int valid_count = cloud.point_size();
  AddPoint(&cloud, 0, 0, 0);
  AddPoint(&cloud, std::numeric_limits<float>::quiet_NaN(), 0, 0);
  LidarSemanticSegmentationResult result;
  ASSERT_TRUE(segmenter.Segment(cloud, &result, &error)) << error;
  EXPECT_EQ(result.point_count(), static_cast<uint32_t>(cloud.point_size()));
  EXPECT_EQ(result.point_label_size(), cloud.point_size());
  EXPECT_EQ(result.source_topic(), options.source_topic);
  EXPECT_EQ(result.source_frame_id(), "lidar");
  EXPECT_DOUBLE_EQ(result.header().timestamp_sec(), 123.25);
  EXPECT_FALSE(result.has_range_image_width());
  for (int row = 0; row < valid_count; ++row) {
    const auto begin = fused.begin() + row * 20;
    const auto best = std::max_element(begin, begin + 20);
    double sum = 0;
    for (auto it = begin; it != begin + 20; ++it) {
      sum += std::exp(*it - *best);
    }
    EXPECT_EQ(result.point_label(row).point_index(),
              static_cast<uint32_t>(row));
    EXPECT_EQ(result.point_label(row).semantic_label(),
              static_cast<uint32_t>(best - begin));
    EXPECT_NEAR(result.point_label(row).confidence(), 1.0 / sum, 1e-5);
    EXPECT_EQ(result.point_label(row).range_image_x(), -1);
  }
  EXPECT_EQ(result.point_label(valid_count).semantic_label(), 0U);
  EXPECT_FLOAT_EQ(result.point_label(valid_count + 1).confidence(), 0);
  apollo::drivers::PointCloud overflow;
  AddPoint(&overflow, std::numeric_limits<float>::max(), 1, 0);
  const auto previous = result.SerializeAsString();
  EXPECT_FALSE(segmenter.Segment(overflow, &result, &error));
  EXPECT_EQ(result.SerializeAsString(), previous);
  ASSERT_TRUE(segmenter.Segment({}, &result, &error));
  EXPECT_EQ(result.point_label_size(), 0);
}

}  // namespace
}  // namespace lidar_semantic_segmentation
}  // namespace apollo
