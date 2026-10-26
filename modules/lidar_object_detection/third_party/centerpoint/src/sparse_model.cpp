// Copyright 2026 WheelOS All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "modules/lidar_object_detection/third_party/centerpoint/include/sparse_model.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "onnx-ml.pb.h"  // NOLINT(build/include_subdir)

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error("spconv ONNX: " + message);
}

int64_t Volume(const std::vector<int64_t>& shape) {
  int64_t count = 1;
  for (auto value : shape) {
    Require(value > 0 && count <= std::numeric_limits<int64_t>::max() / value,
            "invalid/overflowing tensor shape");
    count *= value;
  }
  return count;
}

const onnx::AttributeProto* Attribute(const onnx::NodeProto& node,
                                      const std::string& name) {
  for (const auto& attribute : node.attribute()) {
    if (attribute.name() == name) return &attribute;
  }
  return nullptr;
}

int Integer(const onnx::NodeProto& node, const std::string& name,
            int fallback) {
  const auto* attribute = Attribute(node, name);
  if (!attribute) return fallback;
  Require(attribute->type() == onnx::AttributeProto::INT &&
              attribute->i() >= std::numeric_limits<int>::min() &&
              attribute->i() <= std::numeric_limits<int>::max(),
          "invalid integer attribute " + name);
  return static_cast<int>(attribute->i());
}

std::string String(const onnx::NodeProto& node, const std::string& name) {
  const auto* attribute = Attribute(node, name);
  if (!attribute) return {};
  Require(attribute->type() == onnx::AttributeProto::STRING,
          "invalid string attribute " + name);
  return attribute->s();
}

std::vector<int> Dimensions(const onnx::NodeProto& node,
                            const std::string& name, size_t size) {
  const auto* attribute = Attribute(node, name);
  Require(attribute && attribute->type() == onnx::AttributeProto::INTS &&
              static_cast<size_t>(attribute->ints_size()) == size,
          "missing/invalid dimensions " + name);
  std::vector<int> result;
  for (auto value : attribute->ints()) {
    Require(value >= 0 && value <= std::numeric_limits<int>::max(),
            "invalid dimension in " + name);
    result.push_back(static_cast<int>(value));
  }
  return result;
}

std::vector<int> TransposePermutation(const onnx::NodeProto& node,
                                      size_t rank) {
  const auto* permutation = Attribute(node, "perm");
  const auto* legacy_dimensions = Attribute(node, "dims");
  Require(permutation == nullptr || legacy_dimensions == nullptr,
          "Transpose has both perm and dims attributes");
  if (permutation != nullptr) {
    return Dimensions(node, "perm", rank);
  }
  if (legacy_dimensions != nullptr) {
    return Dimensions(node, "dims", rank);
  }
  std::vector<int> result(rank);
  for (size_t axis = 0; axis < rank; ++axis) {
    result[axis] = static_cast<int>(rank - axis - 1);
  }
  return result;
}

std::vector<uint16_t> Weight(const onnx::GraphProto& graph,
                             const std::string& name,
                             const std::vector<int64_t>& shape) {
  const auto found = std::find_if(
      graph.initializer().begin(), graph.initializer().end(),
      [&](const onnx::TensorProto& value) { return value.name() == name; });
  Require(found != graph.initializer().end(), "missing initializer " + name);
  Require(found->data_type() == onnx::TensorProto::FLOAT16 &&
              std::vector<int64_t>(found->dims().begin(),
                                   found->dims().end()) == shape,
          "initializer dtype/shape mismatch: " + name);
  const auto count = Volume(shape);
  Require(static_cast<uint64_t>(count) <= 64 * 1024 * 1024 &&
              found->raw_data().size() == static_cast<size_t>(count) * 2,
          "initializer size mismatch: " + name);
  std::vector<uint16_t> result(count);
  std::memcpy(result.data(), found->raw_data().data(), result.size() * 2);
  return result;
}

}  // namespace

wheelos::perception::ops::spconv::SparseModel LoadSparseModel(
    const std::string& path) {
  std::ifstream file(path, std::ios::binary);
  onnx::ModelProto model;
  Require(file.good() && model.ParseFromIstream(&file), "cannot parse " + path);
  const auto& graph = model.graph();
  Require(graph.input_size() == 1 && graph.output_size() == 1 &&
              graph.node_size() > 0 &&
              graph.node(0).op_type() == "SparseConvolution",
          "expected one sparse input and one dense output");
  wheelos::perception::ops::spconv::SparseModel result;
  result.input = graph.input(0).name();
  result.output = graph.output(0).name();
  Require(graph.input(0).type().tensor_type().elem_type() ==
                  onnx::TensorProto::FLOAT16 &&
              graph.output(0).type().tensor_type().elem_type() ==
                  onnx::TensorProto::FLOAT16,
          "expected FP16 input/output");
  const auto& input_shape = graph.input(0).type().tensor_type().shape();
  const auto& output_shape = graph.output(0).type().tensor_type().shape();
  Require(input_shape.dim_size() == 2 && input_shape.dim(1).dim_value() == 5 &&
              output_shape.dim_size() == 4,
          "invalid graph input/output rank or feature count");
  result.input_channels = input_shape.dim(1).dim_value();
  for (const auto& dimension : output_shape.dim()) {
    Require(dimension.dim_value() > 0, "output dimensions must be static");
    result.output_shape.push_back(dimension.dim_value());
  }
  std::unordered_set<std::string> initializers;
  for (const auto& value : graph.initializer()) {
    Require(initializers.insert(value.name()).second, "duplicate initializer");
  }
  std::unordered_map<std::string, int> channels{{result.input, 5}};
  std::unordered_map<std::string, std::vector<int>> grids;
  std::unordered_map<std::string, std::vector<int>> dense_shapes;
  int64_t dense_volume = 0;
  for (const auto& node : graph.node()) {
    wheelos::perception::ops::spconv::SparseModelLayer layer;
    layer.op = node.op_type();
    Require(node.output_size() == 1 && node.input_size() >= 1,
            "invalid node arity: " + node.name());
    layer.output = node.output(0);
    layer.inputs.assign(node.input().begin(), node.input().end());
    std::unordered_set<std::string> attributes;
    for (const auto& attribute : node.attribute()) {
      Require(attributes.insert(attribute.name()).second,
              "duplicate attribute");
    }
    Require(channels.count(layer.inputs[0]) && !channels.count(layer.output),
            "unresolved input or duplicate output");
    const int input_channels = channels.at(layer.inputs[0]);
    int output_channels = input_channels;
    if (layer.op == "SparseConvolution") {
      Require(node.input_size() == 3 && input_channels > 0 &&
                  Integer(node, "groups", 1) == 1 &&
                  Integer(node, "ndim", 3) == 3 &&
                  Integer(node, "inverse", 0) == 0 &&
                  Integer(node, "transposed", 0) == 0 &&
                  Integer(node, "in_channels", -1) == input_channels,
              "unsupported convolution configuration");
      for (const auto& name : {"precision", "output_precision"}) {
        const auto precision = String(node, name);
        Require(precision.empty() || precision == "fp16" || precision == "int8",
                "unsupported convolution precision hint");
      }
      Require(!Attribute(node, "output_padding") ||
                  Dimensions(node, "output_padding", 3) ==
                      std::vector<int>({0, 0, 0}),
              "output padding is not supported");
      layer.kernel = Dimensions(node, "kernel_size", 3);
      layer.stride = Dimensions(node, "stride", 3);
      layer.padding = Dimensions(node, "padding", 3);
      layer.dilation = Dimensions(node, "dilation", 3);
      layer.input_grid = Dimensions(node, "input_spatial_shape", 3);
      layer.output_grid = Dimensions(node, "output_spatial_shape", 3);
      const int subm = Integer(node, "subm", 0);
      Require(subm == 0 || subm == 1, "invalid SubM flag");
      layer.subm = subm != 0;
      layer.bound = Integer(node, "output_bound", 0);
      output_channels = Integer(node, "out_channels", 0);
      Require(layer.bound > 0 && layer.bound <= 200000 && output_channels > 0 &&
                  output_channels <= 256,
              "unsupported output/channel bound");
      const auto activation = String(node, "activation");
      Require(
          activation.empty() || activation == "None" || activation == "ReLU",
          "unsupported activation");
      layer.relu = activation == "ReLU";
      const int64_t kernel_volume = Volume(
          std::vector<int64_t>(layer.kernel.begin(), layer.kernel.end()));
      Require(kernel_volume <= 32, "only single-mask convolution is supported");
      for (int j = 0; j < 3; ++j) {
        Require(layer.stride[j] > 0 && layer.stride[j] <= 16 &&
                    layer.dilation[j] > 0 && layer.dilation[j] <= 16 &&
                    layer.padding[j] <= 16 && layer.input_grid[j] > 0 &&
                    layer.input_grid[j] <= 4096 && layer.output_grid[j] > 0,
                "invalid convolution dimensions");
        const int expected = layer.subm
                                 ? layer.input_grid[j]
                                 : (layer.input_grid[j] + 2 * layer.padding[j] -
                                    layer.dilation[j] * (layer.kernel[j] - 1) -
                                    1) / layer.stride[j] +
                                       1;
        Require(layer.output_grid[j] == expected &&
                    (!layer.subm || layer.stride[j] == 1),
                "convolution spatial shape mismatch");
      }
      if (grids.count(layer.inputs[0])) {
        Require(grids.at(layer.inputs[0]) == layer.input_grid,
                "input grid mismatch");
      } else {
        Require(layer.inputs[0] == result.input, "missing input grid");
        grids[result.input] = layer.input_grid;
      }
      layer.weight_shape = {output_channels, layer.kernel[0], layer.kernel[1],
                            layer.kernel[2], input_channels};
      layer.weight_data = Weight(graph, node.input(1), layer.weight_shape);
      layer.bias_data = Weight(graph, node.input(2), {output_channels});
      grids[layer.output] = layer.output_grid;
    } else if (layer.op == "Add") {
      Require(node.input_size() == 2 && input_channels > 0 &&
                  channels.count(node.input(1)) &&
                  channels.at(node.input(1)) == input_channels &&
                  grids.at(node.input(0)) == grids.at(node.input(1)),
              "unsupported sparse Add");
      grids[layer.output] = grids.at(node.input(0));
    } else if (layer.op == "Relu") {
      Require(node.input_size() == 1 && input_channels > 0, "unsupported Relu");
      grids[layer.output] = grids.at(node.input(0));
    } else if (layer.op == "ScatterDense") {
      const auto format = String(node, "format");
      Require(node.input_size() == 1 && input_channels > 0 &&
                  (format == "zyx" || format == "xyz") &&
                  (String(node, "output_layout").empty() ||
                   String(node, "output_layout") == "NCHW"),
              "unsupported ScatterDense layout");
      layer.xyz = format == "xyz";
      layer.input_grid = Dimensions(node, "input_spatial_shape", 3);
      layer.dense_shape = Dimensions(node, "output_shape", 5);
      std::vector<int> expected_shape{1, input_channels};
      expected_shape.insert(expected_shape.end(), layer.input_grid.begin(),
                            layer.input_grid.end());
      Require(layer.input_grid == grids.at(node.input(0)) &&
                  layer.dense_shape == expected_shape,
              "ScatterDense shape mismatch");
      dense_volume = Volume(std::vector<int64_t>(layer.dense_shape.begin(),
                                                 layer.dense_shape.end()));
      dense_shapes[layer.output] = layer.dense_shape;
      output_channels = 0;
    } else if (layer.op == "Transpose") {
      Require(node.input_size() == 1 && input_channels == 0 &&
                  dense_shapes.count(layer.inputs[0]),
              "Transpose requires a dense input");
      const auto& input_shape = dense_shapes.at(layer.inputs[0]);
      layer.transpose = TransposePermutation(node, input_shape.size());
      std::vector<bool> seen(input_shape.size(), false);
      layer.dense_shape.resize(input_shape.size());
      for (size_t axis = 0; axis < input_shape.size(); ++axis) {
        const int input_axis = layer.transpose[axis];
        Require(static_cast<size_t>(input_axis) < input_shape.size() &&
                    !seen[input_axis],
                "invalid Transpose permutation");
        seen[input_axis] = true;
        layer.dense_shape[axis] = input_shape[input_axis];
      }
      dense_shapes[layer.output] = layer.dense_shape;
      output_channels = 0;
    } else if (layer.op == "Reshape") {
      Require(node.input_size() == 1 && input_channels == 0 &&
                  dense_shapes.count(layer.inputs[0]),
              "only dense Reshape is supported");
      layer.dense_shape = Dimensions(node, "dims", 4);
      Require(std::vector<int64_t>(layer.dense_shape.begin(),
                                   layer.dense_shape.end()) ==
                      result.output_shape &&
                  Volume(std::vector<int64_t>(
                      dense_shapes.at(layer.inputs[0]).begin(),
                      dense_shapes.at(layer.inputs[0]).end())) ==
                      Volume(std::vector<int64_t>(layer.dense_shape.begin(),
                                                  layer.dense_shape.end())) &&
                  dense_volume ==
                      Volume(std::vector<int64_t>(result.output_shape.begin(),
                                                  result.output_shape.end())),
              "Reshape does not match graph output");
      dense_shapes[layer.output] = layer.dense_shape;
      output_channels = 0;
    } else {
      throw std::runtime_error("Unsupported spconv ONNX operator: " + layer.op);
    }
    channels.emplace(layer.output, output_channels);
    result.layers.push_back(std::move(layer));
  }
  Require(result.layers.back().op == "Reshape" &&
              result.layers.back().output == result.output,
          "expected final dense output Reshape");
  return result;
}
