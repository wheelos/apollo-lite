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

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "onnx-ml.pb.h"  // NOLINT(build/include_subdir)

namespace {

void Int(onnx::NodeProto* node, const std::string& name, int value) {
  auto* attribute = node->add_attribute();
  attribute->set_name(name);
  attribute->set_type(onnx::AttributeProto::INT);
  attribute->set_i(value);
}

void String(onnx::NodeProto* node, const std::string& name,
            const std::string& value) {
  auto* attribute = node->add_attribute();
  attribute->set_name(name);
  attribute->set_type(onnx::AttributeProto::STRING);
  attribute->set_s(value);
}

void Dims(onnx::NodeProto* node, const std::string& name,
          std::initializer_list<int> values) {
  auto* attribute = node->add_attribute();
  attribute->set_name(name);
  attribute->set_type(onnx::AttributeProto::INTS);
  for (int value : values) attribute->add_ints(value);
}

void SetDims(onnx::NodeProto* node, const std::string& name,
             std::initializer_list<int> values) {
  for (auto& attribute : *node->mutable_attribute()) {
    if (attribute.name() == name) {
      attribute.clear_ints();
      for (int value : values) attribute.add_ints(value);
      return;
    }
  }
  ADD_FAILURE() << "Missing attribute " << name;
}

void SetString(onnx::NodeProto* node, const std::string& name,
               const std::string& value) {
  for (auto& attribute : *node->mutable_attribute()) {
    if (attribute.name() == name) {
      attribute.set_s(value);
      return;
    }
  }
  ADD_FAILURE() << "Missing attribute " << name;
}

class SparseModelTest : public testing::Test {
 protected:
  void SetUp() override {
    std::string temporary = testing::TempDir() + "/spconv-model-XXXXXX";
    const int descriptor = mkstemp(temporary.data());
    ASSERT_GE(descriptor, 0);
    ASSERT_EQ(close(descriptor), 0);
    path_ = temporary;
    auto* graph = model_.mutable_graph();
    auto* input = graph->add_input();
    input->set_name("input");
    auto* type = input->mutable_type()->mutable_tensor_type();
    type->set_elem_type(onnx::TensorProto::FLOAT16);
    type->mutable_shape()->add_dim()->set_dim_value(1);
    type->mutable_shape()->add_dim()->set_dim_value(5);
    auto* output = graph->add_output();
    output->set_name("output");
    type = output->mutable_type()->mutable_tensor_type();
    type->set_elem_type(onnx::TensorProto::FLOAT16);
    for (int value : {1, 256, 180, 180}) {
      type->mutable_shape()->add_dim()->set_dim_value(value);
    }
    auto* weight = graph->add_initializer();
    weight->set_name("weight");
    weight->set_data_type(onnx::TensorProto::FLOAT16);
    for (int value : {128, 1, 1, 1, 5}) weight->add_dims(value);
    weight->set_raw_data(std::string(128 * 5 * 2, '\0'));
    auto* bias = graph->add_initializer();
    bias->set_name("bias");
    bias->set_data_type(onnx::TensorProto::FLOAT16);
    bias->add_dims(128);
    bias->set_raw_data(std::string(128 * 2, '\0'));
    auto* conv = graph->add_node();
    conv->set_op_type("SparseConvolution");
    for (const auto& name : {"input", "weight", "bias"}) conv->add_input(name);
    conv->add_output("conv");
    Int(conv, "in_channels", 5);
    Int(conv, "out_channels", 128);
    Int(conv, "output_bound", 200000);
    Int(conv, "subm", 1);
    for (const auto& name : {"kernel_size", "stride", "dilation"}) {
      Dims(conv, name, {1, 1, 1});
    }
    Dims(conv, "padding", {0, 0, 0});
    Dims(conv, "input_spatial_shape", {2, 180, 180});
    Dims(conv, "output_spatial_shape", {2, 180, 180});
    auto* scatter = graph->add_node();
    scatter->set_op_type("ScatterDense");
    scatter->add_input("conv");
    scatter->add_output("dense");
    auto* format = scatter->add_attribute();
    format->set_name("format");
    format->set_type(onnx::AttributeProto::STRING);
    format->set_s("zyx");
    Dims(scatter, "input_spatial_shape", {2, 180, 180});
    Dims(scatter, "output_shape", {1, 128, 2, 180, 180});
    auto* transpose = graph->add_node();
    transpose->set_op_type("Transpose");
    transpose->add_input("dense");
    transpose->add_output("transposed");
    Dims(transpose, "perm", {0, 1, 2, 3, 4});
    auto* reshape = graph->add_node();
    reshape->set_op_type("Reshape");
    reshape->add_input("transposed");
    reshape->add_output("output");
    Dims(reshape, "dims", {1, 256, 180, 180});
  }

  void TearDown() override {
    if (!path_.empty()) {
      EXPECT_EQ(std::remove(path_.c_str()), 0);
    }
  }

  void Save() {
    std::ofstream output(path_, std::ios::binary);
    ASSERT_TRUE(model_.SerializeToOstream(&output));
  }

  onnx::ModelProto model_;
  std::string path_;
};

TEST_F(SparseModelTest, LoadsFp16GraphWithoutCuda) {
  Save();
  const auto model = LoadSparseModel(path_);
  ASSERT_EQ(model.layers.size(), 4);
  EXPECT_EQ(model.layers[0].weight_data.size(), 128 * 5);
  EXPECT_EQ(model.layers[0].bias_data.size(), 128);
  EXPECT_EQ(model.layers[3].dense_shape, std::vector<int>({1, 256, 180, 180}));
}

TEST_F(SparseModelTest, LoadsXyzSparseLayout) {
  auto* graph = model_.mutable_graph();
  SetDims(graph->mutable_node(0), "input_spatial_shape", {180, 180, 2});
  SetDims(graph->mutable_node(0), "output_spatial_shape", {180, 180, 2});
  SetString(graph->mutable_node(1), "format", "xyz");
  SetDims(graph->mutable_node(1), "input_spatial_shape", {180, 180, 2});
  SetDims(graph->mutable_node(1), "output_shape", {1, 128, 180, 180, 2});
  SetDims(graph->mutable_node(2), "perm", {0, 1, 4, 2, 3});

  Save();
  const auto model = LoadSparseModel(path_);
  ASSERT_EQ(model.layers.size(), 4);
  EXPECT_TRUE(model.layers[1].xyz);
  EXPECT_EQ(model.layers[1].input_grid, std::vector<int>({180, 180, 2}));
  EXPECT_EQ(model.layers[1].dense_shape,
            std::vector<int>({1, 128, 180, 180, 2}));
  EXPECT_EQ(model.layers[2].transpose, std::vector<int>({0, 1, 4, 2, 3}));
  EXPECT_EQ(model.layers[2].dense_shape,
            std::vector<int>({1, 128, 2, 180, 180}));
}

TEST_F(SparseModelTest, UsesDefaultTransposePermutationWhenPermIsOmitted) {
  model_.mutable_graph()->mutable_node(2)->clear_attribute();
  Save();
  const auto model = LoadSparseModel(path_);
  EXPECT_EQ(model.layers[2].transpose, std::vector<int>({4, 3, 2, 1, 0}));
  EXPECT_EQ(model.layers[2].dense_shape,
            std::vector<int>({180, 180, 2, 128, 1}));
}

TEST_F(SparseModelTest, AcceptsLegacyTransposeDimensions) {
  auto* transpose = model_.mutable_graph()->mutable_node(2);
  transpose->mutable_attribute(0)->set_name("dims");
  Save();
  const auto model = LoadSparseModel(path_);
  EXPECT_EQ(model.layers[2].transpose, std::vector<int>({0, 1, 2, 3, 4}));
}

TEST_F(SparseModelTest, AcceptsInt8PrecisionHintsForFp16Execution) {
  String(model_.mutable_graph()->mutable_node(0), "precision", "int8");
  String(model_.mutable_graph()->mutable_node(0), "output_precision", "int8");
  Save();
  const auto model = LoadSparseModel(path_);
  EXPECT_EQ(model.input_channels, 5);
  EXPECT_EQ(model.layers[0].weight_data.size(), 128 * 5);
}

TEST_F(SparseModelTest, RejectsUnsupportedOperator) {
  model_.mutable_graph()->mutable_node(1)->set_op_type("Sigmoid");
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsNonFp16Graph) {
  model_.mutable_graph()
      ->mutable_input(0)
      ->mutable_type()
      ->mutable_tensor_type()
      ->set_elem_type(onnx::TensorProto::FLOAT);
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsInvalidGraphShape) {
  model_.mutable_graph()
      ->mutable_output(0)
      ->mutable_type()
      ->mutable_tensor_type()
      ->mutable_shape()
      ->mutable_dim(1)
      ->set_dim_value(128);
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsTruncatedWeights) {
  model_.mutable_graph()->mutable_initializer(0)->set_raw_data("bad");
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsDuplicateInitializer) {
  auto copy = model_.graph().initializer(0);
  *model_.mutable_graph()->add_initializer() = copy;
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsNonzeroOutputPadding) {
  Dims(model_.mutable_graph()->mutable_node(0), "output_padding", {1, 0, 0});
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsDuplicateAttribute) {
  Int(model_.mutable_graph()->mutable_node(0), "subm", 1);
  Save();
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

TEST_F(SparseModelTest, RejectsMalformedFile) {
  std::ofstream(path_, std::ios::binary) << "not protobuf";
  EXPECT_THROW(LoadSparseModel(path_), std::runtime_error);
}

}  // namespace
