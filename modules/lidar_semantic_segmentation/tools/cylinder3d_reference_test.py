# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Generate a small, untrained upstream-network oracle for native C++ tests.

Runs the supplied Cylinder3D source with test-only dense PyTorch replacements
for spconv. This checks network algebra, not upstream CUDA kernels or quality.
"""

import argparse
import hashlib
import importlib
import json
import sys
import types
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as functional
import yaml

from modules.lidar_semantic_segmentation.tools.export_cylinder3d import (
    SEMANTICKITTI_IDS,
    export_weights,
)


def triple(value):
    return (value,) * 3 if isinstance(value, int) else tuple(value)


class DenseSparseTensor:
    def __init__(self, features, indices, spatial_shape, batch_size):
        self.features = features
        self.indices = indices.long()
        self.spatial_shape = tuple(spatial_shape)
        self.batch_size = batch_size
        self.relations = {}

    def dense(self):
        result = self.features.new_zeros(
            self.batch_size, self.features.shape[1], *self.spatial_shape
        )
        b, a, c, d = self.indices.T
        result[b, :, a, c, d] = self.features
        return result

    def replace_feature(self, features):
        result = DenseSparseTensor(
            features, self.indices, self.spatial_shape, self.batch_size
        )
        result.relations = self.relations
        return result


class DenseSparseConv(torch.nn.Module):
    mode = "subm"

    def __init__(
        self,
        in_channels,
        out_channels,
        kernel_size,
        stride=1,
        padding=0,
        bias=True,
        indice_key=None,
    ):
        super().__init__()
        self.kernel = triple(kernel_size)
        self.stride = triple(stride)
        self.padding = triple(padding)
        self.indice_key = indice_key
        self.weight = torch.nn.Parameter(
            torch.empty(out_channels, *self.kernel, in_channels)
        )
        self.bias = torch.nn.Parameter(torch.zeros(out_channels)) if bias else None

    def forward(self, value):
        dense = value.dense()
        weight = self.weight
        kernel = self.kernel
        padding = self.padding
        if self.mode == "subm" and LEGACY_INDICES:
            cache_key = ("subm_kernel", self.indice_key)
            kernel, padding = value.relations.setdefault(cache_key, (kernel, padding))
            weight = weight.reshape(weight.shape[0], -1, weight.shape[-1])
            weight = weight[:, : np.prod(kernel)].reshape(
                weight.shape[0], *kernel, weight.shape[-1]
            )
        if self.mode == "inverse":
            indices, shape, stride, padding = value.relations[self.indice_key]
            base = [
                (dense.shape[axis + 2] - 1) * stride[axis]
                - 2 * padding[axis]
                + self.kernel[axis]
                for axis in range(3)
            ]
            result = functional.conv_transpose3d(
                dense,
                self.weight.permute(4, 0, 1, 2, 3),
                self.bias,
                stride,
                padding,
                tuple(s - b for s, b in zip(shape, base)),
            )
        else:
            result = functional.conv3d(
                dense,
                weight.permute(0, 4, 1, 2, 3),
                self.bias,
                self.stride,
                padding,
            )
            shape = tuple(result.shape[2:])
            if self.mode == "subm":
                indices = value.indices
            else:
                mask = dense.new_zeros(value.batch_size, 1, *value.spatial_shape)
                b, a, c, d = value.indices.T
                mask[b, 0, a, c, d] = 1
                mask = functional.conv3d(
                    mask,
                    mask.new_ones(1, 1, *self.kernel),
                    stride=self.stride,
                    padding=self.padding,
                )
                indices = torch.nonzero(mask[:, 0] > 0)
                value.relations[self.indice_key] = (
                    value.indices,
                    value.spatial_shape,
                    self.stride,
                    self.padding,
                )
        b, a, c, d = indices.T
        output = DenseSparseTensor(
            result[b, :, a, c, d], indices, shape, value.batch_size
        )
        output.relations = value.relations
        return output


class DenseDownConv(DenseSparseConv):
    mode = "down"


class DenseInverseConv(DenseSparseConv):
    mode = "inverse"


def upstream_modules(source):
    spconv = types.ModuleType("spconv")
    spconv.SubMConv3d = DenseSparseConv
    spconv.SparseConv3d = DenseDownConv
    spconv.SparseInverseConv3d = DenseInverseConv
    spconv.SparseConvTensor = DenseSparseTensor
    sys.modules["spconv"] = spconv
    sys.modules["spconv.pytorch"] = spconv
    sys.modules["torch_scatter"] = types.ModuleType("torch_scatter")
    sys.modules["numba"] = types.ModuleType("numba")
    sys.path.insert(0, str(source))
    backbone = importlib.import_module("network.segmentator_3d_asymm_spconv")
    generator = importlib.import_module("network.cylinder_fea_generator")
    return backbone, generator


def reference_input(points, config, flip):
    xyz = points[:, :3].copy()
    xyz[:, 0] *= -1 if flip & 1 else 1
    xyz[:, 1] *= -1 if flip & 2 else 1
    polar = np.stack(
        (
            np.sqrt(xyz[:, 0] ** 2 + xyz[:, 1] ** 2),
            np.arctan2(xyz[:, 1], xyz[:, 0]),
            xyz[:, 2],
        ),
        axis=1,
    )
    minimum = np.asarray(config["dataset_params"]["min_volume_space"])
    maximum = np.asarray(config["dataset_params"]["max_volume_space"])
    shape = np.asarray(config["model_params"]["output_shape"])
    intervals = (maximum - minimum) / (shape - 1)
    grid = np.floor((np.clip(polar, minimum, maximum) - minimum) / intervals)
    grid = grid.astype(np.int64)
    centers = (grid.astype(np.float32) + 0.5) * intervals + minimum
    features = np.concatenate(
        (polar - centers, polar, xyz[:, :2], points[:, 3:4] / np.float32(255)),
        axis=1,
    ).astype(np.float32)
    return torch.from_numpy(features), torch.from_numpy(grid)


LEGACY_INDICES = False


def generate(source, output, indice_semantics="independent"):
    global LEGACY_INDICES
    LEGACY_INDICES = indice_semantics == "spconv1-shared"
    torch.set_num_threads(1)
    torch.manual_seed(7019)
    with (source / "config/semantickitti.yaml").open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    config["model_params"].update(
        output_shape=[16, 16, 8], init_size=2, num_input_features=4, out_fea_dim=8
    )
    backbone_module, generator_module = upstream_modules(source)
    generator = generator_module.cylinder_fea(
        [16, 16, 8], fea_dim=9, out_pt_fea_dim=8, fea_compre=4
    )
    backbone = backbone_module.Asymm_3d_spconv(
        [16, 16, 8], num_input_features=4, init_size=2, nclasses=20
    )
    model = torch.nn.Module()
    model.add_module("cylinder_3d_generator", generator)
    model.add_module("cylinder_3d_spconv_seg", backbone)
    with torch.no_grad():
        for name, value in model.state_dict().items():
            if name.endswith("num_batches_tracked"):
                continue
            if name.endswith("running_var"):
                value.uniform_(0.7, 1.3)
            elif name.endswith(".weight") and value.ndim == 1:
                value.uniform_(0.8, 1.2)
            else:
                value.uniform_(-0.08, 0.08)
    model.eval()
    points = np.asarray(
        [
            [1.0, 0.2, -1.0, 51],
            [1.01, 0.21, -1.01, 128],
            [-2.0, 1.3, 0.2, 255],
            [3.0, -2.0, -0.5, 0],
            [-1.0, -0.4, 1.0, 77],
            [52.0, 0.0, 4.0, 255],
        ],
        dtype=np.float32,
    )
    logits = []
    with torch.inference_mode():
        for vote in range(4):
            features, grid = reference_input(points, config, vote)
            unique, inverse = torch.unique(grid, dim=0, return_inverse=True)
            encoded = generator.PPmodel(features)
            pooled = encoded.new_full((len(unique), encoded.shape[1]), -torch.inf)
            pooled.scatter_reduce_(
                0, inverse[:, None].expand_as(encoded), encoded, "amax"
            )
            pooled = generator.fea_compression(pooled)
            coordinates = functional.pad(unique, (1, 0), value=0)
            dense = backbone(pooled, coordinates, 1)
            point_logits = dense[0, :, grid[:, 0], grid[:, 1], grid[:, 2]].T
            logits.append(point_logits.flatten().tolist())
    state = model.state_dict()
    layout = "spconv2"
    if LEGACY_INDICES:
        layout = "spconv1"
        state = {
            name: tensor.permute(1, 2, 3, 4, 0) if tensor.ndim == 5 else tensor
            for name, tensor in state.items()
        }
        for name, tensor in state.items():
            if ".upBlock" in name and name.endswith(".conv3.weight"):
                tensor = tensor.contiguous()
                tensor.reshape(-1, *tensor.shape[-2:])[9:] = torch.nan
                state[name] = tensor
    mapping = {"learning_map_inv": dict(enumerate(SEMANTICKITTI_IDS))}
    source_hash = hashlib.sha256(
        (source / "network/segmentator_3d_asymm_spconv.py").read_bytes()
        + (source / "network/cylinder_fea_generator.py").read_bytes()
    ).hexdigest()
    provenance = {
        "kind": "untrained-dense-reference",
        "seed": 7019,
        "upstream_network_sha256": source_hash,
    }
    output.mkdir(parents=True, exist_ok=False)
    if LEGACY_INDICES:
        for semantics, tensors in (
            ("independent", state),
            ("spconv1-shared", dict(state)),
        ):
            if semantics == "spconv1-shared":
                name = "cylinder_3d_spconv_seg.upBlock0.conv3.weight"
                tensors[name] = tensors[name].clone()
                tensors[name].flatten()[0] = torch.nan
            try:
                export_weights(
                    tensors,
                    config,
                    mapping,
                    layout,
                    output / "must-not-export.ts",
                    provenance,
                    semantics,
                )
            except ValueError as error:
                if "non-finite checkpoint parameter" not in str(error):
                    raise
            else:
                raise AssertionError("non-finite active weights must fail export")
    export_weights(
        state,
        config,
        mapping,
        layout,
        output / "model.ts",
        provenance,
        indice_semantics,
    )
    legacy = {
        name: (
            tensor.permute(1, 2, 3, 4, 0)
            if tensor.ndim == 5 and not LEGACY_INDICES
            else tensor
        )
        for name, tensor in state.items()
    }
    export_weights(
        legacy,
        config,
        mapping,
        "spconv1",
        output / "legacy.ts",
        provenance,
        indice_semantics,
    )
    incomplete = dict(state)
    incomplete.pop("cylinder_3d_generator.PPmodel.0.running_var")
    export_weights(
        incomplete,
        config,
        mapping,
        layout,
        output / "incomplete.ts",
        provenance,
        indice_semantics,
    )
    state["unexpected.weight"] = torch.zeros(1)
    export_weights(
        state,
        config,
        mapping,
        layout,
        output / "unexpected.ts",
        provenance,
        indice_semantics,
    )
    with (output / "reference.json").open("x", encoding="utf-8") as stream:
        json.dump({"points": points.tolist(), "logits": logits, **provenance}, stream)
    print(json.dumps(provenance, sort_keys=True))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--indice-semantics",
        choices=("independent", "spconv1-shared"),
        default="independent",
    )
    args = parser.parse_args()
    generate(args.source_dir.resolve(strict=True), args.output, args.indice_semantics)


if __name__ == "__main__":
    main()
