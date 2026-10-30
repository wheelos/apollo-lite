# Copyright 2026 WheelOS. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0
"""Export trusted Cylinder3D/PVKD weights for the native C++ implementation.

The archive contains tensors and metadata, not a traced sparse execution graph.
No upstream training or spconv code is imported or needed during export.
"""

import argparse
import hashlib
import io
import json
import subprocess
from pathlib import Path

import torch
import yaml

SEMANTICKITTI_IDS = [
    0,
    10,
    11,
    15,
    18,
    20,
    30,
    31,
    32,
    40,
    44,
    48,
    49,
    50,
    51,
    70,
    71,
    72,
    80,
    81,
]


class WeightArchive(torch.nn.Module):
    def __init__(self, tensors):
        super().__init__()
        for name, tensor in tensors.items():
            self.register_buffer(name, tensor)

    def forward(self, value):
        return value


def effective_kernel(name):
    if not name.startswith("cylinder_3d_spconv_seg.") or not name.endswith(".weight"):
        return None
    layer = name[len("cylinder_3d_spconv_seg.") : -len(".weight")]
    block, _, operation = layer.partition(".")
    if operation not in ("conv1", "conv1_2", "conv1_3", "conv2", "conv3"):
        return None
    if block == "downCntx" or block.startswith("upBlock"):
        return (1, 3, 3)
    if block.startswith("resBlock"):
        return (3, 1, 3)
    if block == "ReconNet":
        return (3, 1, 1)
    return None


def export_weights(
    state,
    config,
    mapping,
    layout,
    output,
    provenance,
    indice_semantics="independent",
):
    model = config["model_params"]
    dataset = config["dataset_params"]
    if (
        model["model_architecture"] != "cylinder_asym"
        or model["fea_dim"] != 9
        or model["num_class"] != 20
        or dataset["dataset_type"] != "cylinder_dataset"
        or dataset["fixed_volume_space"] is not True
        or dataset["ignore_label"] != 0
        or mapping["learning_map_inv"] != dict(enumerate(SEMANTICKITTI_IDS))
    ):
        raise ValueError(
            "only fixed-volume single-scan SemanticKITTI models are supported"
        )
    if layout not in ("spconv1", "spconv2"):
        raise ValueError("weight layout must be explicitly spconv1 or spconv2")
    if indice_semantics not in ("independent", "spconv1-shared"):
        raise ValueError("unsupported indice semantics")
    if indice_semantics == "spconv1-shared" and layout != "spconv1":
        raise ValueError("legacy shared indices require spconv1 checkpoint layout")
    if isinstance(state, dict) and "state_dict" in state:
        state = state["state_dict"]
    if not isinstance(state, dict) or not state:
        raise ValueError("checkpoint must be a non-empty state_dict")
    if all(name.startswith("module.") for name in state):
        state = {name[len("module.") :]: value for name, value in state.items()}
    buffers = {}
    names = {}
    unused_parameters = {}
    for name, tensor in sorted(state.items()):
        if name.endswith(".num_batches_tracked"):
            continue
        if not isinstance(tensor, torch.Tensor) or not tensor.is_floating_point():
            raise ValueError(f"non-floating checkpoint parameter: {name}")
        if tensor.ndim == 5:
            if not name.startswith("cylinder_3d_spconv_seg.") or not name.endswith(
                ".weight"
            ):
                raise ValueError(f"unrecognized sparse kernel: {name}")
            if layout == "spconv2":
                tensor = tensor.permute(1, 2, 3, 4, 0)
            kernel = (
                effective_kernel(name) if indice_semantics == "spconv1-shared" else None
            )
            if kernel is not None:
                # spconv 1.2.1 iterates cached indiceNum.size(0), not the
                # current filter's kernel volume. Only these planes execute.
                active = kernel[0] * kernel[1] * kernel[2]
                planes = tensor.reshape(-1, tensor.shape[-2], tensor.shape[-1])
                if len(planes) < active:
                    raise ValueError(
                        f"checkpoint has fewer planes than cached indices: {name}"
                    )
                if len(planes) > active:
                    unused_parameters[name] = {
                        "unused_values": planes[active:].numel(),
                        "unused_nonfinite": int(
                            (~torch.isfinite(planes[active:])).sum()
                        ),
                    }
                tensor = planes[:active].reshape(*kernel, *tensor.shape[-2:])
        tensor = tensor.detach().cpu().float().contiguous()
        if not torch.isfinite(tensor).all():
            raise ValueError(f"non-finite checkpoint parameter: {name}")
        buffer_name = f"tensor_{len(buffers)}"
        names[name] = buffer_name
        buffers[buffer_name] = tensor
    metadata = {
        "format": "apollo-cylinder3d-v1",
        "model": model,
        "minimum": dataset["min_volume_space"],
        "maximum": dataset["max_volume_space"],
        "class_ids": SEMANTICKITTI_IDS,
        "buffers": names,
        "provenance": provenance,
        "weight_layout": layout,
        "batch_norm_eps": 1e-5,
        "leaky_relu_slope": 0.01,
        "indice_semantics": indice_semantics,
        "unused_legacy_kernel_values": unused_parameters,
    }
    archive = torch.jit.trace(WeightArchive(buffers).eval(), torch.zeros(1))
    encoded = io.BytesIO()
    torch.jit.save(
        archive,
        encoded,
        _extra_files={"cylinder3d.json": json.dumps(metadata, sort_keys=True)},
    )
    with Path(output).open("xb") as stream:
        stream.write(encoded.getvalue())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--checkpoint-sha256")
    parser.add_argument(
        "--weight-layout", choices=("spconv1", "spconv2"), required=True
    )
    parser.add_argument(
        "--indice-semantics",
        choices=("independent", "spconv1-shared"),
        default="independent",
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    with args.config.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    mapping_path = args.source_dir / config["dataset_params"]["label_mapping"]
    with mapping_path.open(encoding="utf-8") as stream:
        mapping = yaml.safe_load(stream)
    checksum = hashlib.sha256()
    with args.checkpoint.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    digest = checksum.hexdigest()
    if args.checkpoint_sha256 and digest != args.checkpoint_sha256.lower():
        parser.error("checkpoint SHA256 mismatch")
    revision = subprocess.check_output(
        ["git", "-C", str(args.source_dir), "rev-parse", "HEAD"], text=True
    ).strip()
    provenance = {
        "checkpoint_sha256": digest,
        "source_revision": revision,
        "config_sha256": hashlib.sha256(args.config.read_bytes()).hexdigest(),
        "export_torch_version": torch.__version__,
    }
    state = torch.load(args.checkpoint, map_location="cpu", weights_only=True)
    export_weights(
        state,
        config,
        mapping,
        args.weight_layout,
        args.output,
        provenance,
        args.indice_semantics,
    )
    print(json.dumps(provenance, sort_keys=True))


if __name__ == "__main__":
    main()
