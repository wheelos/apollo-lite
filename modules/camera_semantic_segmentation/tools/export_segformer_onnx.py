#!/usr/bin/env python3
import argparse
import os
import sys
from functools import partial


def parse_args():
    parser = argparse.ArgumentParser(
        description="Export SegFormer model to ONNX for Apollo Lite"
    )
    parser.add_argument(
        "--source-dir",
        type=str,
        default="/home/humble/01code/SegFormer",
        help="Path to SegFormer repository",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to SegFormer config file (e.g. local_configs/segformer/B0/segformer.b0.512x1024.city.160k.py)",
    )
    parser.add_argument(
        "--checkpoint",
        type=str,
        default=None,
        help="Path to model checkpoint (.pth, .bin, or .safetensors file)",
    )
    parser.add_argument(
        "--num-classes",
        type=int,
        default=150,
        help="Number of semantic segmentation classes",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output ONNX file path",
    )
    parser.add_argument(
        "--height",
        type=int,
        default=512,
        help="Input image height",
    )
    parser.add_argument(
        "--width",
        type=int,
        default=512,
        help="Input image width",
    )
    parser.add_argument(
        "--interpolate",
        action="store_true",
        default=True,
        help="Bilinearly interpolate output logits to (height, width)",
    )
    parser.add_argument(
        "--opset-version",
        type=int,
        default=13,
        help="ONNX opset version",
    )
    return parser.parse_args()


def convert_sync_batchnorm(module):
    import torch

    module_output = module
    if isinstance(module, torch.nn.SyncBatchNorm):
        module_output = torch.nn.BatchNorm2d(
            module.num_features,
            module.eps,
            module.momentum,
            module.affine,
            module.track_running_stats,
        )
        if module.affine:
            module_output.weight.data = module.weight.data.clone().detach()
            module_output.bias.data = module.bias.data.clone().detach()
            module_output.weight.requires_grad = module.weight.requires_grad
            module_output.bias.requires_grad = module.bias.requires_grad
        module_output.running_mean = module.running_mean
        module_output.running_var = module.running_var
        module_output.num_batches_tracked = module.num_batches_tracked
    for name, child in module.named_children():
        module_output.add_module(name, convert_sync_batchnorm(child))
    del module
    return module_output


def main():
    args = parse_args()
    import torch
    import torch.nn.functional as F

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    input_shape = (1, 3, args.height, args.width)
    dummy_input = torch.randn(*input_shape)

    # Check if we should load HuggingFace / safetensors / pytorch_model.bin model
    if args.config is None or not os.path.exists(args.config):
        from transformers import SegformerForSemanticSegmentation, SegformerConfig

        config = SegformerConfig(
            num_labels=args.num_classes,
            depths=[2, 2, 2, 2],
            hidden_sizes=[32, 64, 160, 256],
            decoder_hidden_size=256,
        )
        base_model = SegformerForSemanticSegmentation(config)
        if args.checkpoint and os.path.exists(args.checkpoint):
            if args.checkpoint.endswith(".safetensors"):
                from safetensors.torch import load_file
                state_dict = load_file(args.checkpoint)
            else:
                state_dict = torch.load(args.checkpoint, map_location="cpu")
            base_model.load_state_dict(state_dict, strict=False)

        class ModelWrapper(torch.nn.Module):
            def __init__(self, model, output_size, do_interp):
                super().__init__()
                self.model = model
                self.output_size = output_size
                self.do_interp = do_interp

            def forward(self, x):
                out = self.model(x)
                logits = out.logits
                if self.do_interp and self.output_size is not None:
                    logits = F.interpolate(
                        logits,
                        size=self.output_size,
                        mode="bilinear",
                        align_corners=False,
                    )
                return logits

        model = ModelWrapper(
            base_model,
            output_size=(args.height, args.width),
            do_interp=args.interpolate,
        )
        model.eval()

        with torch.no_grad():
            torch.onnx.export(
                model,
                dummy_input,
                args.output,
                input_names=["input"],
                output_names=["output"],
                export_params=True,
                opset_version=args.opset_version,
            )
        print(f"Successfully exported SegFormer ONNX model to: {args.output}")
        return

    if os.path.isdir(args.source_dir):
        sys.path.insert(0, args.source_dir)

    import mmcv
    from mmcv.onnx import register_extra_symbolics
    from mmcv.runner import load_checkpoint
    from mmseg.models import build_segmentor

    cfg = mmcv.Config.fromfile(args.config)
    cfg.model.pretrained = None
    cfg.model.train_cfg = None

    segmentor = build_segmentor(
        cfg.model, train_cfg=None, test_cfg=cfg.get("test_cfg")
    )
    segmentor = convert_sync_batchnorm(segmentor)

    if args.checkpoint and os.path.exists(args.checkpoint):
        load_checkpoint(segmentor, args.checkpoint, map_location="cpu")

    segmentor.cpu().eval()

    img_metas = [
        [
            {
                "img_shape": (args.height, args.width, 3),
                "ori_shape": (args.height, args.width, 3),
                "pad_shape": (args.height, args.width, 3),
                "filename": "<dummy>.png",
                "scale_factor": 1.0,
                "flip": False,
            }
        ]
    ]

    origin_forward = segmentor.forward
    segmentor.forward = partial(
        segmentor.forward, img_metas=img_metas, return_loss=False
    )

    register_extra_symbolics(args.opset_version)

    with torch.no_grad():
        torch.onnx.export(
            segmentor,
            ([dummy_input],),
            args.output,
            input_names=["input"],
            output_names=["output"],
            export_params=True,
            keep_initializers_as_inputs=True,
            opset_version=args.opset_version,
        )

    segmentor.forward = origin_forward
    print(f"Successfully exported SegFormer ONNX model to: {args.output}")


if __name__ == "__main__":
    main()
