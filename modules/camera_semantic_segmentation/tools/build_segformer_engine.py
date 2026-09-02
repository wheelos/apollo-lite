#!/usr/bin/env python3
import argparse
import os
import sys


def parse_args():
    parser = argparse.ArgumentParser(
        description="Build TensorRT engine from SegFormer ONNX"
    )
    parser.add_argument(
        "--onnx",
        type=str,
        required=True,
        help="Path to input ONNX model",
    )
    parser.add_argument(
        "--engine",
        type=str,
        required=True,
        help="Path to output TensorRT engine",
    )
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Enable FP16 precision",
    )
    parser.add_argument(
        "--workspace",
        type=int,
        default=2048,
        help="TensorRT builder workspace size in MB",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    import tensorrt as trt

    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    network_flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(network_flags)
    parser = trt.OnnxParser(network, logger)

    if not parser.parse_from_file(args.onnx):
        print("Failed to parse the ONNX file.")
        for error in range(parser.num_errors):
            print(parser.get_error(error))
        sys.exit(1)

    config = builder.create_builder_config()
    if hasattr(config, "set_memory_pool_limit"):
        config.set_memory_pool_limit(
            trt.MemoryPoolType.WORKSPACE, args.workspace * (1024 * 1024)
        )
    else:
        config.max_workspace_size = args.workspace * (1024 * 1024)

    if args.fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)

    os.makedirs(os.path.dirname(os.path.abspath(args.engine)), exist_ok=True)
    print(f"Building TensorRT serialized engine: {args.engine} ...")
    serialized_engine = builder.build_serialized_network(network, config)
    if serialized_engine is None:
        print("Failed to build serialized engine.")
        sys.exit(1)

    with open(args.engine, "wb") as f:
        f.write(bytes(serialized_engine))

    print(f"Successfully built TensorRT engine: {args.engine}")


if __name__ == "__main__":
    main()
