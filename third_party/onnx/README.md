# ONNX schemas

These existing schemas were moved unchanged from `third_party/spconv/onnx`
when the NVIDIA spconv integration was removed. They describe the serialized
model, not a sparse-convolution implementation. Preserve the ONNX copyright
and license notices embedded in each file.

`//third_party/onnx:onnx_proto_cc` supplies host-side protobuf parsing for the
CenterPoint model loader. The loader interprets the export's custom sparse
operators and executes them through the registry module `spconv@2.3.8.bcr.1`.
