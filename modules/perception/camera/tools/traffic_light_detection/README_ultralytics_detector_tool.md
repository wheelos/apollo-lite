`ultralytics_detector_tool` runs the integrated `TrafficLightUltralyticsDetector`
directly on a single image and saves an annotated JPG.

Example:

`bazel-bin/modules/perception/camera/tools/traffic_light_detection/ultralytics_detector_tool \
  --image_path=/tmp/frame.jpg \
  --output_path=/tmp/frame_det.jpg \
  --config_path=/apollo/modules/perception/pipeline/config/trafficlights_perception_ultralytics_efficientnet.pb.txt \
  --num_dummy_lights=32`
