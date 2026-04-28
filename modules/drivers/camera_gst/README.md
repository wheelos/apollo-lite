## camera_gst

`camera_gst` reads multiple camera inputs, stitches them into a single frame,
publishes an Apollo `drivers::Image`, and isolates an optional streaming path so
slow downstream viewers do not block the capture loop.

### Design summary

1. Capture each source independently with OpenCV/V4L2-friendly ingestion.
2. Stitch frames into a deterministic grid layout.
3. Feed the stitched `bgr` frame into an in-process GStreamer pipeline through
   `appsrc`.
4. Split the pipeline with `tee`: a static publish branch terminates at
   `appsink`, and an optional dynamic stream branch is attached / detached with
   request pads and idle pad probes.
5. Publish `rgb8` on Cyber from the static publish branch.

The runtime path is now fully in-process. The stream branch is described by
config and can use software elements on generic Linux hosts or Jetson NVENC
elements on NVIDIA hardware.

### Output channel

- `/apollo/sensor/camera/stitched/image`

### Start the driver

```bash
cd /apollo && cyber_launch start modules/drivers/camera_gst/launch/camera_gst.launch
```

### Config notes

- `sources[*].uri` can be a V4L2 node such as `/dev/video0` or a numeric camera
  index.
- `layout_slots[*]` maps each source to one cell in a grid.
- `stream.branch_pipeline` should begin with a `queue` and contain the encoder,
  payloader, and sink chain for the dynamic stream branch.
- `stream.force_keyframe_on_attach` sends a `GstForceKeyUnit` event after the
  stream branch is linked.
- The repo expects GStreamer development packages to be installed under `/usr`
  so the `@gstreamer` local repository can resolve headers and libraries.
- On Jetson, replace the sample `x264enc` branch with `nvv4l2h264enc` or
  `nvv4l2h265enc` when those plugins are available.
