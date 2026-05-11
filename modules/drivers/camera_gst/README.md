## camera_gst

`camera_gst` builds one in-process GStreamer graph for 1-n cameras, publishes one
Apollo `sensor_msgs::Image` per selected source in `rgb8`, and can also stitch a
selected subset of those sources for optional stitched publish and GPU-native
stream output.

### Design summary

1. Build one GPU-first capture graph instead of polling each source into CPU memory.
2. Decode and normalize every source into `NVMM` memory, then fan out each source
   with `tee`.
3. Keep the stitch path on GPU with `nvcompositor`; only the Cyber publish branch
   converts to CPU `rgb8` at its final `appsink`.
4. Publish each configured source directly on its own Cyber channel without a
   software rate limiter, so cadence follows the camera.
5. Select the stitched/streamed subset by `layout_slots[*]`; sources not listed
   there can still publish independently.
6. Keep the stream branch on GPU from compositor output to encoder/sink.

The runtime path stays fully in-process. On Jetson-style deployments the intended
hot path is `nvarguscamerasrc` or `v4l2src` plus `nvv4l2decoder`, `nvvidconv`,
`nvcompositor`, and NVENC. The only required CPU crossing is the final Cyber
publish copy into `sensor_msgs::Image`.

### Output channels

- `sources[*].publish.channel_name` for each camera
- optional `publish.channel_name` for stitched output

### Start the driver

```bash
cd /apollo
source cyber/setup.bash
cyber_launch start modules/drivers/camera_gst/launch/camera_gst.launch
```

### Config notes

- `sources[*].uri` can be a V4L2 node such as `/dev/video0`, a numeric camera
  index, or `csi://0` / `argus://0` for Jetson CSI cameras.
- `sources[*].publish.channel_name` enables per-camera RGB publishing.
- `sources[*].capture_pipeline` overrides the built-in source head when a
  deployment needs a custom GPU-capable source graph. The custom graph must end
  at a raw frame stream and must not include its own sink.
- `layout_slots[*]` selects which cameras participate in stitched publish or
  streaming. It no longer has to cover all configured sources.
- `publish_rate` is intentionally not applied in the hot path; publish cadence
  follows capture cadence.
- `stream.branch_pipeline` should begin with a `queue` and consume stitched
  `video/x-raw(memory:NVMM),format=NV12`. A production branch can be NVENC + RTP,
  or a `webrtcbin` branch if the surrounding system provides signaling.
- For autonomous driving efficiency, the recommended deployment is: per-source
  Cyber publish for inference, stitched GPU branch for operator stream, and no
  CPU copies anywhere else in the graph.
- The repo expects GStreamer development packages to be installed under `/usr`
  so the `@gstreamer` local repository can resolve headers and libraries.
