# camera_gst

`camera_gst` captures V4L2 camera sources into NVMM and provides GPU-frame
callbacks and/or an optional H.264/RTP stream. It does not convert frames to
RGB or publish Cyber `Image` messages.

Source capture frame rates are configured per source with `sources.fps`.
GStreamer `videorate drop-only=true` and downstream framerate caps control the
source and stitched-output rates; the stitched output rate is configured with
the top-level `fps` field. Frame dropping is performed in the GStreamer
pipeline, not by a separate application-side limiter.

Example configurations, DAGs, and launch files are under `conf/`, `dag/`, and
`launch/`. Supported source backends are `V4L2_MMAP` and `NVV4L2_DMABUF`.
