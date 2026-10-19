## Camera

The `camera` package is based on V4L USB camera devices, providing image
acquisition and publishing functions. This driver uses one telephoto camera and
one wide-angle camera.

The driver reads V4L2 frames, converts supported packed YUV422 or MJPEG input,
and publishes Cyber `Image` messages as `yuyv` or `rgb8`. The compression
component accepts `yuyv`, `rgb8`, or `bgr8` images and publishes JPEG output.
Unit tests cover packed YUV format normalization and RGB conversion; the
camera-device test injects a fake capture device to verify frame metadata,
payload, and buffer requeue behavior without physical hardware.

### Output channels

- /apollo/sensor/camera/front_12mm/image
- /apollo/sensor/camera/front_6mm/image
- /apollo/sensor/camera/front_fisheye/image
- /apollo/sensor/camera/left_fisheye/image
- /apollo/sensor/camera/right_fisheye/image
- /apollo/sensor/camera/rear_6mm/image

### Start the camera driver

**Please modify and confirm that the parameters in the launch file match the
actual vehicle configuration.**

```bash
# in docker
bash /apollo/scripts/camera.sh
# or
cd /apollo && cyber_launch start modules/drivers/camera/launch/camera.launch
```

### Start the camera + video compression driver

**Please modify and confirm that the parameters in the launch file match the
actual vehicle configuration.**

```bash
# in docker
bash /apollo/scripts/camera_and_video.sh
# or
cd /apollo && cyber_launch start modules/drivers/camera/launch/camera_and_video.launch
```

### Common Issues

1. If you encounter the error "sh: 1: v4l2-ctl: not found", you need to install
   the v4l2 library.

```bash
sudo apt-get install v4l-utils
```
