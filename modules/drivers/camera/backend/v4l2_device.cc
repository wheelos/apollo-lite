#include "modules/drivers/camera/backend/v4l2_device.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <sstream>
#include <stdexcept>

#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace camera {

namespace {

inline void throw_v4l2_error(const std::string& msg) {
  // errno is thread-local, so capturing it here is safe
  throw std::system_error(errno, std::generic_category(), msg);
}

inline void throw_runtime_error(const std::string& msg) {
  throw std::runtime_error(msg);
}

}  // namespace

using apollo::drivers::camera::config::IOMethod;

V4L2Device::V4L2Device(ConfPtr config)
    : config_(config), fd_(-1), is_streaming_(false) {
  device_path_ = config_->camera_dev();
  io_method_ = config_->io_method();
  // Constructor only opens the device, throws exception on failure
  OpenDevice();
}

V4L2Device::~V4L2Device() noexcept {
  try {
    if (is_streaming_) {
      StopStreaming();
    }
    UninitBuffers();
  } catch (const std::exception& e) {
    // Exceptions in destructor are serious; only log, do not let them escape
    AERROR << "Caught exception during V4L2Device destruction for "
           << device_path_ << ": " << e.what();
  }

  if (fd_ != -1) {
    if (close(fd_) == -1) {
      AERROR << "Failed to close device " << device_path_ << ": "
             << strerror(errno);
    }
  }
}

void V4L2Device::OpenDevice() {
  struct stat st;
  if (stat(device_path_.c_str(), &st) == -1) {
    throw_v4l2_error("Cannot identify '" + device_path_ + "'");
  }

  if (!S_ISCHR(st.st_mode)) {
    throw_runtime_error(device_path_ + " is not a character device");
  }

  fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (fd_ == -1) {
    throw_v4l2_error("Cannot open '" + device_path_ + "'");
  }
}

int V4L2Device::xioctl(int request, void* arg) {
  int r;
  do {
    r = ioctl(fd_, request, arg);
  } while (r == -1 && errno == EINTR);
  return r;
}

void V4L2Device::Configure(uint32_t width, uint32_t height,
                           uint32_t pixel_format, int frame_rate) {
  v4l2_capability cap{};
  if (xioctl(VIDIOC_QUERYCAP, &cap) == -1) {
    throw_v4l2_error("VIDIOC_QUERYCAP failed for " + device_path_);
  }
  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    throw_runtime_error(device_path_ + " is not a video capture device");
  }

  // Check IO method support
  switch (io_method_) {
    case IOMethod::IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE))
        throw_runtime_error("Device does not support read I/O");
      break;
    case IOMethod::IO_METHOD_MMAP:
    case IOMethod::IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING))
        throw_runtime_error("Device does not support streaming I/O");
      break;
    case IOMethod::IO_METHOD_UNKNOWN:
      break;
  }

  // Set video format
  v4l2_format fmt{};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = pixel_format;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  if (xioctl(VIDIOC_S_FMT, &fmt) == -1) {
    throw_v4l2_error("VIDIOC_S_FMT failed for " + device_path_);
  }

  // Store actual values returned by the driver to internal member variables
  width_ = fmt.fmt.pix.width;
  height_ = fmt.fmt.pix.height;
  pixel_format_ = fmt.fmt.pix.pixelformat;
  buffer_size_ = fmt.fmt.pix.sizeimage;  // Buffer size calculated by driver

  // Set frame rate
  SetFrameRate(frame_rate);

  // Initialize buffers
  InitBuffers();
}

void V4L2Device::SetFrameRate(int frame_rate) {
  if (frame_rate <= 0) {
    AERROR << "Invalid frame rate: " << frame_rate << " for " << device_path_;
    return;
  }

  v4l2_streamparm stream_params{};
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(VIDIOC_G_PARM, &stream_params) == 0) {
    if (stream_params.parm.capture.capability & V4L2_CAP_TIMEPERFRAME) {
      stream_params.parm.capture.timeperframe.numerator = 1;
      stream_params.parm.capture.timeperframe.denominator = frame_rate;
      if (xioctl(VIDIOC_S_PARM, &stream_params) == -1) {
        // Warn instead of error, as some devices may not support precise frame
        // rate setting
        AWARN << "Failed to set framerate to " << frame_rate << " for "
              << device_path_;
      }
    }
  } else {
    AWARN << "Failed to get stream parameters (VIDIOC_G_PARM) for "
          << device_path_;
  }
}

bool V4L2Device::SetParameter(uint32_t id, int32_t value) {
  v4l2_queryctrl queryctrl{};
  queryctrl.id = id;
  if (xioctl(VIDIOC_QUERYCTRL, &queryctrl) == -1) {
    if (errno != EINVAL) {
      AWARN << "VIDIOC_QUERYCTRL failed for control " << id << " on "
            << device_path_;
    }
    return false;  // This control is not supported
  }
  if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    AINFO << "Control " << id << " is disabled on " << device_path_;
    return false;
  }

  v4l2_control control{};
  control.id = id;
  control.value = value;
  if (xioctl(VIDIOC_S_CTRL, &control) == -1) {
    AWARN << "VIDIOC_S_CTRL failed for control " << id << " on "
          << device_path_;
    return false;
  }
  return true;
}

void V4L2Device::InitBuffers() {
  v4l2_requestbuffers req{};
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.count = 4;  // Use a reasonable default value of 4

  switch (io_method_) {
    case IOMethod::IO_METHOD_MMAP:
      req.memory = V4L2_MEMORY_MMAP;
      if (xioctl(VIDIOC_REQBUFS, &req) == -1)
        throw_v4l2_error("VIDIOC_REQBUFS failed for MMAP");

      if (req.count < 2) {  // At least two buffers are needed for streaming
        throw_runtime_error("Insufficient buffer memory on " + device_path_);
      }

      buffers_.resize(req.count);
      for (size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (xioctl(VIDIOC_QUERYBUF, &buf) == -1)
          throw_v4l2_error("VIDIOC_QUERYBUF failed");

        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd_, buf.m.offset);
        if (buffers_[i].start == MAP_FAILED) {
          throw_v4l2_error("mmap failed");
        }
      }
      break;

    case IOMethod::IO_METHOD_USERPTR:
      req.memory = V4L2_MEMORY_USERPTR;
      if (xioctl(VIDIOC_REQBUFS, &req) == -1)
        throw_v4l2_error("VIDIOC_REQBUFS failed for USERPTR");

      buffers_.resize(req.count);
      for (auto& buf : buffers_) {
        size_t page_size = getpagesize();
        buf.length = (buffer_size_ + page_size - 1) & ~(page_size - 1);
        if (posix_memalign(&buf.start, page_size, buf.length) != 0) {
          throw_runtime_error("posix_memalign failed for userptr buffer");
        }
        memset(buf.start, 0, buf.length);
      }
      break;

    case IOMethod::IO_METHOD_READ:
      // READ method does not require pre-requesting buffers
      buffers_.resize(1);
      buffers_[0].length = buffer_size_;
      buffers_[0].start = malloc(buffer_size_);
      if (!buffers_[0].start)
        throw_runtime_error("Out of memory for read buffer");
      break;
    case IOMethod::IO_METHOD_UNKNOWN:
      AERROR << "Unknown IO method for " << device_path_;
      break;
  }
}

void V4L2Device::UninitBuffers() noexcept {
  switch (io_method_) {
    case IOMethod::IO_METHOD_MMAP:
      for (auto& buf : buffers_) {
        if (buf.start && munmap(buf.start, buf.length) == -1) {
          AERROR << "munmap failed for " << device_path_ << ": "
                 << strerror(errno);
        }
      }
      break;
    case IOMethod::IO_METHOD_READ:
    case IOMethod::IO_METHOD_USERPTR:
      for (auto& buf : buffers_) {
        free(buf.start);
      }
      break;
    case IOMethod::IO_METHOD_UNKNOWN:
      AERROR << "Unknown IO method for " << device_path_;
      break;
  }
  buffers_.clear();
}

void V4L2Device::StartStreaming() {
  if (is_streaming_) return;

  if (io_method_ == IOMethod::IO_METHOD_MMAP ||
      io_method_ == IOMethod::IO_METHOD_USERPTR) {
    for (size_t i = 0; i < buffers_.size(); ++i) {
      QueueBuffer(i);
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(VIDIOC_STREAMON, &type) == -1) {
      throw_v4l2_error("VIDIOC_STREAMON failed for " + device_path_);
    }
  }
  is_streaming_ = true;
}

void V4L2Device::StopStreaming() noexcept {
  if (!is_streaming_) return;

  if (io_method_ == IOMethod::IO_METHOD_MMAP ||
      io_method_ == IOMethod::IO_METHOD_USERPTR) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    // During destruction or reconnection, device may be disconnected, STREAMOFF
    // failure is expected
    if (xioctl(VIDIOC_STREAMOFF, &type) == -1) {
      AWARN << "VIDIOC_STREAMOFF failed for " << device_path_ << ": "
            << strerror(errno);
    }
  }
  is_streaming_ = false;
}

int V4L2Device::WaitForData(long sec, long usec) {
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  struct timeval tv;
  tv.tv_sec = sec;
  tv.tv_usec = usec;

  int r = select(fd_ + 1, &fds, nullptr, nullptr, &tv);

  if (r == -1) {
    if (errno == EINTR) {
      return 0;  // Interrupted, not an error, but no data
    }
    AERROR << "select() error on " << device_path_ << ": " << strerror(errno);
    return -1;  // Serious error
  }
  return r;  // Return 0 for timeout, >0 for data available
}

V4L2Buffer V4L2Device::DequeueBuffer() {
  V4L2Buffer result_buf{};

  if (io_method_ == IOMethod::IO_METHOD_READ) {
    ssize_t bytes_read = read(fd_, buffers_[0].start, buffers_[0].length);
    if (bytes_read == -1) {
      if (errno == EAGAIN) {
        // No data in non-blocking mode is normal
        throw_runtime_error("No data available for read (EAGAIN)");
      }
      throw_v4l2_error("read() failed on " + device_path_);
    }
    result_buf.start = buffers_[0].start;
    result_buf.length = bytes_read;
    result_buf.index = 0;
    // READ mode usually does not provide precise timestamp or flags
    return result_buf;
  }

  // For MMAP and USERPTR
  v4l2_buffer buf_info{};
  buf_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf_info.memory = (io_method_ == IOMethod::IO_METHOD_MMAP)
                        ? V4L2_MEMORY_MMAP
                        : V4L2_MEMORY_USERPTR;

  if (xioctl(VIDIOC_DQBUF, &buf_info) == -1) {
    throw_v4l2_error("VIDIOC_DQBUF failed on " + device_path_);
  }

  if (buf_info.index >= buffers_.size()) {
    throw_runtime_error("Invalid buffer index dequeued from driver: " +
                        std::to_string(buf_info.index));
  }

  // Fill return structure
  result_buf.start = buffers_[buf_info.index].start;
  result_buf.length = buf_info.bytesused;
  result_buf.index = buf_info.index;
  result_buf.flags = buf_info.flags;
  result_buf.timestamp = buf_info.timestamp;

  return result_buf;
}

void V4L2Device::QueueBuffer(uint32_t index) {
  if (io_method_ == IOMethod::IO_METHOD_READ) {
    return;  // READ method does not require queue
  }

  if (index >= buffers_.size()) {
    throw_runtime_error("Invalid buffer index to queue: " +
                        std::to_string(index));
  }

  v4l2_buffer buf_info{};
  buf_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf_info.memory = (io_method_ == IOMethod::IO_METHOD_MMAP)
                        ? V4L2_MEMORY_MMAP
                        : V4L2_MEMORY_USERPTR;
  buf_info.index = index;

  if (io_method_ == IOMethod::IO_METHOD_USERPTR) {
    buf_info.m.userptr = reinterpret_cast<uintptr_t>(buffers_[index].start);
    buf_info.length = buffers_[index].length;
  }

  if (xioctl(VIDIOC_QBUF, &buf_info) == -1) {
    throw_v4l2_error("VIDIOC_QBUF failed for " + device_path_);
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
