#include "modules/drivers/camera/backend/camera_device.h"

#include <chrono>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>

#include <linux/videodev2.h>

extern "C" {
#include <libavutil/pixfmt.h>
}

#include "cyber/common/log.h"

namespace apollo {
namespace drivers {
namespace camera {

namespace {

const std::map<std::string, uint32_t> PIXEL_FORMAT_MAP = {
    {"yuyv", V4L2_PIX_FMT_YUYV},
    {"uyvy", V4L2_PIX_FMT_UYVY},
    {"mjpeg", V4L2_PIX_FMT_MJPEG},
    {"rgb24", V4L2_PIX_FMT_RGB24},
    // Adding yuvmono10. Assuming it maps to V4L2_PIX_FMT_Y10.
    // If exact mapping is uncertain for your hardware, confirm or provide an
    // alternative.
    {"yuvmono10", V4L2_PIX_FMT_Y10},
    // Add other formats as needed, e.g., V4L2_PIX_FMT_GREY for 8-bit mono
    {"grey", V4L2_PIX_FMT_GREY}};

/**
 * @brief Safely retrieves the V4L2 pixel format from its string representation.
 * @param format The string representation of the pixel format (e.g., "yuyv",
 * "mjpeg").
 * @return An optional containing the V4L2 pixel format (uint32_t) if found,
 * std::nullopt otherwise.
 */
std::optional<uint32_t> pixel_format_from_string(const std::string& format) {
  auto it = PIXEL_FORMAT_MAP.find(format);
  if (it != PIXEL_FORMAT_MAP.end()) {
    return it->second;
  }
  return std::nullopt;
}

/**
 * @brief Gets the current high-precision monotonic timestamp in nanoseconds.
 * @return Monotonic timestamp in nanoseconds.
 */
uint64_t get_monotonic_now_ns() {
  struct timespec ts;
  // CLOCK_MONOTONIC is unaffected by system time adjustments.
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(ts.tv_nsec);
}

}  // namespace

using apollo::drivers::camera::config::OutputType;

CameraDevice::CameraDevice(const ConfPtr& config)
    : config_(config),
      state_(State::UNINITIALIZED),  // Initial state
      last_timestamp_ns_(0),
      frame_drop_interval_ns_(0),
      frame_warning_interval_ns_(0),
      reconnect_attempts_(0),
      select_timeout_count_(0) {
  AINFO << "Attempting to initialize CameraDevice for "
        << config_->camera_dev();
  if (!Init()) {
    AWARN << "Initial camera initialization failed for "
          << config_->camera_dev() << ". Entering RECONNECTING state.";
    // Even if the initial attempt fails, enter reconnecting state to allow
    // external poll() to trigger retries.
    state_ = State::RECONNECTING;
  }
}

CameraDevice::~CameraDevice() {
  // unique_ptr automatically manages device_ and processor_ destruction.
  // Their destructors are responsible for closing file descriptors,
  // stopping streams, and releasing memory.
  AINFO << "CameraDevice for " << config_->camera_dev() << " is shutting down.";
}

bool CameraDevice::Init() {
  try {
    // 1. Validate and get V4L2 pixel format
    auto pixel_format_opt = pixel_format_from_string(config_->pixel_format());
    if (!pixel_format_opt) {
      throw std::runtime_error("Unsupported pixel format in configuration: " +
                               config_->pixel_format() + " for camera " +
                               config_->camera_dev());
    }
    uint32_t pixel_format_v4l2 = *pixel_format_opt;

    // 2. Create and configure V4L2Device
    // Resetting unique_ptr if it already holds an object (e.g., during
    // reconnect)
    device_.reset();
    device_ = std::make_unique<V4L2Device>(config_);

    // Configure V4L2Device with desired parameters
    AINFO << "Configuring V4L2Device: " << config_->camera_dev()
          << " Width: " << config_->width() << " Height: " << config_->height()
          << " Pixel Format: " << config_->pixel_format()
          << " Frame Rate: " << config_->frame_rate();

    device_->Configure(config_->width(), config_->height(), pixel_format_v4l2,
                       config_->frame_rate());

    // 3. Get actual configuration from device and update config (if driver
    // adjusted parameters) This is crucial as V4L2 might adjust requested
    // parameters. Update the config proto with actual values obtained from the
    // device.
    config_->set_width(device_->GetWidth());
    config_->set_height(device_->GetHeight());
    // Note: V4L2Device should ideally provide actual pixel format and frame
    // rate as well. For this example, we assume get_width/get_height are
    // sufficient for processor creation.

    // 4. Create ImageProcessor based on actual device capabilities and desired
    // output
    processor_.reset();
    if (pixel_format_v4l2 == V4L2_PIX_FMT_MJPEG) {
      processor_ =
          std::make_unique<MjpegProcessor>(config_->width(), config_->height());
    } else {
      // Determine output format for YuvProcessor (RGB or YUYV passthrough)
      YuvProcessor::OutputFormat output_format;
      if (config_->output_type() == OutputType::RGB) {
        output_format = YuvProcessor::OutputFormat::RGB;
        AINFO << "YuvProcessor will output RGB.";
      } else {  // Default or Config::YUYV
        output_format = YuvProcessor::OutputFormat::YUYV;
        AINFO << "YuvProcessor will output YUYV passthrough.";
      }

      // Determine if input YUV is UYVY for conversion
      bool is_uyvy = (pixel_format_v4l2 == V4L2_PIX_FMT_UYVY);
      processor_ = std::make_unique<YuvProcessor>(output_format, is_uyvy);
    }

    // 5. Set all camera parameters (brightness, exposure, etc.)
    SetCameraParameters();

    // 6. Start video streaming
    device_->StartStreaming();

    // 7. Calculate frame interval for frame rate control and timestamp warnings
    double frame_rate = config_->frame_rate() > 0
                            ? config_->frame_rate()
                            : 30.0;  // Default to 30 FPS if not set
    // Calculate minimum interval to avoid dropping frames (e.g., allow 10%
    // jitter)
    frame_drop_interval_ns_ = static_cast<uint64_t>((0.9 / frame_rate) * 1e9);
    // Warn if actual interval exceeds 1.5 times the nominal frame interval
    frame_warning_interval_ns_ =
        static_cast<uint64_t>((1.5 / frame_rate) * 1e9);

    AINFO << "CameraDevice for " << config_->camera_dev()
          << " successfully initialized with actual resolution "
          << config_->width() << "x" << config_->height() << ".";
    state_ = State::INITIALIZED;
    reconnect_attempts_ = 0;
    select_timeout_count_ = 0;  // Reset timeout counter
    last_timestamp_ns_ = 0;     // Reset last timestamp for fresh start
    return true;
  } catch (const std::exception& e) {
    AERROR << "Failed to initialize CameraDevice for " << config_->camera_dev()
           << ": " << e.what();
    // Ensure all resources are cleaned up if initialization fails at any point
    processor_.reset();
    device_.reset();
    state_ = State::UNINITIALIZED;  // Explicitly mark as uninitialized
    return false;
  }
}

void CameraDevice::SetCameraParameters() {
  AINFO << "Setting camera parameters for " << config_->camera_dev();
  if (config_->has_brightness())
    device_->SetParameter(V4L2_CID_BRIGHTNESS, config_->brightness());
  if (config_->has_contrast())
    device_->SetParameter(V4L2_CID_CONTRAST, config_->contrast());
  if (config_->has_saturation())
    device_->SetParameter(V4L2_CID_SATURATION, config_->saturation());
  if (config_->has_sharpness())
    device_->SetParameter(V4L2_CID_SHARPNESS, config_->sharpness());
  if (config_->has_gain())
    device_->SetParameter(V4L2_CID_GAIN, config_->gain());

  // White Balance
  if (config_->auto_white_balance()) {
    // Enable auto white balance
    device_->SetParameter(V4L2_CID_AUTO_WHITE_BALANCE, 1);
    AINFO << "Auto White Balance enabled.";
  } else {
    // Disable auto white balance and set manual value if provided
    device_->SetParameter(V4L2_CID_AUTO_WHITE_BALANCE, 0);
    if (config_->has_white_balance()) {
      device_->SetParameter(V4L2_CID_WHITE_BALANCE_TEMPERATURE,
                            config_->white_balance());
      AINFO << "Manual White Balance set to " << config_->white_balance();
    }
  }

  // Exposure
  if (config_->auto_exposure()) {
    // Enable auto exposure (aperture priority mode if supported)
    device_->SetParameter(V4L2_CID_EXPOSURE_AUTO,
                          V4L2_EXPOSURE_APERTURE_PRIORITY);
    AINFO << "Auto Exposure enabled.";
  } else {
    // Disable auto exposure and set manual value if provided
    device_->SetParameter(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    if (config_->has_exposure()) {
      device_->SetParameter(V4L2_CID_EXPOSURE_ABSOLUTE, config_->exposure());
      AINFO << "Manual Exposure set to " << config_->exposure();
    }
  }

  // Focus
  if (config_->auto_focus()) {
    // Enable auto focus
    device_->SetParameter(V4L2_CID_FOCUS_AUTO, 1);
    AINFO << "Auto Focus enabled.";
  } else {
    // Disable auto focus and set manual value if provided
    device_->SetParameter(V4L2_CID_FOCUS_AUTO, 0);
    if (config_->has_focus()) {
      device_->SetParameter(V4L2_CID_FOCUS_ABSOLUTE, config_->focus());
      AINFO << "Manual Focus set to " << config_->focus();
    }
  }
}

bool CameraDevice::Poll(std::shared_ptr<Image> pb_image) {
  // If in reconnecting state, attempt to reconnect first
  if (state_ == State::RECONNECTING) {
    Reconnect();
    // Always return false if we are reconnecting, as no valid image is produced
    // in this cycle.
    return false;
  }

  // If not initialized, cannot poll. This state should theoretically be caught
  // by RECONNECTING.
  if (state_ != State::INITIALIZED) {
    AERROR << "Camera " << config_->camera_dev()
           << " is not initialized (state: " << static_cast<int>(state_)
           << "), cannot poll.";
    return false;
  }

  try {
    // 1. Wait for data using select()
    // Using a 2-second timeout to allow the system to respond to other tasks.
    int ready = device_->WaitForData(2, 0);  // (seconds, microseconds)

    if (ready < 0) {  // An error occurred during select()
      AERROR << "select() on " << config_->camera_dev()
             << " failed with error: " << strerror(errno)
             << ". Triggering reconnect.";
      state_ = State::RECONNECTING;  // Transition to reconnecting state
      return false;
    }
    if (ready == 0) {  // Timeout: no data ready within the specified time
      select_timeout_count_++;
      AWARN << "select() timeout on " << config_->camera_dev()
            << ". Consecutive timeout count: " << select_timeout_count_;
      // If consecutive timeouts exceed a threshold, consider the device
      // unresponsive. Threshold can be tuned based on application needs
      if (select_timeout_count_ >= 3) {
        AERROR << "Consecutive timeouts exceeded (" << select_timeout_count_
               << "), triggering reconnect for " << config_->camera_dev()
               << ".";
        state_ = State::RECONNECTING;
      }
      return false;  // No image available this poll cycle
    }
    // Data is ready, reset timeout count as operation proceeds
    select_timeout_count_ = 0;

    // 2. Dequeue a buffer from the V4L2 device
    V4L2Buffer v4l2_buf = device_->DequeueBuffer();

    // 3. Get and validate timestamp
    uint64_t current_ts_ns = GetMonotonicTimestamp(v4l2_buf);

    // 4. Frame rate control and timestamp check
    if (last_timestamp_ns_ != 0) {  // Skip for the very first frame
      uint64_t interval = current_ts_ns - last_timestamp_ns_;

      // Drop frame if interval is too small (e.g., camera delivering frames too
      // fast)
      if (interval < frame_drop_interval_ns_) {
        AINFO << "Dropping frame from " << config_->camera_dev()
              << " due to small interval (" << interval << " ns)."
              << " Expected min: " << frame_drop_interval_ns_ << " ns.";
        device_->QueueBuffer(v4l2_buf.index);
        return false;  // Frame dropped
      }

      // Warn if interval is too large (potential frame loss or timestamp jump)
      if (interval > frame_warning_interval_ns_) {
        AWARN << "Timestamp jump detected on " << config_->camera_dev()
              << ". Last: " << last_timestamp_ns_
              << ", Current: " << current_ts_ns << ", Diff: " << interval
              << " ns. Expected max: " << frame_warning_interval_ns_ << " ns.";
      }
    }
    last_timestamp_ns_ = current_ts_ns;  // Update last timestamp for next cycle

    // 5. Fill protobuf Image message metadata
    pb_image->set_measurement_time(static_cast<double>(current_ts_ns) /
                                   1e9);  // Convert ns to seconds
    pb_image->set_frame_id(config_->frame_id());
    pb_image->set_encoding(config_->output_type() == OutputType::RGB
                               ? "bgr8"
                               : "yuyv");  // Set encoding string
    pb_image->set_width(config_->width());
    pb_image->set_height(config_->height());
    pb_image->set_step(config_->output_type() == OutputType::RGB
                           ? config_->width() * 3
                           : config_->width() * 2);  // Bytes per row
    pb_image->mutable_data()->resize(pb_image->height() * pb_image->step());

    // 6. Process image data (decode/convert) using the image processor
    // Processor will write directly into pb_image->mutable_data()->data()
    processor_->Process(v4l2_buf.start, v4l2_buf.length, pb_image);

    // 7. Enqueue the buffer back to the V4L2 device for reuse
    device_->QueueBuffer(v4l2_buf.index);

    // Image successfully processed and ready
    return true;

  } catch (const std::exception& e) {
    AERROR << "Polling failed with exception on " << config_->camera_dev()
           << ": " << e.what();
    // In case of any exception during poll, assume device is in a bad state and
    // trigger reconnect.
    state_ = State::RECONNECTING;
    // It's crucial to return false here as no valid image was produced.
    return false;
  }
}

bool CameraDevice::IsCapturing() const { return state_ == State::INITIALIZED; }

uint64_t CameraDevice::GetMonotonicTimestamp(const V4L2Buffer& v4l2_buf) {
  // Prefer to use V4L2-provided monotonic timestamp from the buffer.
  // This is the most accurate timestamp for when the frame was actually
  // captured by the hardware.
  if (v4l2_buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
    return static_cast<uint64_t>(v4l2_buf.timestamp.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(v4l2_buf.timestamp.tv_usec) * 1000ULL;
  }

  // If hardware monotonic timestamp is unavailable or unreliable
  // (V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC not set), fall back to getting the
  // system monotonic time immediately after data is ready. This provides the
  // closest possible high-precision monotonic timestamp to the actual capture
  // event, compensating for potential driver/hardware limitations.
  AWARN_EVERY(1000) << "Hardware monotonic timestamp not available for "
                    << config_->camera_dev()
                    << ". Using system monotonic time.";
  return get_monotonic_now_ns();
}

void CameraDevice::Reconnect() {
  AWARN << "Attempting to reconnect camera " << config_->camera_dev()
        << " (Attempt " << reconnect_attempts_ + 1 << ")";

  // Set state to RECONNECTING explicitly for clarity, though it might already
  // be.
  state_ = State::RECONNECTING;

  // Cleanup existing resources before attempting re-initialization.
  // This ensures a clean slate for the new connection attempt.
  processor_.reset();
  device_.reset();
  last_timestamp_ns_ = 0;  // Reset timestamp history for new connection

  // Exponential backoff retry logic: 1s, 2s, 4s, ..., capped at 30s.
  // This prevents hammering the system/device on rapid consecutive failures.
  uint64_t wait_ms =
      std::min(1000ULL * (1ULL << reconnect_attempts_), 30000ULL);
  AINFO << "Waiting for " << wait_ms
        << " ms before next reconnection attempt for " << config_->camera_dev();
  std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));

  // Attempt to re-initialize the camera.
  if (Init()) {
    AINFO << "Reconnection to " << config_->camera_dev() << " successful after "
          << reconnect_attempts_ + 1 << " attempts.";
    // The 'state_' is set to INITIALIZED within init() upon success.
  } else {
    AERROR << "Reconnection to " << config_->camera_dev() << " failed (attempt "
           << reconnect_attempts_ + 1 << ").";
    reconnect_attempts_++;  // Increment retry count for the next attempt.
    // 'state_' remains RECONNECTING or becomes UNINITIALIZED if init() fails.
    // The poll() loop will continue to trigger reconnect() until successful.
  }
}

}  // namespace camera
}  // namespace drivers
}  // namespace apollo
