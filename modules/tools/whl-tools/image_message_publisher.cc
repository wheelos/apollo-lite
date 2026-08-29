// Copyright 2026 WheelOS. All Rights Reserved.
//
// Publish decoded images as apollo.drivers.Image messages for offline debugging.

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "wheelos_msgs/sensor_msgs/sensor_image.pb.h"

namespace {

struct Options {
  std::filesystem::path input_path;
  std::string channel = "/apollo/sensor/camera/front_6mm/image";
  std::string frame_id = "camera_front_6mm";
  double fps = 10.0;
  uint64_t start_timestamp_ns = 0;
  uint32_t max_frames = 0;
};

void PrintUsage(const char* program) {
  std::cout
      << "Usage: " << program << " --input <directory|video> [options]\n"
      << "Publish images as packed rgb8 apollo.drivers.Image messages.\n\n"
      << "Options:\n"
      << "  --channel <topic>             Output channel.\n"
      << "  --frame-id <id>               Image and header frame ID.\n"
      << "  --fps <positive number>       Publish rate; video defaults to source FPS"
         " when omitted.\n"
      << "  --max-frames <count>          Stop after this many frames; 0 means all.\n"
      << "  --start-timestamp-ns <uint64> First frame sensor timestamp; defaults to"
         " current Cyber time.\n"
      << "  -h, --help                    Show this help.\n";
}

bool ParseUint64(const std::string& text, uint64_t* value) {
  if (value == nullptr || text.empty()) {
    return false;
  }
  try {
    size_t consumed = 0;
    const auto parsed = std::stoull(text, &consumed);
    if (consumed != text.size()) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseUint32(const std::string& text, uint32_t* value) {
  uint64_t parsed = 0;
  if (!ParseUint64(text, &parsed) ||
      parsed > std::numeric_limits<uint32_t>::max()) {
    return false;
  }
  *value = static_cast<uint32_t>(parsed);
  return true;
}

bool ParseDouble(const std::string& text, double* value) {
  if (value == nullptr || text.empty()) {
    return false;
  }
  try {
    size_t consumed = 0;
    const double parsed = std::stod(text, &consumed);
    if (consumed != text.size() || !std::isfinite(parsed) || parsed <= 0.0) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

bool ParseOptions(int argc, char* argv[], Options* options) {
  if (options == nullptr) {
    return false;
  }
  bool fps_specified = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "-h" || argument == "--help") {
      PrintUsage(argv[0]);
      return false;
    }
    if (index + 1 == argc) {
      std::cerr << "Missing value for " << argument << "\n";
      return false;
    }
    const std::string value = argv[++index];
    if (argument == "--input") {
      options->input_path = value;
    } else if (argument == "--channel") {
      options->channel = value;
    } else if (argument == "--frame-id") {
      options->frame_id = value;
    } else if (argument == "--fps") {
      fps_specified = ParseDouble(value, &options->fps);
      if (!fps_specified) {
        std::cerr << "Invalid --fps value: " << value << "\n";
        return false;
      }
    } else if (argument == "--max-frames") {
      if (!ParseUint32(value, &options->max_frames)) {
        std::cerr << "Invalid --max-frames value: " << value << "\n";
        return false;
      }
    } else if (argument == "--start-timestamp-ns") {
      if (!ParseUint64(value, &options->start_timestamp_ns)) {
        std::cerr << "Invalid --start-timestamp-ns value: " << value << "\n";
        return false;
      }
    } else {
      std::cerr << "Unknown option: " << argument << "\n";
      return false;
    }
  }
  if (options->input_path.empty() || options->channel.empty() ||
      options->frame_id.empty()) {
    std::cerr << "--input, --channel, and --frame-id must not be empty\n";
    return false;
  }
  if (!std::filesystem::exists(options->input_path)) {
    std::cerr << "Input does not exist: " << options->input_path << "\n";
    return false;
  }
  if (std::filesystem::is_regular_file(options->input_path) && !fps_specified) {
    cv::VideoCapture video(options->input_path.string());
    const double source_fps = video.get(cv::CAP_PROP_FPS);
    if (source_fps > 0.0 && std::isfinite(source_fps)) {
      options->fps = source_fps;
    }
  }
  return true;
}

bool IsImageFile(const std::filesystem::path& path) {
  std::string extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
                 [](unsigned char character) { return std::tolower(character); });
  return extension == ".bmp" || extension == ".jpeg" || extension == ".jpg" ||
         extension == ".png" || extension == ".ppm" || extension == ".tif" ||
         extension == ".tiff";
}

std::vector<std::filesystem::path> ImagePaths(
    const std::filesystem::path& directory) {
  std::vector<std::filesystem::path> paths;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && IsImageFile(entry.path())) {
      paths.push_back(entry.path());
    }
  }
  std::sort(paths.begin(), paths.end());
  return paths;
}

bool PublishFrame(const cv::Mat& input, uint64_t timestamp_ns, uint32_t sequence,
                  const Options& options,
                  const std::shared_ptr<apollo::cyber::Writer<
                      apollo::drivers::Image>>& writer) {
  if (input.empty() || writer == nullptr) {
    return false;
  }
  cv::Mat rgb;
  if (input.channels() == 1) {
    cv::cvtColor(input, rgb, cv::COLOR_GRAY2RGB);
  } else if (input.channels() == 3) {
    cv::cvtColor(input, rgb, cv::COLOR_BGR2RGB);
  } else if (input.channels() == 4) {
    cv::cvtColor(input, rgb, cv::COLOR_BGRA2RGB);
  } else {
    std::cerr << "Unsupported image channel count: " << input.channels() << "\n";
    return false;
  }
  if (!rgb.isContinuous()) {
    rgb = rgb.clone();
  }
  const uint64_t byte_count =
      static_cast<uint64_t>(rgb.rows) * static_cast<uint64_t>(rgb.cols) * 3U;
  if (byte_count > std::numeric_limits<int>::max()) {
    std::cerr << "Image is too large to serialize\n";
    return false;
  }
  auto image = std::make_shared<apollo::drivers::Image>();
  const double timestamp_sec = static_cast<double>(timestamp_ns) * 1.0e-9;
  auto* header = image->mutable_header();
  header->set_timestamp_sec(timestamp_sec);
  header->set_camera_timestamp(timestamp_ns);
  header->set_module_name("whl_image_message_publisher");
  header->set_sequence_num(sequence);
  header->set_frame_id(options.frame_id);
  image->set_frame_id(options.frame_id);
  image->set_measurement_time(timestamp_sec);
  image->set_height(static_cast<uint32_t>(rgb.rows));
  image->set_width(static_cast<uint32_t>(rgb.cols));
  image->set_encoding("rgb8");
  image->set_step(static_cast<uint32_t>(rgb.cols * 3));
  image->set_data(reinterpret_cast<const char*>(rgb.data),
                  static_cast<int>(byte_count));
  writer->Write(image);
  return true;
}

template <typename NextFrame>
int PublishFrames(const Options& options, NextFrame next_frame) {
  const uint64_t interval_ns =
      static_cast<uint64_t>(std::llround(1.0e9 / options.fps));
  const uint64_t start_timestamp_ns =
      options.start_timestamp_ns == 0
          ? static_cast<uint64_t>(apollo::cyber::Time::Now().ToNanosecond())
          : options.start_timestamp_ns;
  auto node = apollo::cyber::CreateNode("whl_image_message_publisher");
  auto writer = node->CreateWriter<apollo::drivers::Image>(options.channel);
  if (writer == nullptr) {
    std::cerr << "Unable to create writer for " << options.channel << "\n";
    return 1;
  }

  uint32_t sequence = 1;
  cv::Mat frame;
  while ((options.max_frames == 0U || sequence <= options.max_frames) &&
         next_frame(&frame)) {
    if (!PublishFrame(frame, start_timestamp_ns +
                                 static_cast<uint64_t>(sequence - 1) * interval_ns,
                      sequence, options, writer)) {
      return 1;
    }
    std::cout << "Published frame " << sequence << " at "
              << start_timestamp_ns +
                     static_cast<uint64_t>(sequence - 1) * interval_ns
              << " ns\n";
    ++sequence;
    std::this_thread::sleep_for(
        std::chrono::nanoseconds(static_cast<int64_t>(interval_ns)));
  }
  if (sequence == 1) {
    std::cerr << "No decodable frames found in " << options.input_path << "\n";
    return 1;
  }
  return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
  Options options;
  if (!ParseOptions(argc, argv, &options)) {
    return argc > 1 && (std::string(argv[1]) == "--help" ||
                        std::string(argv[1]) == "-h")
               ? 0
               : 2;
  }
  apollo::cyber::Init("whl_image_message_publisher");
  int result = 0;
  if (std::filesystem::is_directory(options.input_path)) {
    const auto paths = ImagePaths(options.input_path);
    size_t index = 0;
    result = PublishFrames(options, [&paths, &index](cv::Mat* frame) {
      if (index == paths.size()) {
        return false;
      }
      *frame = cv::imread(paths[index++].string(), cv::IMREAD_UNCHANGED);
      return true;
    });
  } else {
    cv::VideoCapture video(options.input_path.string());
    if (!video.isOpened()) {
      std::cerr << "Unable to open video: " << options.input_path << "\n";
      result = 1;
    } else {
      result = PublishFrames(
          options, [&video](cv::Mat* frame) { return video.read(*frame); });
    }
  }
  apollo::cyber::Clear();
  return result;
}
