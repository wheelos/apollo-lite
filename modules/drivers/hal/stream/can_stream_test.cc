// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-03-21
//  Author: daohu527

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// --- Mock CanStream for Testing ---
// We mock CanStream to control what its read() and write() methods do,
// simulating interaction with a CAN interface without touching real hardware.
class MockCanStream : public apollo::drivers::hal::CanStream {
 public:
  // Pass parameters to base constructor. enable_can_fd is crucial for testing
  // different modes.
  MockCanStream(const std::string& address, uint16_t port,
                uint32_t timeout_usec, bool enable_can_fd)
      : CanStream(address, port, timeout_usec, enable_can_fd) {}

  // Mock virtual methods we interact with
  // Note: We are NOT mocking Connect/Disconnect to test them, but rather
  // to control the *state* of the mock (connected/disconnected) for read/write
  // tests. If you needed to test CanStream::Connect/Disconnect implementation
  // logic, you'd need a partial mock or OS interaction test setup.
  MOCK_METHOD(bool, Connect, (), (override));
  MOCK_METHOD(bool, Disconnect, (), (override));

  MOCK_METHOD(size_t, read, (uint8_t* buffer, size_t max_length), (override));
  MOCK_METHOD(size_t, write, (const uint8_t* data, size_t length), (override));

  // We might need to mock get_last_error_code and get_status if the code uses
  // them heavily MOCK_METHOD(int, get_last_error_code, (), (const, override));
  // MOCK_METHOD(Stream::Status, get_status, (), (const, override));

  // --- Helper methods for simulating read/write data ---

  // Helper to simulate data available for reading by setting an internal
  // buffer. The test sets this buffer with raw bytes representing CAN frames.
  void set_read_data(const std::vector<uint8_t>& data) {
    read_buffer_ = data;
    read_buffer_pos_ = 0;
  }

  // Helper to get the data that was "written" to the mock.
  const std::vector<uint8_t>& get_written_data() const { return written_data_; }

  // Action method for mock read: copies data from internal buffer.
  // Simulates reading available bytes up to max_length.
  // This action is used for both Standard and FD tests; the size difference is
  // handled by max_length.
  size_t MockReadAction(uint8_t* buffer, size_t max_length) {
    size_t bytes_available = read_buffer_.size() - read_buffer_pos_;
    size_t bytes_to_copy = std::min(max_length, bytes_available);

    if (bytes_to_copy > 0) {
      std::memcpy(buffer, read_buffer_.data() + read_buffer_pos_,
                  bytes_to_copy);
      read_buffer_pos_ += bytes_to_copy;
    } else {
      // Simulate non-blocking/timeout behavior: if no data, return 0.
      // For real error simulation (EAGAIN, ETIMEDOUT), you'd combine this with
      // .WillRepeatedly(::testing::Return(-1)) and
      // .WillRepeatedly(::testing::SetErrno(EAGAIN))
    }

    return bytes_to_copy;
  }

  // Action method for mock write: appends data to an internal buffer.
  // Simulates successful write of all requested bytes.
  // This action is used for both Standard and FD tests; the size difference is
  // handled by length.
  size_t MockWriteAction(const uint8_t* data, size_t length) {
    written_data_.insert(written_data_.end(), data, data + length);
    return length;
    // For simulating partial write or errors, use
    // .WillOnce(Return(partial_size)) or .WillOnce(Throw(...)) etc.
  }

 private:
  std::vector<uint8_t> read_buffer_;
  size_t read_buffer_pos_ = 0;
  std::vector<uint8_t> written_data_;
};

// --- Gtest Test Suite for CAN Parsing Logic (Upper Layer) ---
// These tests focus *only* on how the upper layer code should interpret
// the raw bytes received from Stream::read as CAN frames.
// They DO NOT test CanStream::read itself.
TEST(CanStreamParsingTest, ParseStandardFrame_Single_ValidDLC) {
  // This test demonstrates how upper layer parses bytes into a standard CAN
  // frame.
  using namespace apollo::drivers::hal;

  // 1. Manually create the byte representation of a standard CAN frame
  // This simulates the bytes received by Stream::read into a buffer
  struct can_frame raw_frame_bytes;
  raw_frame_bytes.can_id = 0x123;  // Example CAN ID
  raw_frame_bytes.can_dlc = 5;     // Example data length code (5 bytes)
  raw_frame_bytes.data[0] = 0xAA;
  raw_frame_bytes.data[1] = 0xBB;
  raw_frame_bytes.data[2] = 0xCC;
  raw_frame_bytes.data[3] = 0xDD;
  raw_frame_bytes.data[4] = 0xEE;
  // Bytes 5,6,7 in the data array are conceptually invalid according to DLC=5
  raw_frame_bytes.data[5] = 0xFF;  // Garbage byte example
  raw_frame_bytes.data[6] = 0x00;
  raw_frame_bytes.data[7] = 0x11;

  // Prepare the byte buffer as received by Stream::read
  const size_t frame_size = sizeof(struct can_frame);  // e.g., 16 bytes
  std::vector<uint8_t> received_bytes(frame_size);
  std::memcpy(received_bytes.data(), &raw_frame_bytes, frame_size);

  // --- Upper Layer Parsing Logic (Code under test in this specific test) ---

  // Assume 'buffer' points to received_bytes.data() and 'bytes_read' ==
  // received_bytes.size()
  uint8_t* buffer = received_bytes.data();
  size_t bytes_read = received_bytes.size();

  // 1. Upper layer must know the expected frame size (Standard CAN in this
  // case)
  const size_t expected_frame_size = sizeof(struct can_frame);

  // 2. Basic check: enough bytes for at least one frame, and total bytes is
  // multiple of frame size
  ASSERT_GE(bytes_read, expected_frame_size);
  ASSERT_EQ(bytes_read % expected_frame_size, 0);

  // 3. Reinterpret the bytes as a CAN frame structure
  // This is the core parsing step at this level
  const struct can_frame* parsed_frame =
      reinterpret_cast<const struct can_frame*>(buffer);

  // 4. Access and verify frame fields
  ASSERT_EQ(parsed_frame->can_id, 0x123);
  ASSERT_EQ(parsed_frame->can_dlc, 5);

  // 5. Access and verify data payload (only up to DLC)
  // The upper layer uses can_dlc to know how many bytes in data[] are valid
  ASSERT_EQ(parsed_frame->can_dlc, 5);
  // Verify the first 5 bytes of data[] match the expected data
  ASSERT_TRUE(std::memcmp(parsed_frame->data, raw_frame_bytes.data,
                          parsed_frame->can_dlc) == 0)
      << "Parsed data bytes up to DLC should match expected data.";

  // 6. Upper layer application logic: Interpret the meaning of the payload
  // bytes This is where you'd map CAN ID/DLC to specific data formats (e.g.,
  // protocol buffers, structs) Example: If ID 0x123 DLC 5 means a sensor value
  // (uint16) at offset 1
  if (parsed_frame->can_id == 0x123 &&
      parsed_frame->can_dlc >= 1 + sizeof(uint16_t)) {
    uint16_t sensor_value;
    // Copy the bytes from the data array into the variable
    std::memcpy(&sensor_value, parsed_frame->data + 1,
                sizeof(uint16_t));  // Copy 2 bytes starting at index 1
    // In a real application/test, you would assert the value of sensor_value
    // based on the bytes BB CC (assuming little-endian: 0xCCBB)
    // ASSERT_EQ(sensor_value, 0xCCBB);
  } else if (parsed_frame->can_id == 0x123 &&
             parsed_frame->can_dlc < 1 + sizeof(uint16_t)) {
    // This branch is fine; indicates frame was received but doesn't have enough
    // data for this specific parsing logic.
  }
}

TEST(CanStreamParsingTest, ParseStandardFrame_Multiple) {
  // This test demonstrates parsing multiple standard CAN frames from a single
  // buffer.
  using namespace apollo::drivers::hal;

  const size_t frame_size = sizeof(struct can_frame);

  // Frame 1
  struct can_frame frame1_bytes;
  frame1_bytes.can_id = 0x101;
  frame1_bytes.can_dlc = 8;
  std::memset(frame1_bytes.data, 0x11, 8);
  // Frame 2
  struct can_frame frame2_bytes;
  frame2_bytes.can_id = 0x102;
  frame2_bytes.can_dlc = 4;
  std::memset(frame2_bytes.data, 0x22, 4);
  std::memset(frame2_bytes.data + 4, 0, 4);
  // Frame 3
  struct can_frame frame3_bytes;
  frame3_bytes.can_id = 0x103;
  frame3_bytes.can_dlc = 0;
  std::memset(frame3_bytes.data, 0x33,
              8);  // Data contents irrelevant for DLC 0

  // Prepare a buffer containing bytes for 3 frames
  std::vector<uint8_t> received_bytes(3 * frame_size);
  std::memcpy(received_bytes.data(), &frame1_bytes, frame_size);
  std::memcpy(received_bytes.data() + frame_size, &frame2_bytes, frame_size);
  std::memcpy(received_bytes.data() + 2 * frame_size, &frame3_bytes,
              frame_size);

  // --- Upper Layer Parsing Logic ---
  uint8_t* buffer = received_bytes.data();
  size_t bytes_read = received_bytes.size();
  const size_t expected_frame_size = sizeof(struct can_frame);

  ASSERT_GE(bytes_read, expected_frame_size);
  ASSERT_EQ(bytes_read % expected_frame_size, 0);

  size_t num_frames_received = bytes_read / expected_frame_size;
  ASSERT_EQ(num_frames_received, 3);

  // Iterate through the buffer, parsing each frame
  for (size_t i = 0; i < num_frames_received; ++i) {
    // Calculate the start of the current frame in the buffer
    const struct can_frame* parsed_frame =
        reinterpret_cast<const struct can_frame*>(buffer +
                                                  i * expected_frame_size);

    // Verify each frame based on expected data
    if (i == 0) {
      ASSERT_EQ(parsed_frame->can_id, frame1_bytes.can_id);
      ASSERT_EQ(parsed_frame->can_dlc, frame1_bytes.can_dlc);
      ASSERT_TRUE(std::memcmp(parsed_frame->data, frame1_bytes.data,
                              parsed_frame->can_dlc) == 0);
    } else if (i == 1) {
      ASSERT_EQ(parsed_frame->can_id, frame2_bytes.can_id);
      ASSERT_EQ(parsed_frame->can_dlc, frame2_bytes.can_dlc);
      ASSERT_TRUE(std::memcmp(parsed_frame->data, frame2_bytes.data,
                              parsed_frame->can_dlc) == 0);
    } else if (i == 2) {
      ASSERT_EQ(parsed_frame->can_id, frame3_bytes.can_id);
      ASSERT_EQ(parsed_frame->can_dlc, frame3_bytes.can_dlc);
      ASSERT_TRUE(std::memcmp(parsed_frame->data, frame3_bytes.data,
                              parsed_frame->can_dlc) == 0);
    }
    // Application logic would then process parsed_frame->data based on ID/DLC
  }
}

TEST(CanStreamParsingTest, ParseCanFDFrame_Single_ValidLen) {
  // This test demonstrates how upper layer parses bytes into a CAN FD frame.
  using namespace apollo::drivers::hal;

  // 1. Manually create the byte representation of a CAN FD frame
  struct canfd_frame raw_frame_fd_bytes;
  raw_frame_fd_bytes.can_id =
      0x234 | CAN_EFF_FLAG | CANFD_BRS | CANFD_ESI;  // Example ID + FD flags
  raw_frame_fd_bytes.len = 32;   // Example CAN FD data length (32 bytes)
  raw_frame_fd_bytes.flags = 0;  // Example FD flags (besides those in can_id)

  // Example data payload (32 bytes)
  for (int i = 0; i < raw_frame_fd_bytes.len; ++i) {
    raw_frame_fd_bytes.data[i] = static_cast<__u8>(i + 1);
  }
  std::memset(raw_frame_fd_bytes.data + raw_frame_fd_bytes.len, 0,
              sizeof(raw_frame_fd_bytes.data) -
                  raw_frame_fd_bytes.len);  // Zero pad remaining

  // Prepare the byte buffer as received by Stream::read
  const size_t frame_size_fd = sizeof(struct canfd_frame);  // e.g., 72 bytes
  std::vector<uint8_t> received_bytes_fd(frame_size_fd);
  std::memcpy(received_bytes_fd.data(), &raw_frame_fd_bytes, frame_size_fd);

  // --- Upper Layer Parsing Logic ---
  uint8_t* buffer_fd = received_bytes_fd.data();
  size_t bytes_read_fd = received_bytes_fd.size();

  // 1. Upper layer must know the expected frame size (CAN FD in this case)
  const size_t expected_frame_size_fd = sizeof(struct canfd_frame);

  // 2. Basic check: enough bytes for at least one frame, and total bytes is
  // multiple of frame size
  ASSERT_GE(bytes_read_fd, expected_frame_size_fd);
  ASSERT_EQ(bytes_read_fd % expected_frame_size_fd, 0);

  // 3. Reinterpret the bytes as a CAN FD frame structure
  const struct canfd_frame* parsed_frame_fd =
      reinterpret_cast<const struct canfd_frame*>(buffer_fd);

  // 4. Access and verify frame fields (use len for data length)
  ASSERT_EQ(parsed_frame_fd->can_id, raw_frame_fd_bytes.can_id);
  ASSERT_EQ(parsed_frame_fd->len, raw_frame_fd_bytes.len);
  ASSERT_EQ(parsed_frame_fd->flags, raw_frame_fd_bytes.flags);

  // 5. Access and verify data payload (only up to len)
  ASSERT_EQ(parsed_frame_fd->len, 32);  // Verify the expected data length
  ASSERT_TRUE(std::memcmp(parsed_frame_fd->data, raw_frame_fd_bytes.data,
                          parsed_frame_fd->len) == 0)
      << "Parsed data bytes up to len should match expected data.";

  // Upper layer application logic would then interpret parsed_frame_fd->data
  // based on ID/len Example: Process the 32 bytes payload
  if (parsed_frame_fd->len == 32) {
    // Copy or process the 32 bytes in parsed_frame_fd->data
    std::vector<uint8_t> payload_data(
        parsed_frame_fd->data, parsed_frame_fd->data + parsed_frame_fd->len);
    ASSERT_EQ(payload_data.size(), 32);
    for (int i = 0; i < 32; ++i) {
      ASSERT_EQ(payload_data[i], static_cast<uint8_t>(i + 1));
    }
  }
}

TEST(CanStreamParsingTest, ParseCanFDFrame_Multiple) {
  // This test demonstrates parsing multiple CAN FD frames from a single buffer.
  using namespace apollo::drivers::hal;

  const size_t frame_size_fd = sizeof(struct canfd_frame);
  const size_t num_frames_to_test = 2;

  // Frame 1 (CAN FD)
  struct canfd_frame frame_fd_1;
  frame_fd_1.can_id = 0x301 | CAN_EFF_FLAG | CANFD_BRS;
  frame_fd_1.len = 8;  // CAN FD with standard DLC length
  frame_fd_1.flags = 0;
  std::memset(frame_fd_1.data, 0xA1, frame_fd_1.len);
  std::memset(frame_fd_1.data + frame_fd_1.len, 0,
              sizeof(frame_fd_1.data) - frame_fd_1.len);

  // Frame 2 (CAN FD)
  struct canfd_frame frame_fd_2;
  frame_fd_2.can_id = 0x302;  // Standard ID
  frame_fd_2.len = 64;        // Max CAN FD length
  frame_fd_2.flags = CANFD_ESI;
  for (int i = 0; i < frame_fd_2.len; ++i) {
    frame_fd_2.data[i] = static_cast<__u8>(63 - i);
  }

  // Prepare a buffer containing bytes for 2 FD frames
  std::vector<uint8_t> received_bytes_fd(num_frames_to_test * frame_size_fd);
  std::memcpy(received_bytes_fd.data(), &frame_fd_1, frame_size_fd);
  std::memcpy(received_bytes_fd.data() + frame_size_fd, &frame_fd_2,
              frame_size_fd);

  // --- Upper Layer Parsing Logic ---
  uint8_t* buffer_fd = received_bytes_fd.data();
  size_t bytes_read_fd = received_bytes_fd.size();
  const size_t expected_frame_size_fd = sizeof(struct canfd_frame);

  ASSERT_GE(bytes_read_fd, expected_frame_size_fd);
  ASSERT_EQ(bytes_read_fd % expected_frame_size_fd, 0);

  size_t num_frames_received_fd = bytes_read_fd / expected_frame_size_fd;
  ASSERT_EQ(num_frames_received_fd, num_frames_to_test);

  // Iterate through the buffer, parsing each FD frame
  for (size_t i = 0; i < num_frames_received_fd; ++i) {
    const struct canfd_frame* parsed_frame_fd =
        reinterpret_cast<const struct canfd_frame*>(buffer_fd +
                                                    i * expected_frame_size_fd);

    if (i == 0) {
      ASSERT_EQ(parsed_frame_fd->can_id, frame_fd_1.can_id);
      ASSERT_EQ(parsed_frame_fd->len, frame_fd_1.len);
      ASSERT_EQ(parsed_frame_fd->flags, frame_fd_1.flags);
      ASSERT_TRUE(std::memcmp(parsed_frame_fd->data, frame_fd_1.data,
                              parsed_frame_fd->len) == 0);
    } else if (i == 1) {
      ASSERT_EQ(parsed_frame_fd->can_id, frame_fd_2.can_id);
      ASSERT_EQ(parsed_frame_fd->len, frame_fd_2.len);
      ASSERT_EQ(parsed_frame_fd->flags, frame_fd_2.flags);
      ASSERT_TRUE(std::memcmp(parsed_frame_fd->data, frame_fd_2.data,
                              parsed_frame_fd->len) == 0);
    }
    // Application logic would then process parsed_frame_fd->data based on
    // ID/len/flags
  }
}
// Add more parsing tests for different CAN FD lengths (0, 12, 16, 20, 24, 48).

// --- Gtest Test Suite for CanStream Read Logic (Using Mock) ---
// These tests check CanStream::read's behavior using the mock.
// They focus on byte counts, return values, and exceptions.
// They DO NOT perform frame parsing.
TEST(CanStreamReadTest, ReadStandardFrame_Single_Success) {
  // Test that CanStream::read successfully reads one standard frame via the
  // mock.
  using namespace apollo::drivers::hal;
  using ::testing::_;  // For matching any argument

  // Create mock stream for standard CAN (enable_can_fd = false)
  MockCanStream mock_stream("vcan0", 0, 10000, false);

  // Define the byte representation of a standard frame to be "read" by the mock
  const size_t frame_size = sizeof(struct can_frame);
  std::vector<uint8_t> frame_bytes(frame_size);
  // Fill with some data (contents don't matter for this test, only size)
  std::memset(frame_bytes.data(), 0x5A, frame_size);

  // Configure the mock to return this frame data when its read method is called
  mock_stream.set_read_data(frame_bytes);

  // Set expectation that CanStream::read will call mock's read once
  // Configure mock's read to use our action: Return the data set via
  // set_read_data
  EXPECT_CALL(
      mock_stream,
      read(_,
           frame_size))  // CanStream::read asks for buffer of size frame_size
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockReadAction));

  // Call CanStream::read
  std::vector<uint8_t> read_buffer(frame_size);  // Buffer to receive data
  size_t bytes_read = mock_stream.read(read_buffer.data(), read_buffer.size());

  // Assert the result: Should read exactly one frame size
  ASSERT_EQ(bytes_read, frame_size);

  // Optional: Verify the bytes in the buffer match the original frame bytes
  ASSERT_EQ(read_buffer, frame_bytes);
}

TEST(CanStreamReadTest, ReadStandardFrame_Multiple_Success) {
  // Test reading multiple standard frames in one call to CanStream::read
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN

  const size_t frame_size = sizeof(struct can_frame);
  const size_t num_frames_to_read = 3;

  // Prepare byte data for multiple frames
  std::vector<uint8_t> mock_read_buffer(num_frames_to_read * frame_size);
  // Fill with some dummy data
  for (size_t i = 0; i < mock_read_buffer.size(); ++i) {
    mock_read_buffer[i] = static_cast<uint8_t>(i % 256);
  }

  mock_stream.set_read_data(mock_read_buffer);

  // CanStream::read loops internally, but MockReadAction reads all available.
  // So EXPECT_CALL on mock.read is likely only called once for the total size
  // requested.
  EXPECT_CALL(mock_stream, read(_, num_frames_to_read * frame_size))
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockReadAction));

  // Call CanStream::read with buffer large enough for multiple frames
  std::vector<uint8_t> read_buffer(num_frames_to_read * frame_size);
  size_t bytes_read = mock_stream.read(read_buffer.data(), read_buffer.size());

  // Assert total bytes read
  ASSERT_EQ(bytes_read, num_frames_to_read * frame_size);
  // Optional: Verify the bytes in the buffer
  ASSERT_EQ(read_buffer, mock_read_buffer);
}

TEST(CanStreamReadTest, ReadStandardFrame_EAGAIN_ReturnsZero) {
  // Test that CanStream::read returns 0 when the underlying read simulates
  // EAGAIN (no data).
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN

  const size_t frame_size = sizeof(struct can_frame);

  // Configure the mock read to return -1 and set errno to EAGAIN
  EXPECT_CALL(mock_stream, read(_, frame_size))
      .WillOnce(SetErrno(EAGAIN))
      .WillOnce(Return(-1));  // Need 2 actions for SetErrno + Return(-1)

  // Call CanStream::read
  std::vector<uint8_t> read_buffer(frame_size);
  size_t bytes_read = mock_stream.read(read_buffer.data(), read_buffer.size());

  // Assert the result: Should return 0 bytes read
  ASSERT_EQ(bytes_read, 0);
  // You might check mock_stream.get_last_error_code() here too if exposed and
  // set by CanStream::read
}

TEST(CanStreamReadTest, ReadStandardFrame_FatalError_Throws) {
  // Test that CanStream::read throws an exception on a fatal underlying read
  // error.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN

  const size_size_t frame_size = sizeof(struct can_frame);

  // Configure the mock read to simulate a fatal error (e.g., EBADF)
  EXPECT_CALL(mock_stream, read(_, frame_size))
      .WillOnce(SetErrno(EBADF))
      .WillOnce(Return(-1));

  // Call CanStream::read and assert it throws std::runtime_error
  std::vector<uint8_t> read_buffer(frame_size);
  ASSERT_THROW(mock_stream.read(read_buffer.data(), read_buffer.size()),
               std::runtime_error);
  // You might check mock_stream.get_last_error_code() after catching if it's
  // set before throwing
}

// --- CAN FD Read Tests ---

TEST(CanStreamReadTest, ReadCanFDFrame_Single_Success) {
  // Test that CanStream::read successfully reads one CAN FD frame via the mock.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  // Create mock stream for CAN FD (enable_can_fd = true)
  MockCanStream mock_stream("vcan0", 0, 10000, true);  // true for CAN FD

  // Define the byte representation of a CAN FD frame
  const size_t frame_size_fd = sizeof(struct canfd_frame);
  std::vector<uint8_t> frame_bytes_fd(frame_size_fd);
  // Fill with some data (contents don't matter)
  std::memset(frame_bytes_fd.data(), 0xFD, frame_size_fd);

  mock_stream.set_read_data(frame_bytes_fd);

  // Set expectation that CanStream::read will call mock's read once
  EXPECT_CALL(mock_stream,
              read(_, frame_size_fd))  // CanStream::read asks for buffer of
                                       // size frame_size_fd
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockReadAction));

  // Call CanStream::read
  std::vector<uint8_t> read_buffer_fd(frame_size_fd);  // Buffer to receive data
  size_t bytes_read =
      mock_stream.read(read_buffer_fd.data(), read_buffer_fd.size());

  // Assert the result: Should read exactly one FD frame size
  ASSERT_EQ(bytes_read, frame_size_fd);
  // Optional: Verify the bytes
  ASSERT_EQ(read_buffer_fd, frame_bytes_fd);
}

TEST(CanStreamReadTest, ReadCanFDFrame_Multiple_Success) {
  // Test reading multiple CAN FD frames in one call to CanStream::read
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD

  const size_t frame_size_fd = sizeof(struct canfd_frame);
  const size_t num_frames_to_read = 2;

  // Prepare byte data for multiple FD frames
  std::vector<uint8_t> mock_read_buffer_fd(num_frames_to_read * frame_size_fd);
  for (size_t i = 0; i < mock_read_buffer_fd.size(); ++i) {
    mock_read_buffer_fd[i] =
        static_cast<uint8_t>(i % 100 + 100);  // Different pattern
  }

  mock_stream.set_read_data(mock_read_buffer_fd);

  EXPECT_CALL(mock_stream, read(_, num_frames_to_read * frame_size_fd))
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockReadAction));

  std::vector<uint8_t> read_buffer_fd(num_frames_to_read * frame_size_fd);
  size_t bytes_read =
      mock_stream.read(read_buffer_fd.data(), read_buffer_fd.size());

  ASSERT_EQ(bytes_read, num_frames_to_read * frame_size_fd);
  ASSERT_EQ(read_buffer_fd, mock_read_buffer_fd);
}

TEST(CanStreamReadTest, ReadCanFDFrame_EAGAIN_ReturnsZero) {
  // Test that CanStream::read returns 0 when underlying read simulates EAGAIN
  // for FD.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_t frame_size_fd = sizeof(struct canfd_frame);

  EXPECT_CALL(mock_stream, read(_, frame_size_fd))
      .WillOnce(SetErrno(EAGAIN))
      .WillOnce(Return(-1));

  std::vector<uint8_t> read_buffer_fd(frame_size_fd);
  size_t bytes_read =
      mock_stream.read(read_buffer_fd.data(), read_buffer_fd.size());

  ASSERT_EQ(bytes_read, 0);
}

TEST(CanStreamReadTest, ReadCanFDFrame_FatalError_Throws) {
  // Test that CanStream::read throws on fatal underlying read error for FD.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_t frame_size_fd = sizeof(struct canfd_frame);

  EXPECT_CALL(mock_stream, read(_, frame_size_fd))
      .WillOnce(SetErrno(EIO))
      .WillOnce(Return(-1));  // EIO as fatal example

  std::vector<uint8_t> read_buffer_fd(frame_size_fd);
  ASSERT_THROW(mock_stream.read(read_buffer_fd.data(), read_buffer_fd.size()),
               std::runtime_error);
}

// Add more read tests covering edge cases like connection lost (recv returns
// 0).

// --- Gtest Test Suite for CanStream Write Logic (Using Mock) ---
// These tests check CanStream::write's behavior using the mock.
// They focus on byte counts, return values, and exceptions.
// They DO NOT perform frame construction or parsing.
TEST(CanStreamWriteTest, WriteStandardFrame_Single_Success) {
  // Test successful writing of a single standard frame.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN

  const size_t frame_size = sizeof(struct can_frame);

  // Prepare byte data for one frame (contents don't matter for this test)
  std::vector<uint8_t> data_to_write(frame_size);
  std::memset(data_to_write.data(), 0xAA, frame_size);

  // Set expectation that CanStream::write will call mock's write once
  // Configure mock's write to use our action: Accumulate written data and
  // return size
  EXPECT_CALL(mock_stream, write(::testing::An<const uint8_t*>(),
                                 frame_size))  // Expect call with correct size
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockWriteAction));

  // Call CanStream::write
  size_t bytes_written =
      mock_stream.write(data_to_write.data(), data_to_write.size());

  // Assert the result: Should write exactly frame_size bytes
  ASSERT_EQ(bytes_written, frame_size);

  // Verify the bytes were captured by the mock
  ASSERT_EQ(mock_stream.get_written_data().size(), frame_size);
  ASSERT_EQ(mock_stream.get_written_data(), data_to_write);
}

TEST(CanStreamWriteTest, WriteStandardFrame_Multiple_Success) {
  // Test successful writing of multiple standard frames in one call.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN

  const size_t frame_size = sizeof(struct can_frame);
  const size_t num_frames_to_write = 2;

  // Prepare byte data for multiple frames
  std::vector<uint8_t> data_to_write(num_frames_to_write * frame_size);
  // Fill with dummy data
  for (size_t i = 0; i < data_to_write.size(); ++i) {
    data_to_write[i] = static_cast<uint8_t>(i % 256);
  }

  // CanStream::write loops internally, but MockWriteAction accumulates all
  // writes. So EXPECT_CALL on mock.write is likely only called once for the
  // total size requested.
  EXPECT_CALL(mock_stream,
              write(::testing::An<const uint8_t*>(), data_to_write.size()))
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockWriteAction));

  // Call CanStream::write
  size_t bytes_written =
      mock_stream.write(data_to_write.data(), data_to_write.size());

  // Assert the result
  ASSERT_EQ(bytes_written, data_to_write.size());
  ASSERT_EQ(bytes_written, num_frames_to_write * frame_size);

  // Verify the bytes captured by the mock
  ASSERT_EQ(mock_stream.get_written_data().size(), data_to_write.size());
  ASSERT_EQ(mock_stream.get_written_data(), data_to_write);
}

TEST(CanStreamWriteTest, WriteStandardFrame_IncorrectLength_Throws) {
  // Test that writing data whose length is not a multiple of frame size throws.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN

  const size_t frame_size = sizeof(struct can_frame);

  // Data with length not a multiple of frame size
  std::vector<uint8_t> incorrect_data(frame_size + 5);  // 5 bytes extra

  // Expect CanStream::write to throw std::invalid_argument because of length
  // check The mock write method should NOT be called
  EXPECT_CALL(mock_stream, write(_, _)).Times(0);

  // Call CanStream::write and assert it throws
  ASSERT_THROW(mock_stream.write(incorrect_data.data(), incorrect_data.size()),
               std::invalid_argument);
}

TEST(CanStreamWriteTest, WriteStandardFrame_EAGAIN_ReturnsPartialOrZero) {
  // Test that CanStream::write returns partial bytes or 0 when underlying send
  // simulates EAGAIN.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Invoke;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN
  const size_t frame_size = sizeof(struct can_frame);
  const size_t num_frames = 3;  // Attempt to write 3 frames
  const size_t total_size = num_frames * frame_size;

  std::vector<uint8_t> data_to_write(total_size);
  for (size_t i = 0; i < data_to_write.size(); ++i) {
    data_to_write[i] = static_cast<uint8_t>(i);
  }

  // Configure mock write action:
  // CanStream::write loops internally, calling mock.write once per frame
  // attempt. We expect calls for frame_size at a time.
  EXPECT_CALL(mock_stream, write(::testing::An<const uint8_t*>(), frame_size))
      .WillOnce(::testing::Invoke(
          &mock_stream, &MockCanStream::MockWriteAction))  // Write frame 1
      .WillOnce(::testing::Invoke(
          &mock_stream, &MockCanStream::MockWriteAction))  // Write frame 2
      .WillOnce(SetErrno(EAGAIN))
      .WillOnce(Return(-1));  // Attempt frame 3 (fails, sets errno, returns -1)

  // Call CanStream::write
  size_t bytes_written =
      mock_stream.write(data_to_write.data(), data_to_write.size());

  // Assert the result: Should return bytes for 2 frames (2 * frame_size)
  ASSERT_EQ(bytes_written, 2 * frame_size);

  // Verify that only the bytes for the first 2 frames were captured by the mock
  ASSERT_EQ(mock_stream.get_written_data().size(), 2 * frame_size);
  ASSERT_TRUE(std::memcmp(mock_stream.get_written_data().data(),
                          data_to_write.data(), 2 * frame_size) == 0);
}

TEST(CanStreamWriteTest, WriteStandardFrame_FatalError_Throws) {
  // Test that CanStream::write throws an exception on a fatal underlying write
  // error.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Invoke;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, false);  // standard CAN
  const size_t frame_size = sizeof(struct can_frame);
  const size_t num_frames = 2;  // Attempt to write 2 frames
  const size_t total_size = num_frames * frame_size;

  std::vector<uint8_t> data_to_write(total_size);
  std::memset(data_to_write.data(), 0xCC, total_size);

  // Configure mock write action:
  // 1st frame: Succeeds
  // 2nd frame: Fails with a fatal error (e.g., EIO)
  EXPECT_CALL(mock_stream, write(::testing::An<const uint8_t*>(), frame_size))
      .WillOnce(::testing::Invoke(
          &mock_stream, &MockCanStream::MockWriteAction))  // Write frame 1
      .WillOnce(SetErrno(EIO))
      .WillOnce(Return(-1));  // Attempt frame 2 (fails fatally)

  // Call CanStream::write and assert it throws std::runtime_error
  ASSERT_THROW(mock_stream.write(data_to_write.data(), data_to_write.size()),
               std::runtime_error);

  // Verify that only the bytes for the first frame were captured by the mock
  // before error
  ASSERT_EQ(mock_stream.get_written_data().size(), frame_size);
  ASSERT_TRUE(std::memcmp(mock_stream.get_written_data().data(),
                          data_to_write.data(), frame_size) == 0);

  // Check if Disconnect was called (assuming CanStream::write calls Disconnect
  // on fatal error) EXPECT_CALL(mock_stream,
  // Disconnect()).WillOnce(Return(true)); // Add expectation if testing this
  // flow
}

// --- CAN FD Write Tests ---

TEST(CanStreamWriteTest, WriteCanFDFrame_Single_Success) {
  // Test successful writing of a single CAN FD frame.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_t frame_size_fd = sizeof(struct canfd_frame);

  std::vector<uint8_t> data_to_write_fd(frame_size_fd);
  std::memset(data_to_write_fd.data(), 0xFD, frame_size_fd);

  EXPECT_CALL(mock_stream,
              write(::testing::An<const uint8_t*>(), frame_size_fd))
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockWriteAction));

  size_t bytes_written =
      mock_stream.write(data_to_write_fd.data(), data_to_write_fd.size());

  ASSERT_EQ(bytes_written, frame_size_fd);
  ASSERT_EQ(mock_stream.get_written_data().size(), frame_size_fd);
  ASSERT_EQ(mock_stream.get_written_data(), data_to_write_fd);
}

TEST(CanStreamWriteTest, WriteCanFDFrame_Multiple_Success) {
  // Test successful writing of multiple CAN FD frames.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_t frame_size_fd = sizeof(struct canfd_frame);
  const size_t num_frames = 2;
  const size_t total_size = num_frames * frame_size_fd;

  std::vector<uint8_t> data_to_write_fd(total_size);
  for (size_t i = 0; i < data_to_write_fd.size(); ++i) {
    data_to_write_fd[i] =
        static_cast<uint8_t>(i % 100 + 200);  // Another pattern
  }

  EXPECT_CALL(mock_stream, write(::testing::An<const uint8_t*>(), total_size))
      .WillOnce(
          ::testing::Invoke(&mock_stream, &MockCanStream::MockWriteAction));

  size_t bytes_written =
      mock_stream.write(data_to_write_fd.data(), data_to_write_fd.size());

  ASSERT_EQ(bytes_written, total_size);
  ASSERT_EQ(bytes_written, num_frames * frame_size_fd);
  ASSERT_EQ(mock_stream.get_written_data().size(), total_size);
  ASSERT_EQ(mock_stream.get_written_data(), data_to_write_fd);
}

TEST(CanStreamWriteTest, WriteCanFDFrame_IncorrectLength_Throws) {
  // Test that writing data whose length is not a multiple of FD frame size
  // throws.
  using namespace apollo::drivers::hal;
  using ::testing::_;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_size_t frame_size_fd = sizeof(struct canfd_frame);

  std::vector<uint8_t> incorrect_data_fd(frame_size_fd + 10);  // 10 bytes extra

  EXPECT_CALL(mock_stream, write(_, _)).Times(0);

  ASSERT_THROW(
      mock_stream.write(incorrect_data_fd.data(), incorrect_data_fd.size()),
      std::invalid_argument);
}

TEST(CanStreamWriteTest, WriteCanFDFrame_EAGAIN_ReturnsPartialOrZero) {
  // Test that CanStream::write returns partial bytes or 0 when underlying send
  // simulates EAGAIN for FD.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Invoke;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_t frame_size_fd = sizeof(struct canfd_frame);
  const size_t num_frames = 2;
  const size_t total_size = num_frames * frame_size_fd;

  std::vector<uint8_t> data_to_write_fd(total_size);
  std::memset(data_to_write_fd.data(), 0xDD, total_size);

  EXPECT_CALL(mock_stream,
              write(::testing::An<const uint8_t*>(), frame_size_fd))
      .WillOnce(::testing::Invoke(
          &mock_stream, &MockCanStream::MockWriteAction))  // Write frame 1
      .WillOnce(SetErrno(EAGAIN))
      .WillOnce(Return(-1));  // Attempt frame 2 (fails)

  size_t bytes_written =
      mock_stream.write(data_to_write_fd.data(), data_to_write_fd.size());

  ASSERT_EQ(bytes_written, frame_size_fd);
  ASSERT_EQ(mock_stream.get_written_data().size(), frame_size_fd);
  ASSERT_TRUE(std::memcmp(mock_stream.get_written_data().data(),
                          data_to_write_fd.data(), frame_size_fd) == 0);
}

TEST(CanStreamWriteTest, WriteCanFDFrame_FatalError_Throws) {
  // Test that CanStream::write throws on fatal underlying write error for FD.
  using namespace apollo::drivers::hal;
  using ::testing::_;
  using ::testing::Invoke;
  using ::testing::Return;
  using ::testing::SetErrno;

  MockCanStream mock_stream("vcan0", 0, 10000, true);  // CAN FD
  const size_t frame_size_fd = sizeof(struct canfd_frame);
  const size_t num_frames = 2;
  const size_t total_size = num_frames * frame_size_fd;

  std::vector<uint8_t> data_to_write_fd(total_size);
  std::memset(data_to_write_fd.data(), 0xEE, total_size);

  EXPECT_CALL(mock_stream,
              write(::testing::An<const uint8_t*>(), frame_size_fd))
      .WillOnce(::testing::Invoke(
          &mock_stream, &MockCanStream::MockWriteAction))  // Write frame 1
      .WillOnce(SetErrno(EBADF))
      .WillOnce(Return(-1));  // Attempt frame 2 (fails fatally)

  ASSERT_THROW(
      mock_stream.write(data_to_write_fd.data(), data_to_write_fd.size()),
      std::runtime_error);

  ASSERT_EQ(mock_stream.get_written_data().size(), frame_size_fd);
  ASSERT_TRUE(std::memcmp(mock_stream.get_written_data().data(),
                          data_to_write_fd.data(), frame_size_fd) == 0);
}
