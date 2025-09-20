/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "modules/drivers/gnss/proto/config.pb.h"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"
#include "modules/drivers/gnss/util/data_buffer.h"

namespace apollo {
namespace drivers {
namespace gnss {

// An abstract class of Parser for GNSS data streams.
// Derived classes implement parsing logic for specific data formats.
class Parser {
 public:
  // States for the parsing finite state machine.
  enum class ParseState {
    SEEK_HEADER,     // Looking for the start of a message/packet header.
    PROCESS_PAYLOAD  // Header found, processing the message payload.
  };

  // Types of messages that can be parsed.
  enum class MessageType {
    NONE,
    GNSS,
    GNSS_RANGE,
    IMU,
    INS,
    INS_STAT,
    WHEEL,
    EPHEMERIDES,
    OBSERVATION,
    BDGGA,
    GPGGA,
    BDSEPHEMERIDES,
    RAWIMU,
    GPSEPHEMERIDES,
    GLOEPHEMERIDES,
    BEST_GNSS_POS,
    HEADING,
  };

  // A general pointer to a protobuf message. Used for parsed results.
  using ProtoMessagePtr = std::shared_ptr<::google::protobuf::Message>;
  using RawDataPtr = std::shared_ptr<std::vector<uint8_t>>;

  // Represents a successfully parsed message.
  // It's a pair of the message type and a variant holding either a protobuf
  // message or raw bytes (for formats not fully converted to protobuf
  // internally).
  using ParsedMessage =
      std::pair<MessageType, std::variant<ProtoMessagePtr, RawDataPtr>>;

  virtual ~Parser() = default;

  // Static factory method to create a Parser instance based on configuration.
  // This method is implemented in the .cpp file.
  static std::unique_ptr<Parser> Create(const config::Config &config);

  void AppendData(const uint8_t *data, size_t length) {
    return buffer_.Append(data, length);
  }

  void AppendData(std::string_view data) { return buffer_.Append(data); }

  // Parses all complete messages currently available in the buffer.
  // Calls TryParseMessage repeatedly until no more messages can be parsed
  // or no progress is made on the buffer.
  // Returns a vector of all successfully parsed messages.
  inline virtual std::vector<ParsedMessage> ParseAllMessages();

 protected:
  // Protected constructor enforces creation through the static Create method or
  // derived classes.
  explicit Parser(size_t buffer_max_size = 4096)
      : buffer_(buffer_max_size), state_(ParseState::SEEK_HEADER) {}

  // Tries to parse a single message or make progress towards parsing a message
  // from the internal buffer based on the current state.
  // This method manages the state transitions and calls the virtual methods.
  // Returns:
  // - An optional vector of ParsedMessage if one or more messages were
  // successfully
  //   parsed in this step (state transitions to SEEK_HEADER).
  // - std::nullopt if no complete message was parsed in this call (either
  //   still seeking header, processing payload, or buffer is incomplete).
  inline std::optional<std::vector<ParsedMessage>> TryParseMessage();

  // Pure virtual method to be implemented by derived classes.
  // This method is called when the parser is in the SEEK_HEADER state.
  // It should search the buffer for a valid header pattern and consume
  // any leading garbage data.
  // Implementation must interact with 'buffer_'.
  // Returns:
  // - true if a valid header is found and the buffer is positioned at the start
  //   of the payload (or the header itself if needed for payload processing).
  // - false if no valid header is found in the current buffer content
  //   (might need more data, or data is invalid).
  // If false is returned, the implementation *must* ensure that the buffer
  // state is updated appropriately to prevent infinite loops (e.g., consume
  // some data if possible, or rely on ParseAllMessages' progress check).
  virtual bool ProcessHeader() = 0;

  // Pure virtual method to be implemented by derived classes.
  // This method is called when the parser is in the PROCESS_PAYLOAD state.
  // It should attempt to extract a complete message payload from the buffer,
  // parse it, and generate the corresponding ParsedMessage(s).
  // Implementation must interact with 'buffer_'.
  // Returns:
  // - An optional vector of ParsedMessage if one or more messages were
  // successfully
  //   parsed from the current payload chunk. The implementation must consume
  //   the corresponding data from 'buffer_'.
  // - std::nullopt if the payload is incomplete (need more data) or invalid.
  //   If std::nullopt is returned due to incompleteness, the buffer should not
  //   have been fully consumed for this payload. If returned due to invalidity,
  //   the implementation should handle error recovery and potentially
  //   consume/skip invalid data to prevent infinite loops.
  virtual std::optional<std::vector<ParsedMessage>> ProcessPayload() = 0;

  // Internal data buffer holding raw input bytes.
  DataBuffer buffer_;

  // Current state of the parsing state machine.
  ParseState state_;

 private:
  DISALLOW_COPY_AND_ASSIGN(Parser);
};

std::optional<std::vector<Parser::ParsedMessage>> Parser::TryParseMessage() {
  switch (state_) {
    case ParseState::SEEK_HEADER: {
      // Try to find and process a header.
      if (ProcessHeader()) {
        // Header found, transition to processing payload.
        state_ = ParseState::PROCESS_PAYLOAD;
        // Return nullopt as we haven't processed a full message yet, just the
        // header.
        return std::nullopt;
      }
      // Header not found or buffer incomplete, stay in SEEK_HEADER.
      return std::nullopt;
    }
    case ParseState::PROCESS_PAYLOAD: {
      // Try to process the payload and get messages.
      auto parsed_msg = ProcessPayload();
      if (parsed_msg.has_value()) {
        // Payload processed successfully (even if 0 messages were yielded but
        // data was consumed). Transition back to seeking the next header.
        state_ = ParseState::SEEK_HEADER;
        // Return the parsed messages (could be an empty vector if payload was
        // just consumption).
        return parsed_msg;
      }
      // Payload incomplete or invalid, stay in PROCESS_PAYLOAD or transition
      // based on ProcessPayload's handling. ProcessPayload returning
      // std::nullopt indicates insufficient data or unrecoverable error *for
      // the current payload*.
      return std::nullopt;
    }
    default:
      // Should not happen, but handle unexpected states gracefully.
      AINFO << "Unknown parser state: " << static_cast<int>(state_)
            << ". Resetting to SEEK_HEADER.";
      state_ = ParseState::SEEK_HEADER;
      return std::nullopt;
  }
}

std::vector<Parser::ParsedMessage> Parser::ParseAllMessages() {
  std::vector<Parser::ParsedMessage> parsed_messages;

  while (true) {
    // Store buffer ReadableBytes before attempting to parse.
    size_t initial_buffer_size = buffer_.ReadableBytes();

    // Try to parse one unit (header or payload).
    auto result = TryParseMessage();

    if (result.has_value()) {
      // Successfully parsed one or more messages from a payload.
      std::move(result.value().begin(), result.value().end(),
                std::back_inserter(parsed_messages));
    } else {
      // TryParseMessage returned std::nullopt.
      // This means either:
      // 1. We are in SEEK_HEADER and couldn't find a header (buffer incomplete
      // or no header).
      // 2. We are in PROCESS_PAYLOAD and couldn't complete the payload (buffer
      // incomplete or invalid).

      // Check if any data was consumed by the call to TryParseMessage (via
      // ProcessHeader/ProcessPayload).
      if (buffer_.ReadableBytes() < initial_buffer_size) {
        // Data was consumed. This could be:
        // - Skipping leading garbage in SEEK_HEADER.
        // - Partially processing a header or payload.
        // - Recovering from an error in ProcessPayload by skipping invalid
        // bytes. Since progress (data consumption) was made, continue the loop
        // to try again with the potentially smaller buffer.
        ADEBUG
            << "Parser consumed data without yielding a message. Buffer ReadableBytes "
               "decreased from "
            << initial_buffer_size << " to " << buffer_.ReadableBytes();
        continue;
      } else {
        // No data was consumed, and no message was parsed.
        // This indicates that the parser is stuck - it needs more data to
        // proceed or the remaining data is insufficient/invalid for *any*
        // further processing *right now*. Break the loop as no progress can be
        // made with the current buffer content.
        ADEBUG << "Parser made no progress. Buffer ReadableBytes remains "
               << buffer_.ReadableBytes()
               << ". Stopping parsing until more data arrives.";
        break;
      }
    }
  }

  return parsed_messages;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
