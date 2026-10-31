// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace apollo::localization::openloc_diagnostic::recorded {
using Rows = std::vector<std::vector<std::string>>;

inline Rows Read(const std::filesystem::path& path, const std::string& header,
                 std::size_t columns) {
  std::ifstream stream(path);
  std::string line;
  const auto next = [&] {
    if (!std::getline(stream, line)) return false;
    if (!line.empty() && line.back() == '\r') line.pop_back();
    return true;
  };
  if (!stream || !next() || line != header)
    throw std::runtime_error("invalid input/header: " + path.string());
  Rows rows;
  while (next()) {
    if (line.empty() || line.back() == ',' ||
        line.find('"') != std::string::npos)
      throw std::runtime_error("invalid raw replay row");
    std::istringstream cells(line);
    std::vector<std::string> row;
    std::string field;
    while (std::getline(cells, field, ',')) row.push_back(field);
    if (row.size() != columns)
      throw std::runtime_error("invalid raw replay column count");
    rows.push_back(std::move(row));
  }
  if (!stream.eof()) throw std::runtime_error("incomplete raw replay input");
  return rows;
}

inline int64_t Time(const std::string& text) {
  std::size_t used = 0;
  const auto value = std::stoll(text, &used);
  if (used != text.size() || value <= 0)
    throw std::runtime_error("invalid positive source timestamp");
  return value;
}

inline double Number(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value))
    throw std::runtime_error("invalid finite source number");
  return value;
}
}  // namespace apollo::localization::openloc_diagnostic::recorded
