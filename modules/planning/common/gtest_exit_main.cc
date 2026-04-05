#include <cstdio>
#include <cstdlib>

#include "gtest/gtest.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  std::fflush(nullptr);
  std::_Exit(result);
}