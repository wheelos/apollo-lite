#pragma once

#include <cstddef>
#include <cstdint>

namespace apollo {
namespace localization {
namespace msf {

class MD5 {
 public:
  MD5();
  void init();
  void update(const unsigned char* input, unsigned int input_len);
  void finalize();

  unsigned char digest[16];

 private:
  uint32_t state[4];
  uint32_t count[2];
  unsigned char buffer[64];

  void transform(const unsigned char block[64]);
  static void encode(unsigned char* output, const uint32_t* input,
                     unsigned int len);
  static void decode(uint32_t* output, const unsigned char* input,
                     unsigned int len);
};

}  // namespace msf
}  // namespace localization
}  // namespace apollo
