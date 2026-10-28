#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

#include <gtest/gtest.h>

#include "modules/drivers/hal/stream/can_stream.h"
#include "modules/drivers/hal/stream/can_stream_utils.h"

namespace apollo {
namespace drivers {
namespace hal {

TEST(CanStreamRegressionTest, FailedConnectCanBeRetried) {
  CanStream stream("zz_no_can_if", 0);

  EXPECT_FALSE(stream.Connect());
  EXPECT_FALSE(stream.Connect());
}

TEST(CanStreamRegressionTest, ZeroTimeoutEnablesNonBlockingSocket) {
  int sockets[2] = {-1, -1};
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_STREAM, 0, sockets), 0);

  const int nonblocking_result = internal::SetCanSocketNonBlocking(sockets[0]);
  const int flags = fcntl(sockets[0], F_GETFL, 0);
  close(sockets[0]);
  close(sockets[1]);

  EXPECT_EQ(nonblocking_result, 0);
  ASSERT_GE(flags, 0);
  EXPECT_NE(flags & O_NONBLOCK, 0);
}

TEST(CanStreamRegressionTest, FdModeAcceptsClassicalAndFdFrameSizes) {
  EXPECT_TRUE(internal::IsCanFrameSizeValid(sizeof(can_frame), true));
  EXPECT_TRUE(internal::IsCanFrameSizeValid(sizeof(canfd_frame), true));
  EXPECT_FALSE(internal::IsCanFrameSizeValid(sizeof(canfd_frame), false));

  can_frame classical_frame{};
  classical_frame.can_id = 0x123;
  classical_frame.can_dlc = 2;
  classical_frame.data[0] = 0xA5;
  classical_frame.data[1] = 0x5A;

  canfd_frame slot{};
  std::memset(&slot, 0xFF, sizeof(slot));
  internal::StoreCanFrame(reinterpret_cast<uint8_t*>(&slot), sizeof(slot),
                          reinterpret_cast<const uint8_t*>(&classical_frame),
                          sizeof(classical_frame));

  EXPECT_EQ(slot.can_id, classical_frame.can_id);
  EXPECT_EQ(slot.len, classical_frame.can_dlc);
  EXPECT_EQ(slot.data[0], classical_frame.data[0]);
  EXPECT_EQ(slot.data[1], classical_frame.data[1]);
  EXPECT_EQ(slot.data[8], 0);
}

}  // namespace hal
}  // namespace drivers
}  // namespace apollo
