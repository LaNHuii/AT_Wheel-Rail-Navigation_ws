#include "wr_chassis_gateway/socketcan_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace wr_chassis_gateway
{

SocketCanTransport::~SocketCanTransport()
{
  close();
}

void SocketCanTransport::open(const std::string & interface_name)
{
  close();
  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    throw std::runtime_error("failed to create SocketCAN socket: " + std::string(std::strerror(errno)));
  }
  const int enable_can_fd = 1;
  if (::setsockopt(
      socket_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
      &enable_can_fd, sizeof(enable_can_fd)) < 0)
  {
    const auto reason = std::string(std::strerror(errno));
    close();
    throw std::runtime_error("failed to enable CAN-FD: " + reason);
  }
  struct ifreq request {};
  if (interface_name.size() >= IFNAMSIZ) {
    close();
    throw std::invalid_argument("SocketCAN interface name is too long");
  }
  std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &request) < 0) {
    const auto reason = std::string(std::strerror(errno));
    close();
    throw std::runtime_error("SocketCAN interface lookup failed: " + reason);
  }
  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    const auto reason = std::string(std::strerror(errno));
    close();
    throw std::runtime_error("SocketCAN bind failed: " + reason);
  }
  const int flags = ::fcntl(socket_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_, F_SETFL, flags | O_NONBLOCK) < 0) {
    const auto reason = std::string(std::strerror(errno));
    close();
    throw std::runtime_error("SocketCAN nonblocking setup failed: " + reason);
  }
}

void SocketCanTransport::close()
{
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool SocketCanTransport::is_open() const
{
  return socket_ >= 0;
}

bool SocketCanTransport::send(const can_protocol::CanFdFrame & frame) const
{
  if (socket_ < 0 || frame.length > can_protocol::kMaxDataLength) {
    return false;
  }
  struct canfd_frame wire {};
  wire.can_id = frame.id & CAN_SFF_MASK;
  wire.len = frame.length;
  std::copy_n(frame.data.begin(), frame.length, wire.data);
  return ::write(socket_, &wire, CANFD_MTU) == CANFD_MTU;
}

bool SocketCanTransport::receive(can_protocol::CanFdFrame & frame) const
{
  if (socket_ < 0) {
    return false;
  }
  struct canfd_frame wire {};
  const auto bytes = ::read(socket_, &wire, CANFD_MTU);
  if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return false;
  }
  if (bytes != CANFD_MTU || (wire.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG))) {
    return false;
  }
  frame = can_protocol::CanFdFrame{};
  frame.id = wire.can_id & CAN_SFF_MASK;
  frame.length = wire.len;
  std::copy_n(wire.data, wire.len, frame.data.begin());
  return true;
}

}  // namespace wr_chassis_gateway
