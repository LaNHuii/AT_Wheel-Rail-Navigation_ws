#pragma once

#include <string>

#include "wr_chassis_gateway/can_protocol.hpp"

namespace wr_chassis_gateway
{

class SocketCanTransport
{
public:
  SocketCanTransport() = default;
  ~SocketCanTransport();
  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport & operator=(const SocketCanTransport &) = delete;

  void open(const std::string & interface_name);
  void close();
  bool is_open() const;
  bool send(const can_protocol::CanFdFrame & frame) const;
  bool receive(can_protocol::CanFdFrame & frame) const;

private:
  int socket_{-1};
};

}  // namespace wr_chassis_gateway
