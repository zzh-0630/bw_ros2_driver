#include "bw_ros_driver/can_port.hpp"

// System headers
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// Standard headers
#include <cerrno>
#include <cstring>
#include <vector>

namespace bw
{
CanPort::CanPort(const std::string& can_interface) :
  can_interface_(can_interface),
  socket_fd_(-1)
{
}

CanPort::~CanPort()
{
  closeCan();
}

bool CanPort::openCan(const std::vector<uint32_t>& filter_ids)
{
  closeCan();

  // Create socket
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);

  if (socket_fd_ < 0)
  {
    return false;
  }

  // Resolve interface index
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", can_interface_.c_str());

  if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    closeCan();
    return false;
  }

  // Bind
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    closeCan();
    return false;
  }

  // Optional filters (reduce CPU + unrelated frames)
  if (!filter_ids.empty())
  {
    std::vector<struct can_filter> filters;
    filters.reserve(filter_ids.size());

    for (const uint32_t id : filter_ids)
    {
      struct can_filter f;
      f.can_id = id & CAN_SFF_MASK;
      f.can_mask = CAN_SFF_MASK;
      filters.push_back(f);
    }

    if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(filters.size() * sizeof(filters[0]))) < 0)
    {
      closeCan();
      return false;
    }
  }

  // Disable receive own frames
  int recv_own = 0;
  (void)::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own));

  return true;
}

void CanPort::closeCan()
{
  if (socket_fd_ >= 0)
  {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

int CanPort::readFrame(struct can_frame& refOut, int timeout_ms)
{
  if (socket_fd_ < 0)
  {
    errno = EBADF;
    return -1;
  }
  
  struct pollfd pfd;
  pfd.fd = socket_fd_;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const int pr = ::poll(&pfd, 1, timeout_ms);

  if (pr == 0)
  {
    return 0;
  }

  if (pr < 0)
  {
    return -1;
  }

  const ssize_t n = ::read(socket_fd_, &refOut, sizeof(refOut));

  if (n < 0)
  {
    return -1;
  }

  if (n != static_cast<ssize_t>(sizeof(refOut)))
  {
    errno = EIO;
    return -1;
  }

  return 1;
}

}  // namespace bw
