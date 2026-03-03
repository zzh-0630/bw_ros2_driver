#include "bw_ros2_driver/serial_port.hpp"

// System headers
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

// Standard headers
#include <cerrno>
#include <cstdio>
#include <cstring>

namespace bw
{
//---------------------------------------------------------------------//
//- Constructors / destructor                                          -//
//---------------------------------------------------------------------//
SerialPort::SerialPort(const std::string& port, int baud) :
  port_(port),
  baud_(baud),
  fd_(-1)
{
}

SerialPort::~SerialPort()
{
  closeSerial();
}

//---------------------------------------------------------------------//
//- Private methods                                                    -//
//---------------------------------------------------------------------//
speed_t SerialPort::mapBaud(int baud)
{
  switch (baud)
  {
    case 9600:
      return B9600;

    case 19200:
      return B19200;

    case 38400:
      return B38400;

    case 57600:
      return B57600;

    case 115200:
      return B115200;

#ifdef B230400
    case 230400:
      return B230400;
#endif

#ifdef B460800
    case 460800:
      return B460800;
#endif

#ifdef B500000
    case 500000:
      return B500000;
#endif

#ifdef B576000
    case 576000:
      return B576000;
#endif

#ifdef B921600
    case 921600:
      return B921600;
#endif

#ifdef B1000000
    case 1000000:
      return B1000000;
#endif

#ifdef B1152000
    case 1152000:
      return B1152000;
#endif

#ifdef B1500000
    case 1500000:
      return B1500000;
#endif

#ifdef B2000000
    case 2000000:
      return B2000000;
#endif

#ifdef B3000000
    case 3000000:
      return B3000000;
#endif

    default:
      return B9600;
  }
}

bool SerialPort::configureTermios()
{
  termios tio{};

  if (tcgetattr(fd_, &tio) != 0)
  {
    std::perror("tcgetattr");
    return false;
  }

  //-------------------------------------------------------------------//
  // Raw mode                                                          //
  //-------------------------------------------------------------------//
  cfmakeraw(&tio);

  //-------------------------------------------------------------------//
  // Baud rate                                                         //
  //-------------------------------------------------------------------//
  const speed_t sp = mapBaud(baud_);
  cfsetispeed(&tio, sp);
  cfsetospeed(&tio, sp);

  //-------------------------------------------------------------------//
  // 8N1, ignore modem control                                          //
  //-------------------------------------------------------------------//
  tio.c_cflag |= (CLOCAL | CREAD | CS8);
  tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);

  //-------------------------------------------------------------------//
  // Read behavior (blocking mode):                                     //
  // - VMIN = 0: return as soon as any data is available                //
  // - VTIME = 2: timeout after 0.2 seconds if no data                  //
  //-------------------------------------------------------------------//
  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 2;

  if (tcsetattr(fd_, TCSANOW, &tio) != 0)
  {
    std::perror("tcsetattr");
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

//---------------------------------------------------------------------//
//- Public methods                                                     -//
//---------------------------------------------------------------------//
bool SerialPort::openSerial()
{
  closeSerial();

  // O_NOCTTY: do not make this device the controlling terminal.
  // O_NONBLOCK: avoid blocking on open for some devices.
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (fd_ < 0)
  {
    std::perror("open");
    return false;
  }

  if (!configureTermios())
  {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Clear O_NONBLOCK so VTIME/VMIN work as intended.
  const int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags >= 0)
  {
    (void)::fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
  }

  return true;
}

void SerialPort::closeSerial()
{
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::reOpenSerial()
{
  closeSerial();
  return openSerial();
}

bool SerialPort::setBaud(int baud)
{
  baud_ = baud;

  if (fd_ < 0)
  {
    return true;
  }

  return configureTermios();
}

ssize_t SerialPort::readSome(uint8_t* p_buffer, size_t max)
{
  if (fd_ < 0)
  {
    return -1;
  }

  const ssize_t n = ::read(fd_, p_buffer, max);

  if (n < 0 && errno == EAGAIN)
  {
    return 0;
  }

  return n;
}

ssize_t SerialPort::writeAll(const uint8_t* p_data, size_t len)
{
  if (fd_ < 0)
  {
    return -1;
  }

  size_t written = 0;

  while (written < len)
  {
    const ssize_t n = ::write(fd_, p_data + written, len - written);

    if (n < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }

      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        pollfd pfd{};
        pfd.fd = fd_;
        pfd.events = POLLOUT;

        // Wait up to 100 ms for the fd to become writable.
        (void)::poll(&pfd, 1, 100);
        continue;
      }

      return -1;
    }

    written += static_cast<size_t>(n);
  }

  return static_cast<ssize_t>(written);
}

}  // namespace bw
