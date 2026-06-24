/**
 * ============================================================================
 * SerialPort — Pure POSIX serial port implementation
 * ============================================================================
 */

#include "mobile_sdk/serial_port.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace mobile_sdk
{

SerialPort::SerialPort()
    : fd_(-1)
{
}

SerialPort::~SerialPort()
{
    close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(other.fd_), port_(std::move(other.port_))
{
    other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept
{
    if (this != &other)
    {
        close();
        fd_ = other.fd_;
        port_ = std::move(other.port_);
        other.fd_ = -1;
    }
    return *this;
}

unsigned int SerialPort::baudrateToSpeed(int baudrate)
{
    switch (baudrate)
    {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B19200;
    }
}

bool SerialPort::open(const std::string& port, int baudrate)
{
    close();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        fprintf(stderr, "[SerialPort] Failed to open %s: %s\n",
                port.c_str(), strerror(errno));
        return false;
    }

    // Clear O_NONBLOCK after open (we use non-blocking read via available() check)
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0)
    {
        fprintf(stderr, "[SerialPort] tcgetattr error: %s\n", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t spd = baudrateToSpeed(baudrate);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // 8N1, no flow control
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CLOCAL | CREAD);

    // Raw mode
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;

    // Read timeout: 100ms (VTIME=1 * 100ms), minimum 0 bytes
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcflush(fd_, TCIOFLUSH);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        fprintf(stderr, "[SerialPort] tcsetattr error: %s\n", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    port_ = port;
    printf("[SerialPort] Opened %s @ %d baud\n", port.c_str(), baudrate);
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const
{
    return fd_ >= 0;
}

size_t SerialPort::available() const
{
    if (fd_ < 0) return 0;
    int bytes = 0;
    if (ioctl(fd_, FIONREAD, &bytes) < 0) return 0;
    return static_cast<size_t>(bytes);
}

ssize_t SerialPort::read(uint8_t* buffer, size_t size) const
{
    if (fd_ < 0) return -1;
    return ::read(fd_, buffer, size);
}

ssize_t SerialPort::write(const uint8_t* buffer, size_t size) const
{
    if (fd_ < 0) return -1;
    return ::write(fd_, buffer, size);
}

void SerialPort::flush() const
{
    if (fd_ >= 0)
    {
        tcdrain(fd_);
    }
}

}  // namespace mobile_sdk
