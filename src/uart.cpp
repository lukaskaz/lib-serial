#include "serial.hpp"

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdexcept>

uart::uart(const std::string& device, speed_t baud) :
    fd{open(device.c_str(), O_RDWR | O_NOCTTY)}
{
    if (0 > fd)
    {
        throw std::runtime_error("Cannot initialize serial interface");
    }

    configure(baud);
    flushBuffer();
}

uart::~uart()
{
    close(fd);
}

size_t uart::read(std::vector<uint8_t>& vect, ssize_t size, uint32_t timeoutMs,
                  debug_t isdebug)
{
    auto bytesToRead{size};
    auto epollfd = epoll_create1(0);
    if (epollfd >= 0)
    {
        epoll_event event{.events = EPOLLIN, .data = {.fd = fd}}, revent{};
        epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &event);

        while (bytesToRead && 0 < epoll_wait(epollfd, &revent, 1, timeoutMs))
        {
            if (revent.events & EPOLLIN)
            {
                std::vector<uint8_t> data(bytesToRead);
                auto bytes = ::read(fd, &data[0], bytesToRead);
                vect.insert(vect.end(), data.begin(), data.begin() + bytes);
                bytesToRead -= bytes;
            }
        }
        close(epollfd);
    }
    showserialtraces("read", vect, isdebug);
    return size - bytesToRead;
}

size_t uart::read(std::vector<uint8_t>& vect, ssize_t size, debug_t isdebug)
{
    return read(vect, size, 100, isdebug);
}

size_t uart::write(const std::vector<uint8_t>& vect, debug_t isdebug)
{
    showserialtraces("write", vect, isdebug);
    return ::write(fd, &vect[0], vect.size());
}

void uart::flushBuffer()
{
    ioctl(fd, TCFLSH, TCIOFLUSH);
}

inline void uart::configure(speed_t baud)
{
    termios options{};
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    cfmakeraw(&options);
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);
    fcntl(fd, F_SETFL, FNDELAY);
}
