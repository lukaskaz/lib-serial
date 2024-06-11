#include "serial.hpp"

#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <iostream>
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
                  bool debug = false)
{
    showserialtraces("read", vect, debug);

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
    return size - bytesToRead;
}

size_t uart::read(std::vector<uint8_t>& vect, ssize_t size, bool debug = false)
{
    return read(vect, size, 100, debug);
}

size_t uart::write(const std::vector<uint8_t>& vect, bool debug = false)
{
    showserialtraces("write", vect, debug);
    return ::write(fd, &vect[0], vect.size());
}

void uart::flushBuffer()
{
    ioctl(fd, TCFLSH, TCIOFLUSH);
}

inline void usb::showserialtraces(std::string_view name,
                                  const std::vector<uint8_t>& packet,
                                  bool debug)
{
    if (debug)
    {
        uint8_t maxcolumn = 4, column{maxcolumn};
        std::cout << std::dec << "\n> Name: " << name
                  << ", size: " << packet.size() << std::hex << "\n";
        for (const auto& byte : packet)
        {
            std::cout << "0x" << (uint32_t)byte << " ";
            if (!--column)
            {
                column = maxcolumn;
                std::cout << "\n";
            }
        }
        std::cout << std::dec;
    }
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
