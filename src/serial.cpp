#include "serial.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <stdexcept>

usb::usb(const std::string& device, speed_t baud) :
    fd{open(device.c_str(), O_RDWR | O_NOCTTY)}
{
    if (0 > fd)
    {
        throw std::runtime_error("Cannot initialize serial interface");
    }

    disableFlowControl();
    configureSerial(baud);
    flushBuffer();
}

usb::~usb()
{
    close(fd);
}

size_t usb::read(std::vector<uint8_t>& vect, uint32_t size, uint32_t timeoutMs)
{
    size_t bytesToRead{size};
    while (bytesToRead)
    {
        if (pollfd pollFd{fd, POLLIN, 0}; 0 < poll(&pollFd, 1, timeoutMs))
        {
            if (pollFd.revents & POLLIN)
            {
                std::vector<uint8_t> data(bytesToRead);
                auto bytes = ::read(fd, &data[0], bytesToRead);
                vect.insert(vect.end(), data.begin(), data.begin() + bytes);
                bytesToRead -= bytes;
            }
        }
        else
        {
            break; // timeout or error occured, abort
        }
    }
    return size - bytesToRead;
}

size_t usb::read(std::vector<uint8_t>& vect, uint32_t size)
{
    return read(vect, size, 100);
}

size_t usb::write(const std::vector<uint8_t>& vect)
{
    return ::write(fd, &vect[0], vect.size());
}

inline void usb::flushBuffer()
{
    ioctl(fd, TCFLSH, TCIOFLUSH);
}

inline void usb::disableFlowControl()
{
    static const uint8_t bitsToClear{TIOCM_DTR | TIOCM_RTS};
    ioctl(fd, TIOCMBIC, &bitsToClear);
}

inline void usb::configureSerial(speed_t baud)
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
