#include "serial.hpp"

#include <fcntl.h>
#include <sys/epoll.h>
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

    serial::device = device;
    serial::baudname = getbaudname(baud);
    disableFlowControl();
    configure(baud);
    flushBuffer();
}

usb::~usb()
{
    close(fd);
}

size_t usb::read(std::vector<uint8_t>& vect, ssize_t size, uint32_t timeoutMs,
                 debug_t isdebug)
{
    static const auto readdata = [this](std::vector<uint8_t>& out,
                                        ssize_t bytesToRead) {
        auto bytesTotal{bytesToRead};
        while (bytesToRead)
        {
            std::vector<uint8_t> data(bytesToRead);
            auto bytes = ::read(fd, &data[0], bytesToRead);
            if (bytes > 0)
            {
                out.insert(out.end(), data.begin(), data.begin() + bytes);
                bytesToRead -= bytes;
            }
            else
            {
                throw std::runtime_error("Cannot read data");
            }
        }
        return bytesTotal - bytesToRead;
    };
    auto bytesToRead{size}, bytesRead{ssize_t{}};
    auto bytesAvailable = bytesInBuffer();

    if (bytesToRead <= bytesAvailable)
    {
        bytesRead = readdata(vect, bytesToRead);
    }
    else
    {
        if (bytesToRead > UINT8_MAX)
        {
            throw std::runtime_error("Size of read bytes too big");
        }

        termios tm{};
        tcgetattr(fd, &tm);
        tm.c_cc[VTIME] = 0;
        tm.c_cc[VMIN] = static_cast<uint8_t>(bytesToRead);
        tcsetattr(fd, TCSANOW, &tm);

        auto epollfd = epoll_create1(0);
        if (epollfd >= 0)
        {
            epoll_event event{.events = EPOLLIN, .data = {.fd = fd}}, revent{};
            epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &event);

            int32_t ret{};
            while ((ret = epoll_wait(epollfd, &revent, 1, timeoutMs)) < 0)
                ;
            if (ret == 0)
            {
                bytesAvailable = bytesInBuffer();
                bytesRead = readdata(vect, bytesAvailable);
            }
            else
            {
                if (revent.events & EPOLLIN)
                {
                    bytesRead = readdata(vect, bytesToRead);
                }
            }
            close(epollfd);
        }
    }
    showserialtraces("read", vect, isdebug);
    return bytesRead;
}

size_t usb::read(std::vector<uint8_t>& vect, ssize_t size, debug_t isdebug)
{
    return read(vect, size, 100, isdebug);
}

size_t usb::write(const std::vector<uint8_t>& vect, debug_t isdebug)
{
    showserialtraces("write", vect, isdebug);
    return ::write(fd, &vect[0], vect.size());
}

void usb::flushBuffer()
{
    ioctl(fd, TCFLSH, TCIOFLUSH);
}

inline ssize_t usb::bytesInBuffer()
{
    ssize_t bytes{};
    if (auto ret = ioctl(fd, FIONREAD, &bytes); ret < 0)
    {
        throw std::runtime_error("Cannot check data amount in buffer!");
    }
    return bytes;
}

inline void usb::disableFlowControl()
{
    static const uint8_t bitsToClear{TIOCM_DTR | TIOCM_RTS};
    ioctl(fd, TIOCMBIC, &bitsToClear);
}

inline void usb::configure(speed_t baud)
{
    termios options{};
    tcgetattr(fd, &options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    cfmakeraw(&options);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);
    fcntl(fd, F_SETFL, FNDELAY);
}
