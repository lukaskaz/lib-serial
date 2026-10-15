#include "serial.hpp"

#define termios kernel_termios
#include <asm/termbits.h>
#undef termios
#include <asm/ioctls.h>
#include <sys/ioctl.h>

#include <iostream>
#include <stdexcept>
#include <unordered_map>

namespace
{
const std::unordered_map<speed_t, std::string>& baudtoname()
{
    static const std::unordered_map<speed_t, std::string> names{
        {B9600, "9600"},     {B19200, "19200"},   {B38400, "38400"},
        {B57600, "57600"},   {B115200, "115200"}, {B460800, "460800"}};
    return names;
}
}

std::string serial::getdevice() const
{
    return device;
}

std::string serial::getbaud() const
{
    return baudname;
}

std::string serial::getbaudname(speed_t baud) const
{
    if (const auto iter = baudtoname().find(baud); iter != baudtoname().end())
    {
        return iter->second;
    }
    return std::to_string(baud);
}

void serial::configureBaud(int32_t fd, termios& options, speed_t baud)
{
    const bool customBaud = !baudtoname().contains(baud);
    const auto termiosBaud = customBaud ? B38400 : baud;
    cfsetispeed(&options, termiosBaud);
    cfsetospeed(&options, termiosBaud);
    tcsetattr(fd, TCSANOW, &options);
    if (customBaud)
    {
        setCustomBaud(fd, baud);
    }
}

void serial::setCustomBaud(int32_t fd, speed_t baud)
{
    termios2 options{};
    if (ioctl(fd, TCGETS2, &options) < 0)
    {
        throw std::runtime_error("Cannot read serial settings");
    }

    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;
    options.c_ispeed = baud;
    options.c_ospeed = baud;
    if (ioctl(fd, TCSETS2, &options) < 0)
    {
        throw std::runtime_error("Cannot set serial baud rate");
    }
}

size_t serial::read(std::vector<uint8_t>& vect, ssize_t size)
{
    return read(vect, size, debug_t::nodebug);
}
size_t serial::read(std::vector<uint8_t>& vect, ssize_t size, uint32_t timeout)
{
    return read(vect, size, timeout, debug_t::nodebug);
}
size_t serial::write(const std::vector<uint8_t>& vect)
{
    return write(vect, debug_t::nodebug);
}

void serial::showserialtraces(std::string_view name,
                              const std::vector<uint8_t>& packet,
                              debug_t isdebug) const
{
    if (isdebug == debug_t::usedebug)
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
