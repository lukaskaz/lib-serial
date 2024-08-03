#include "serial.hpp"

#include <iostream>
#include <stdexcept>
#include <unordered_map>

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
    static const std::unordered_map<speed_t, std::string> baudtoname{
        {B9600, "9600"},   {B19200, "19200"},   {B38400, "38400"},
        {B57600, "57600"}, {B115200, "115200"}, {B460800, "460800"}};

    if (baudtoname.contains(baud))
    {
        return baudtoname.at(baud);
    }
    throw std::runtime_error("Cannot find baud to name mapping");
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
