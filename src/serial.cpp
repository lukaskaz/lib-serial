#include "serial.hpp"

#include <iostream>

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
                              debug_t isdebug)
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
