#include "serial.hpp"

#include <iostream>

inline void serial::showserialtraces(std::string_view name,
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
