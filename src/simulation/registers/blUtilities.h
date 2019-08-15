
#ifndef _BL_UTILITIES_HH_
#define _BL_UTILITIES_HH_

#include <iostream>
#include <cctype> // is*
#include <algorithm>
#include <string>
#include <vector>
#include <iterator>
#include <iomanip>
#include <sstream>
#include <typeinfo>

namespace BL {
    int to_int(int c);

    template<class InputIterator, class OutputIterator>
    int
    unhexlify(InputIterator first, InputIterator last, OutputIterator ascii) {
        while (first != last) {
            int top = to_int(*first++);
            int bot = to_int(*first++);
            if (top == -1 or bot == -1)
                return -1; // error
            *ascii++ = (top << 4) + bot;
        }
        return 0;
    }

    template<typename T>
    std::string int_to_hex(T i) {
        std::stringstream stream;
        stream //<< "0x"
                << std::setfill('0') << std::setw(sizeof(T) / 2)
                << std::hex << i;
        return stream.str();
    }
    
    char* str_to_char(std::string hex_str);

    unsigned reverse_endianness(unsigned x);
    
    uint64_t simple_byteswap(uint64_t base_value, uint32_t byte_count);
    
    uint32_t create_descriptorFSW_mar(uint32_t current_packet_size);
    
    uint32_t create_descriptorFSW_scott(uint32_t current_packet_size);
    
    uint16_t combine_2bytes(uint8_t dataLeft, uint8_t dataRight);
    uint32_t combine_4bytes(uint8_t dataLeft, uint8_t dataMiddleLeft, uint8_t dataMiddleRight, uint8_t dataRight);
    uint32_t add_masked_word_to_value(uint32_t mask, uint32_t word, uint32_t read_val);
    uint32_t apply_mask_to_value(uint32_t mask, uint32_t read_val);
    std::vector<uint32_t> combine_vectors(std::vector<uint32_t> A, std::vector<uint32_t> B);
    std::vector<uint32_t> split_word_into_4(uint32_t word);
    std::vector<uint16_t> split_word_into_2(uint16_t word);
    unsigned short calcCRCFast(void* packet, unsigned short length);
}

#endif /* _BL_UTILITIES_HH_ */
