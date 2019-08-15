
#include <iostream>
#include <cctype> // is*
#include <algorithm>
#include <string>
#include <vector>
#include <iterator>
#include <iomanip>
#include <sstream>
#include <typeinfo>
#include "blUtilities.h"
#include <cstring>

namespace BL {
    int to_int(int c) {
        if (not isxdigit(c)) return -1; // error: non-hexadecimal digit found
        if (isdigit(c)) return c - '0';
        if (isupper(c)) c = tolower(c);
        return c - 'a' + 10;
    }

    char * str_to_char(std::string hex_str) {
        char *hex_char = new char[hex_str.size() + 1];
        hex_char[hex_str.size()] = 0;
        memcpy(hex_char, hex_str.c_str(), hex_str.size());
        return (hex_char);
    }


    unsigned reverse_endianness(unsigned x) {
        x = (x & 0x55555555) << 1 | ((x >> 1) & 0x55555555);
        x = (x & 0x33333333) << 2 | ((x >> 2) & 0x33333333);
        x = (x & 0x0F0F0F0F) << 4 | ((x >> 4) & 0x0F0F0F0F);
        x = (x << 24) | ((x & 0xFF00) << 8) | ((x >> 8) & 0xFF00) | (x >> 24);
        return x;
    }

    uint64_t simple_byteswap(uint64_t base_value, uint32_t byte_count)
    {
        uint64_t out_value = 0;
        uint8_t *base_ptr = reinterpret_cast<uint8_t *> (&base_value);
        uint8_t *out_ptr = reinterpret_cast<uint8_t*> (&out_value);
        out_ptr += byte_count - 1;
        for(int i=0; i<byte_count; i++)
        {
            *out_ptr = *base_ptr;
            out_ptr--;
            base_ptr++;
        }
        return(out_value);
    }

    uint32_t create_descriptorFSW_mar(uint32_t current_packet_size)
    {
        uint32_t descriptor_word = 1 << 23;
        uint32_t size_word = simple_byteswap(current_packet_size, sizeof(uint32_t));
        descriptor_word |= size_word;
        uint32_t descriptor_FSWinbound = simple_byteswap(descriptor_word, sizeof(uint32_t));
        return(descriptor_FSWinbound);
    }
    
    uint32_t create_descriptorFSW_scott(uint32_t current_packet_size)
    {
        uint16_t local_descriptor = 1 << 15;
        uint16_t mask = 0x07FF;
        uint16_t packet_size = current_packet_size;
        local_descriptor |= (packet_size&mask);
        uint32_t descriptor_fsw = reverse_endianness(local_descriptor);
        return(descriptor_fsw);
    }
    
    uint16_t combine_2bytes(uint8_t dataLeft, uint8_t dataRight)
    {
        uint16_t data16 = dataLeft << 8;
        data16 |= dataRight;
        return(data16);
    }
    uint32_t combine_4bytes(uint8_t dataLeft, uint8_t dataMiddleLeft, uint8_t dataMiddleRight, uint8_t dataRight)
    {
        uint32_t data32 = dataRight | (dataMiddleRight << 8) | (dataMiddleLeft << 16) | (dataLeft << 24);
        return(data32);
    }
    uint32_t add_masked_word_to_value(uint32_t mask, uint32_t word, uint32_t read_val)
    {
        uint32_t inv_mask = ~mask;
        read_val = read_val & inv_mask;
        read_val = read_val | (word & mask);
        return(read_val);
    }

    uint32_t apply_mask_to_value(uint32_t mask, uint32_t read_val)
    {
        uint32_t masked_val = read_val & mask;
        return(masked_val);
    }
    std::vector<uint32_t> combine_vectors(std::vector<uint32_t> A, std::vector<uint32_t> B)
    {
        std::vector<uint32_t> AB;
        AB.reserve( A.size() + B.size() ); // preallocate memory
        AB.insert( AB.end(), A.begin(), A.end() );
        AB.insert( AB.end(), B.begin(), B.end() );
        return(AB);
    }
    std::vector<uint32_t> split_word_into_4(uint32_t word)
    {
        uint8_t byte_array[4];
        byte_array[0] = word >> 24;
        byte_array[1] = word >> 16;
        byte_array[2] = word >> 8;
        byte_array[3] = word;
        std::vector<uint32_t> word_array = {byte_array[0],  byte_array[1], byte_array[2], byte_array[3]};
        return(word_array);
    }

    std::vector<uint16_t> split_word_into_2(uint16_t word)
    {
        uint8_t byte_array[2];
        byte_array[0] = word & 0xff;
        byte_array[1] = (word >> 8);
        std::vector<uint16_t> word_array = {byte_array[0],  byte_array[1]};
        return(word_array);
    }

    unsigned short calcCRCFast(void* packet, unsigned short length)
    {
        // Definition of the 8 bit lookup table. This declared static, such that
        // it does not need redefinition at each call.
        static unsigned short CRCTable[256];
        // To flag, that the lookup table has not been initialized yet.
        static unsigned char first=1;
        // Counters and temporary storages
        int i, j, bit;
        // Used to hold the shift register.
        unsigned short reg;
        // The topmost 16 bits
        unsigned short top;
        // The CRC key (or generator polynomial)
        unsigned short key;
        // If this is the first call, initialize the lookup table
        if (first)
        {
            // The key is set to X^12 + X^5 + 1 (X^16 is implicit)
            key = 0x1021;
            for (i=0; i<256; i++)
            {
                // Loop through the 8 bit of the current entry
                reg = i << 8;
                for (j=0; j<8; j++)
                {
                    bit = reg & 0x8000;
                    reg <<= 1;
                    if (bit)
                    {
                        reg ^= key;
                    }
                }
                // The lookup table is updated with the determined divisor
                CRCTable[i] = reg;
            }
            // No need to initialization again
            first=0;
        }
        // The shift register is preset to 0xffff
        reg = 0x0ffff;
        for (i=0; i<length; i++)
        {
            // Insert the packet byte in the topmost word
            top = reg >> 8;
            top ^= ((unsigned char*)packet)[i];
            // Update the shift register with the selected entry from
            // the lookup table
            reg = (reg << 8) ^ CRCTable[top];
        }
        return reg;
    }
}

