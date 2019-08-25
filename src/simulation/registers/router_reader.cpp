#include "router_reader.hpp"
#include <iostream>

void RegisterReader::create_packet_from_FSWword()
{
    //uint8_t *current_buffer = reinterpret_cast<uint8_t*>(this->current_data_word);
    std::vector<unsigned char> current_payload(current_buffer, current_buffer + this->msg_size);
    DataExchangeObject local_object = DataExchangeObject(this->msg_size, current_payload);
    this->dataPackets.push_back(local_object);
    //delete [] current_buffer;
    return;
}

bool RegisterReader::packets_waiting()
{
    return(this->dataPackets.size() > 0);
}

