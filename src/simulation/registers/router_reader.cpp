#include "router_reader.hpp"
#include <iostream>

void RegisterReader::create_packet_from_FSWdata(uint8_t* data_buffer)
{
    std::vector<unsigned char> current_payload(data_buffer, data_buffer + this->msg_size);
    DataExchangeObject local_object = DataExchangeObject(this->msg_size, current_payload);
    this->dataPackets.clear(); // Only one packet sent out
    this->dataPackets.push_back(local_object);
    return;
}

bool RegisterReader::packets_waiting()
{
    return(this->dataPackets.size() > 0);
}

