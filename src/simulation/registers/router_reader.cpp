#include "router_reader.hpp"
#include <iostream>



void RegisterReader::add_packet_to_router()
{
    std::cout << "RegisterReader::add_packet_to_router()" << std::endl;
    /*
     uint8_t *current_buffer = new uint8_t[this->current_packet_size];
     QemuSingleton::GetInstance()->read_message_from_buffer(this->buffer_idx, this->base_packet_address,
     this->current_packet_size, &current_buffer[0]);
     std::vector<unsigned char> current_payload(current_buffer, current_buffer + this->current_packet_size);
     DataExchangeObject local_object = DataExchangeObject(this->current_packet_size, current_payload);
     this->dataPackets.push_back(local_object);
     delete [] current_buffer;
     */
    return;
}

bool RegisterReader::packets_waiting()
{
    return(this->dataPackets.size() > 0);
}

void RegisterReader::create_packet_from_FSWword()
{
    /*
     QemuSingleton::GetInstance()->write_message_to_buffer(this->get_buffer_index(), this->access_address,sizeof(this->current_data_word), reinterpret_cast<uint8_t*> (&(this->current_data_word)));
     this->update_packet_size(sizeof(this->current_data_word));
     */
    std::cout << "RegisterReader::create_packet_from_FSWword()" << std::endl;
    this->add_packet_to_router();
}
