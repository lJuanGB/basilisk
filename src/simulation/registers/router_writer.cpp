#include "router_writer.hpp"
#include <iostream>

void RegisterWriter::add_bytes_to_packet(std::vector<unsigned char> cdata, uint32_t pay_size)
{
    std::cout << "Incoming bytes from: "<< this->get_router_name() << std::endl;
    //this->update_packet_size(pay_size);
    this->messageData = cdata;
    this->data_fresh = true;
    return;
}
