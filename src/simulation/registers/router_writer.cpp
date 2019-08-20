#include "router_writer.hpp"

void RegisterWriter::add_bytes_to_packet(std::vector<unsigned char> cdata, uint32_t pay_size)
{
    //this->update_packet_size(pay_size);
    this->messageData = cdata;
    //this->writer_callback_REGinbound();
    return;
}

