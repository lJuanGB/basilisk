#include "router_writer.hpp"
#include "simFswInterfaceMessages/navAttIntMsg.h"

void RegisterWriter::add_bytes_to_packet(std::vector<unsigned char> cdata, uint32_t pay_size)
{
    //this->update_packet_size(pay_size);
    this->messageData = cdata;
    //this->writer_callback_REGinbound();
    return;
}

void RegisterWriter::serialize_simpleNav()
{
    NavAttIntMsg nav_buffer;
    memset(&nav_buffer, 0x0, sizeof(NavAttIntMsg));
    memcpy(&nav_buffer, this->messageData.data(), sizeof(NavAttIntMsg));
    //SystemMessaging::GetInstance()->WriteMessage(this->get_msgID(), Clock, this->get_msg_size(),  reinterpret_cast<uint8_t*> (&nav_buffer), this->moduleID);
    return;
}
void RegisterWriter::serialize_bytes()
{
    return;
}

//void RegisterWriter::serialize_css_bytes()
//{
//    EmmCssFswMsg cssLSB_bufferIn;
//    memset(&cssLSB_bufferIn, 0x0, sizeof(EmmCssFswMsg));
//    memcpy(&cssLSB_bufferIn, this->messageData.data(), sizeof(EmmCssFswMsg));
//}

