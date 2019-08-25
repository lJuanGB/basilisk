#include "router.hpp"
#include <iostream>

/***********************************  ROUTER ***********************************/
QemuRouter::QemuRouter(std::string msg_name, int64_t msg_id, uint64_t msg_size)
{
    this->msg_name = msg_name;
    this->msg_id = msg_id;
    this->msg_size = msg_size;
    return;
}
QemuRouter::~QemuRouter()
{
    return;
}
