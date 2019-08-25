#ifndef _ROUTER_WRITER_HH_
#define _ROUTER_WRITER_HH_
#include "router.hpp"
#include <vector>
#include <set>

class RegisterWriter: public QemuRouter
{
public:
    RegisterWriter(){};
    RegisterWriter(std::string msg_name, int64_t msg_id, uint64_t msg_size):
    QemuRouter(msg_name, msg_id, msg_size)
    {
        this->messageData.clear();
        this->writer_callback_REGinbound = NULL;
        this->writer_callback_FSWinbound = NULL;
    };
    ~RegisterWriter(){};
    std::function<void()> writer_callback_REGinbound;
    std::function<unsigned int()> writer_callback_FSWinbound;
    
public:
    void add_bytes_to_packet(std::vector<unsigned char> cdata, uint32_t pay_size);
    
public:
    std::vector<unsigned char> messageData;
    void serialize_simpleNav();
    void serialize_bytes();
};

#endif /* _ROUTER_WRITER_HH_ */
