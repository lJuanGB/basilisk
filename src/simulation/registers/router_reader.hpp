#ifndef _ROUTER_READER_HH_
#define _ROUTER_READER_HH_
#include "router.hpp"
#include "worker_process.hpp"

class RegisterReader: public QemuRouter
{
public:
    RegisterReader(){};
    RegisterReader(std::string msg_name, int64_t msg_id, uint64_t msg_size):
    QemuRouter(msg_name, msg_id, msg_size)
    {
        this->dataPackets.clear();
        this->reader_callback_FSWoutbound = NULL;
    };
    ~RegisterReader(){};
    std::vector<DataExchangeObject> dataPackets;
    //void* current_data_word;
    uint8_t* current_buffer;
    std::function<void()> reader_callback_FSWoutbound;
public:
    bool packets_waiting();
    void create_packet_from_FSWword();
};

#endif /* _ROUTER_READER_HH_ */
