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
public:
    bool packets_waiting();
    void add_packet_to_router();
    void create_packet_from_FSWword();
    std::function<void()> reader_callback_FSWoutbound;
};

#endif /* _ROUTER_READER_HH_ */
