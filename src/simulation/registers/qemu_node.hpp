#ifndef _QEMU_NODE_HH_
#define _QEMU_NODE_HH_

#include "worker_process.hpp"
#include "router_reader.hpp"
#include "router_writer.hpp"

/*********************************** QEMU NODE ***********************************/

class QemuWorkerProcess : public WorkerProcess {
public:
    QemuWorkerProcess();
    ~QemuWorkerProcess();

    bool getTicked();
    void add_router(RegisterReader *newRouter);
    bool should_router_publish(RegisterReader* router);
    void publish_single_router_data(RegisterReader* router);

    void handle_publish();
    void handle_step_process(); // pack_data_from_registers();
    zmqpp::message handle_tock_message();

    void route_in_packet(std::string msg_name, std::string packet_type, int pay_size, std::string pay_hex);
    void add_writer(RegisterWriter *newWriter);
public:
    std::vector<RegisterReader*> routers;
    std::vector<RegisterWriter*> writers;
};

#endif /* _QEMU_NODE_HH_ */
