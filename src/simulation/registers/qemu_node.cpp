#include "qemu_node.hpp"
#include "worker_process.hpp"
#include "blUtilities.h"

/*********************************** QEMU WP ***********************************/
QemuWorkerProcess::QemuWorkerProcess()
{
    std::cout << "Hello, Qemu WP here! "<< std::endl;
    this->step_node_callback = std::bind(&QemuWorkerProcess::handle_step_process, this);
    this->tock_node_callback = std::bind(&QemuWorkerProcess::handle_tock_message, this);
    this->publish_node_callback =std::bind(&QemuWorkerProcess::handle_publish, this);
    this->sub_callback = std::bind(&WorkerProcess::handle_sub_interrupt_callback, this);
    this->rep_callback = std::bind(&WorkerProcess::handle_rep_interrupt_callback, this);
}

QemuWorkerProcess::~QemuWorkerProcess()
{
    return;
}

void QemuWorkerProcess::route_in_packet(std::string msg_name, std::string packet_type, int pay_size, std::string pay_hex)
{
    /* Unhexlify payload */
    char* hex_ptr = BL::str_to_char(pay_hex);
    size_t len = pay_size*2;
    unsigned char ascii[pay_size];
    assert(BL::unhexlify(hex_ptr, hex_ptr+len, ascii) >= 0);

    /* Convert from unsigned char pointer to vector of unsigned char */
    std::vector<unsigned char> cdata(pay_size);
    int i = 0;
    for (unsigned int x : ascii)
    {
        cdata[i] = (unsigned char) x;
        i ++;
    }
    /* Do something with the payload */
    std::vector<RegisterWriter*>::iterator it;
    for(it = this->writers.begin(); it!=this->writers.end(); it++)
    {
        if (msg_name == (*it)->get_router_name())
        {
            (*it)->add_bytes_to_packet(cdata, pay_size);
            std::cout << "Writer "<< msg_name << " received bytes to route into the registers." << std::endl;
        }
    }
    return;
}

bool QemuWorkerProcess::getTicked()
{
    return(this->tickReceived);
}

void QemuWorkerProcess::add_router(RegisterReader *newRouter)
{
    this->routers.push_back(newRouter);
    this->registers_all.insert(newRouter->get_router_name());
}

void QemuWorkerProcess::add_writer(RegisterWriter *newWriter)
{
    this->writers.push_back(newWriter);
    this->subscriptions.insert(newWriter->get_router_name());
}

void QemuWorkerProcess::handle_step_process()
{
    std::cout << "Qemu WP: step_process(). Nothing. "<< std::endl;
}

zmqpp::message QemuWorkerProcess::handle_tock_message()
{
    zmqpp::message msg = std::string(BL_TOCK);
    std::vector<RegisterReader*>::iterator it;
    std::string register_name;
    int packets_count = 0;
    for(it = this->routers.begin(); it!=this->routers.end(); it++)
    {
        if (this->should_router_publish((*it))){
            register_name =(*it)->get_router_name();
            packets_count = (*it)->dataPackets.size();
 //           std::cout << "Router "<< register_name << " has contents to publish: # "<< packets_count << std::endl;
            for (int i = 0; i < packets_count; i++)
            {
                msg.add((*it)->get_router_name());
            }
        }
    }
    return (msg);
}

void QemuWorkerProcess::handle_publish()
{
    std::cout << "QemuWorkerProcess::handle_publish()\n " << std::endl;
    std::vector<RegisterReader*>::iterator it;
    for(it = this->routers.begin(); it!=this->routers.end(); it++)
    {
        if (this->should_router_publish((*it)))
        {
            this->publish_single_router_data((*it));
        }
    }
    return;
}

bool QemuWorkerProcess::should_router_publish(RegisterReader* router)
{
    bool publish = false;
    std::string register_name = router->get_router_name();
    if (this->publications.find(register_name) != this->publications.end())
    {
        publish = router->packets_waiting();
    }
    return publish;
}

void QemuWorkerProcess::publish_single_router_data(RegisterReader* router)
{
    std::string register_name = router->get_router_name();
//    std::cout << "Router "<< register_name  << " has contents to publish." << std::endl;
    std::vector<DataExchangeObject>::iterator it;
    for(it = router->dataPackets.begin(); it!=router->dataPackets.end(); it++)
    {
        zmqpp::message local_msg;
//        std::cout << "pay_size = " << (it)->payload_size << std::endl;
        local_msg.add(register_name);
        local_msg.add((it)->packet_type);
        local_msg.add(std::to_string((it)->payload_size));
        local_msg.add((it)->marshall_payload());
        this->pub_socket->send(local_msg);
    }
//    std::cout << "Packets published. Clearing stack."<< std::endl;
    router->dataPackets.clear();
    return;
}
