#include <iostream>
#include "qemu_singleton.hpp"


QemuSingleton* QemuSingleton::TheInstance = NULL;

QemuSingleton::QemuSingleton()
{
    this->QemuWorker = NULL;
    this->conn_string = "None";
    this->conn_present = false;
    this->firstTockCommand = true;
}

QemuSingleton::~QemuSingleton()
{
    this->QemuWorker = NULL;
}

QemuSingleton* QemuSingleton::GetInstance()
{
    if (TheInstance == NULL)
    {
        TheInstance = new QemuSingleton();
    }
    return(TheInstance);
}

void QemuSingleton::set_connection_string(std::string conn_string_in)
{
    this->conn_string = conn_string_in;
    this->conn_present=true;
}

void QemuSingleton::create_worker_process(std::string conn_string_in)
{
    this->QemuWorker = new QemuWorkerProcess();
    this->set_connection_string(conn_string_in);
    if(this->conn_present) {
        this->QemuWorker->set_master_address(conn_string);
        this->QemuWorker->start_poller();
    }
}

void QemuSingleton::poll_worker_process()
{
    if(!this->conn_present)
    {
        return;
    }
    do{
        std::cout << "Polling for data..."<<std::endl;
        this->QemuWorker->reactor->poll();
    }while(!this->QemuWorker->getTicked() && !this->QemuWorker->interrupted_node);
}

void QemuSingleton::add_snorkel_reader(RegisterReader *new_reader)
{
    this->QemuWorker->add_router(new_reader);
}

void QemuSingleton::add_snorkel_writer(RegisterWriter *new_writer)
{
    this->QemuWorker->add_writer(new_writer);
}

void QemuSingleton::execute_tock()
{
    if(!this->firstTockCommand && this->conn_present){
        zmqpp::message tock_msg = this->QemuWorker->tock_node_callback();
        this->QemuWorker->rep_socket->send(tock_msg);
        std::cout << "QemuSingleton::execute_tock(). TOCK!" << std::endl;
    }
    this->firstTockCommand = false;
}


void QemuSingleton::update_registers_FSWoutbound(int64_t MessageID, uint8_t *MsgPayload)
{
    // Check if Reader exists
    RegisterReader* local_reader = find_reader_by_msgID(MessageID);
    if(local_reader!= NULL)
    {
        std::cout << "Updating local_reader: " << local_reader->get_router_name()<< std::endl;
        local_reader->create_packet_from_FSWdata(MsgPayload);
    }
}

RegisterReader* QemuSingleton::find_reader_by_msgID(int64_t msgID)
{
     std::vector<RegisterReader*>::iterator it;
    for(it = this->QemuWorker->routers.begin(); it!=this->QemuWorker->routers.end(); it++){
        if ( (*it)->msg_id== msgID) {
            std::cout << "Device Reader found: "<< (*it)->get_router_name() <<  std::endl;
            return(*it);
        }
     }
    return(NULL);
}

RegisterWriter* QemuSingleton::find_writer_by_msgID(int64_t msgID)
{
     std::vector<RegisterWriter*>::iterator it;
    for(it = this->QemuWorker->writers.begin(); it!=this->QemuWorker->writers.end(); it++){
        if ( (*it)->msg_id == msgID ){
            std::cout << "Device Writer found: "<< (*it)->get_router_name() <<  std::endl;
            return(*it);
        }
     }
    return(NULL);
}





