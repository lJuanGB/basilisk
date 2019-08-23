#ifndef _QEMU_SINGLETON_HH_
#define _QEMU_SINGLETON_HH_
#include "qemu_node.hpp"
#include "router_reader.hpp"
#include "router_writer.hpp"

class QemuSingleton{
public:
    static QemuSingleton* GetInstance();
    void create_worker_process(std::string conn_string_in);
    void set_connection_string(std::string conn_string_in);
    void poll_worker_process();
    
    void add_snorkel_reader(RegisterReader *new_reader);
    void add_snorkel_writer(RegisterWriter *new_writer);
    
    void update_registers_FSWoutbound(int64_t MessageID, uint8_t *MsgPayload);
    
    void execute_tock();
    
    void setQemuTime(uint64_t clockTime) {this->qemuTime = clockTime;}
    uint64_t getQemuTime() {return this->qemuTime;}

    
private:
    QemuSingleton();
    ~QemuSingleton();
    QemuSingleton(QemuSingleton const &) {};
    QemuSingleton& operator = (QemuSingleton const &){ return(*this);};
private:
    static QemuSingleton *TheInstance;
    QemuWorkerProcess *QemuWorker;
    std::string conn_string;
    bool conn_present;
    bool firstTockCommand;
    uint64_t qemuTime;
    RegisterReader* find_reader_by_msgID(int64_t msgID);
    RegisterWriter* find_writer_by_msgID(int64_t msgID);
};

#endif /* _QEMU_SINGLETON_HH_ */
