#include "leon_registers.h"
#include <iostream>
#include "qemu_singleton.hpp"



LeonRegisters::LeonRegisters()
{
    this->outputBufferCount = 2;
    this->rwTorqueMsgName_fswOut = "reactionwheel_torques";
    this->rwSpeedsMsgName_fswIn = "reactionwheel_output_states";
    this->simpleNavAttMsgName_fswIn = "simple_att_nav_output";
    this->BlackLionConnectionInfo = "";
    return;
}


LeonRegisters::~LeonRegisters()
{
    return;
}

void LeonRegisters::SelfInit()
{
    /* Create Qemu interface */
    QemuSingleton::GetInstance()->create_worker_process(this->BlackLionConnectionInfo);
    
    /* Declare output messages and associated snorkels */
    this->rwSpeedsMsgID_fswIn = SystemMessaging::GetInstance()->CreateNewMessage(this->rwSpeedsMsgName_fswIn,
                     sizeof(RWSpeedIntMsg), this->outputBufferCount, "RWSpeedIntMsg", moduleID);
    RegisterReader* Snorkel_RWSpeeds = new RegisterReader(this->rwSpeedsMsgName_fswIn, this->rwSpeedsMsgID_fswIn, sizeof(RWSpeedIntMsg));
    
    this->simpleNavAttMsgID_fswIn = SystemMessaging::GetInstance()->CreateNewMessage(this->simpleNavAttMsgName_fswIn, sizeof(NavAttIntMsg),
                                                                                     this->outputBufferCount, "NavAttIntMsg", moduleID);
    RegisterReader* Snorkel_AttNav = new RegisterReader(this->simpleNavAttMsgName_fswIn, this->simpleNavAttMsgID_fswIn, sizeof(NavAttIntMsg));
    
    /* Add snorkel-readers to the Qemu interface */
    QemuSingleton::GetInstance()->add_snorkel_reader(Snorkel_RWSpeeds);
    QemuSingleton::GetInstance()->add_snorkel_reader(Snorkel_AttNav);
    
    return;
}

void LeonRegisters::CrossInit()
{
    /* Define input messages and associated snorkels  */
    this->rwTorqueMsgID_fswOut = SystemMessaging::GetInstance()->subscribeToMessage(this->rwTorqueMsgName_fswOut, sizeof(RWArrayTorqueIntMsg), moduleID);
    RegisterWriter* Snorkel_RWTorques = new RegisterWriter(this->rwTorqueMsgName_fswOut, this->rwTorqueMsgID_fswOut, sizeof(RWArrayTorqueIntMsg));
    
    /* Add snorkel-writers to the Qemu interface */
    QemuSingleton::GetInstance()->add_snorkel_writer(Snorkel_RWTorques);
    return;
}

void LeonRegisters::read_rwTorque()
{
    SingleMessageHeader LocalHeader;
    RWArrayTorqueIntMsg rw_torques_buffer;
    memset(&(rw_torques_buffer), 0x0, sizeof(RWArrayTorqueIntMsg));
    SystemMessaging::GetInstance()->ReadMessage(this->rwTorqueMsgID_fswOut, &LocalHeader, sizeof(RWArrayTorqueIntMsg), reinterpret_cast<uint8_t*> (&rw_torques_buffer), this->moduleID);
    QemuSingleton::GetInstance()->update_registers_FSWoutbound(this->rwTorqueMsgID_fswOut, reinterpret_cast<uint8_t*>(&rw_torques_buffer));
}

void LeonRegisters::UpdateState(uint64_t CurrentSimNanos)
{
    /* Read messages from FSW */
    this->read_rwTorque();
    this->update_registers_from_fsw();
    /* Wake up BL interface */
    this->processorExternalWakeup(CurrentSimNanos);
    /* Write messages: from the external world to the rest of the sim */
    this->update_registers_to_fsw(CurrentSimNanos);
    return;
}

void LeonRegisters::update_registers_to_fsw(uint64_t CurrentSimNanos)
{
    /*TODO: write into the BSK msg sys the incoming messages from BL, for the rest of the sim to execute right afterwards */
    /* Subcribe[currTime]:
     -- worker_process: parse_incoming_packet
     -- qemu_node: route_in_packet
     -- writer: add_bytes_to_packet & this->writer_callback_REGinbound()
     */

    std::vector<RegisterWriter*> local_writers = QemuSingleton::GetInstance()->get_writers();
    std::vector<RegisterWriter*>::iterator it;
    for(it = local_writers.begin(); it!=local_writers.end(); it++)
    {
        uint8_t *local_buffer = new uint8_t[(*it)->msg_size];
        memset(local_buffer, 0x0, (*it)->msg_size);
        memcpy(local_buffer, (*it)->messageData.data(), (*it)->msg_size);
        SystemMessaging::GetInstance()->WriteMessage((*it)->msg_id, CurrentSimNanos, (*it)->msg_size, &local_buffer[0], this->moduleID);
        delete [] local_buffer;
    }
    return;
}

void LeonRegisters::update_registers_from_fsw()
{
    SingleMessageHeader LocalHeader;
    std::vector<RegisterReader*> local_readers = QemuSingleton::GetInstance()->get_readers();
    std::vector<RegisterReader*>::iterator it;
    for(it = local_readers.begin(); it!=local_readers.end(); it++)
    {
        SystemMessaging::GetInstance()->ReadMessage((*it)->msg_id, &LocalHeader, (*it)->msg_size, (*it)->current_buffer, this->moduleID);
        (*it)->create_packet_from_FSWword();
    }
    return;
}

void LeonRegisters::Reset(uint64_t CurrentSimNanos)
{
    return;
}

void LeonRegisters::processorExternalWakeup(uint64_t CurrentSimNanos)
{
    std::cout << "LeonRegisters::processorExternalWakeup()" << std::endl;
    QemuSingleton::GetInstance()->setQemuTime(CurrentSimNanos);
    QemuSingleton::GetInstance()->execute_tock();
    QemuSingleton::GetInstance()->poll_worker_process(); // here we tick, publish & subcribe
    return;
}





