#include "leon_registers.h"
#include <iostream>
#include "qemu_singleton.hpp"



LeonRegisters::LeonRegisters()
{
    this->outputBufferCount = 2;
    this->rwTorqueMsgName_fswOut = "reactionwheel_torques";
    this->rwSpeedsMsgName_fswIn = "reactionwheel_output_states";
    this->simpleNavAttMsgName_fswIn = "simple_att_nav_output";
    
    this->BlackLionInterfacePresent = false;
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
    this->create_singleton_worker_process();
    
    /* Declare output messages and associated snorkels */
    this->rwSpeedsMsgID_fswIn = SystemMessaging::GetInstance()->CreateNewMessage(this->rwSpeedsMsgName_fswIn,
                     sizeof(RWSpeedIntMsg), this->outputBufferCount, "RWSpeedIntMsg", moduleID);
    RegisterReader* Snorkel_RWSpeeds = new RegisterReader(this->rwSpeedsMsgName_fswIn, this->rwSpeedsMsgID_fswIn, sizeof(RWSpeedIntMsg));
    
    this->simpleNavAttMsgID_fswIn = SystemMessaging::GetInstance()->CreateNewMessage(this->simpleNavAttMsgName_fswIn, sizeof(NavAttIntMsg),
                                                                                     this->outputBufferCount, "NavAttIntMsg", moduleID);
    RegisterReader* Snorkel_AttNav = new RegisterReader(this->simpleNavAttMsgName_fswIn, this->simpleNavAttMsgID_fswIn, sizeof(NavAttIntMsg));
    
    /*TODOO: BIND SNORKEL CALLBACKS */
    
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

void LeonRegisters::UpdateState(uint64_t CurrentSimNanos)
{
    /* Read messages from FSW */
    SingleMessageHeader LocalHeader;
    RWArrayTorqueIntMsg rw_torques_buffer;
    memset(&(rw_torques_buffer), 0x0, sizeof(RWArrayTorqueIntMsg));
    SystemMessaging::GetInstance()->ReadMessage(this->rwTorqueMsgID_fswOut, &LocalHeader, sizeof(RWArrayTorqueIntMsg), reinterpret_cast<uint8_t*> (&rw_torques_buffer), this->moduleID);
    QemuSingleton::GetInstance()->update_registers_FSWoutbound(this->rwTorqueMsgID_fswOut, reinterpret_cast<uint8_t*>(&rw_torques_buffer));
    
    /* Find associated Singleton reader and add each message as a FSW packet:
        -- Find reader by message ID. Reader should have the pair <ID, msgName(i.e. stream_name)> and dataPackets (although we might want to preseve only the latest packet)
     */
    
    /* Wake up BL interface*/
    // if (this->BlackLionInterfacePresent)
    this->processorExternalWakeup(CurrentSimNanos);
    return;
}

void LeonRegisters::Reset(uint64_t CurrentSimNanos)
{
    return;
}

void LeonRegisters::processorExternalWakeup(uint64_t CurrentSimNanos)
{
    std::cout << "LeonRegisters::processorExternalWakeup()" << std::endl;
    this->save_current_clock(CurrentSimNanos);
    this->send_tock_msg(); // here we tock
    this->poll_worker_sockets(); // here we tick, publish & subcribe
    return;
}

void LeonRegisters::create_sbc_register()
{
    std::cout << "LeonRegisters::create_sbc_register()" << std::endl;
    /* Define readers and writers
     -- Readers: we need one for each fswOut msg with valid ID (those in CrossInit).
     -- Writers: we need one for each fswIn msg as (declared in SelfInit)
     */
    
    
    /* Create register buffer*/
    //QemuSingleton* local_qemu = QemuSingleton::GetInstance();
    //local_qemu->sbc_buffer_index = local_qemu->attach_storage_container("sbc_register", 1048575);
    return;
}
void LeonRegisters::create_singleton_worker_process()
{
    std::cout << "LeonRegisters. BlackLionConnectionInfo = " << this->BlackLionConnectionInfo <<std::endl;
    QemuSingleton* registers_interface =  QemuSingleton::GetInstance();
    registers_interface->create_worker_process(this->BlackLionConnectionInfo);
    return;
}
void LeonRegisters::poll_worker_sockets()
{
    /* Tick. */
    
    /* Publish [prevTime]: pakcet/s stored in each reader go out to the world.
     */
    /* Subcribe[currTime]:
     -- worker_process: parse_incoming_packet
     -- qemu_node: route_in_packet
     -- writer: add_bytes_to_packet & this->writer_callback_REGinbound()
     Idea: could the writer_callback be simply a WriteMessage? the writer would have to know  [MessageID, uint64_t ClockTimeNanos, MsgSize, void *MsgPayload]
     */
    return;
}

void LeonRegisters::send_tock_msg()
{
    QemuSingleton* registers_interface =  QemuSingleton::GetInstance();
    registers_interface->execute_tock();
    return;
}
void LeonRegisters::save_current_clock(uint64_t clockTime)
{
    return;
}





