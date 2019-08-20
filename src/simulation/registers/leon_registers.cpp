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
    /* Declare output message/s */
    this->rwSpeedsMsgID_fswIn = SystemMessaging::GetInstance()->CreateNewMessage(this->rwSpeedsMsgName_fswIn,
                     sizeof(RWSpeedIntMsg), this->outputBufferCount, "RWSpeedIntMsg", moduleID);
    
    this->simpleNavAttMsgID_fswIn = SystemMessaging::GetInstance()->CreateNewMessage(this->simpleNavAttMsgName_fswIn, sizeof(NavAttIntMsg), this->outputBufferCount, "NavAttIntMsg", moduleID);
    return;
}

void LeonRegisters::CrossInit()
{
    /* Define input message/s */
    this->rwTorqueMsgID_fswOut = SystemMessaging::GetInstance()->subscribeToMessage(this->rwTorqueMsgName_fswOut, sizeof(RWArrayTorqueIntMsg), moduleID);
    return;
}

void LeonRegisters::UpdateState(uint64_t CurrentSimNanos)
{
    /* Read messages from FSW */
    SingleMessageHeader LocalHeader;
    RWArrayTorqueIntMsg rw_torques_buffer;
    memset(&(rw_torques_buffer), 0x0, sizeof(RWArrayTorqueIntMsg));
    SystemMessaging::GetInstance()->ReadMessage(this->rwTorqueMsgID_fswOut, &LocalHeader, sizeof(RWArrayTorqueIntMsg), reinterpret_cast<uint8_t*> (&rw_torques_buffer),moduleID);
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
    // if (this->BlackLionInterfacePresent)
    this->create_singleton_worker_process();
    this->create_sbc_register();
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
    return;
}
void LeonRegisters::create_singleton_worker_process()
{
    std::cout << "LeonRegisters::create_singleton_worker_process()" << std::endl;
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





