#include "leon_registers.h"
#include <iostream>


LeonRegisters::LeonRegisters()
{
    this->outputBufferCount = 2;
    this->rwTorqueMsgName_fswOut = "reactionwheel_torques";
    this->rwSpeedsMsgName_fswIn = "reactionwheel_output_states";
    this->simpleNavAttMsgName_fswIn = "simple_att_nav_output";
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
    /* Define input message/s */
    std::cout << "LeonRegisters::UpdateState" << std::endl;
    //! - Zero the input buffer and read the incoming message
    SingleMessageHeader LocalHeader;
    RWArrayTorqueIntMsg rw_torques_buffer;
    memset(&(rw_torques_buffer), 0x0, sizeof(RWArrayTorqueIntMsg));
    SystemMessaging::GetInstance()->ReadMessage(this->rwTorqueMsgID_fswOut, &LocalHeader, sizeof(RWArrayTorqueIntMsg), reinterpret_cast<uint8_t*> (&rw_torques_buffer),moduleID);
    
    return;
}
