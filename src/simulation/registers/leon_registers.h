#ifndef LEON_REGISTERS_H
#define LEON_REGISTERS_H

#include "_GeneralModuleFiles/sys_model.h"
#include "architecture/messaging/system_messaging.h"
#include "simFswInterfaceMessages/rwArrayTorqueIntMsg.h"
#include "simFswInterfaceMessages/rwSpeedIntMsg.h"
#include "simFswInterfaceMessages/navAttIntMsg.h"
#include <vector>
//#include "zmqpp/zmqpp.hpp"


class LeonRegisters : public SysModel {
public:
    LeonRegisters();
    ~LeonRegisters();
    void SelfInit();
    void CrossInit();
    void UpdateState(uint64_t CurrentSimNanos);
    void Reset(uint64_t CurrentSimNanos);
    
    void read_rwTorque();
    
private:
    void processorExternalWakeup(uint64_t CurrentSimNanos);
    void update_registers_from_fsw();
    void update_registers_to_fsw(uint64_t CurrentSimNanos);
    //void save_connect_info(char *connect_string);
    
public:
    uint64_t outputBufferCount;
    std::string rwTorqueMsgName_fswOut;
    std::string rwSpeedsMsgName_fswIn;
    std::string simpleNavAttMsgName_fswIn;
    
    std::string BlackLionConnectionInfo;

private:
    int64_t rwTorqueMsgID_fswOut;
    int64_t rwSpeedsMsgID_fswIn;
    int64_t simpleNavAttMsgID_fswIn;
    
};



#endif /* LEON_REGISTERS_H */
