#ifndef blZmqSupportTypes_h
#define blZmqSupportTypes_h

#include <string>

typedef struct blHydraMsgHeader {
    uint32_t numBytesMsgName;
    uint32_t numBytesPayload;
    uint32_t packetType;
    uint32_t msgID;
} BlHydraMsgHeader;

typedef enum zmqTransportType {
    TCP = 1,
    IPC
} ZmqTransportType;

class ZmqSocketConfig {

public:
    ZmqTransportType type = TCP;
    int port = 0;
    std::string ipAddress = "";
    
    std::string generateAddressString(){
        std::string add = "";
        if (this->type == TCP){
            add += "tcp://";
        } else if (this->type == IPC) {
            add += "ipc://";
        }
        add += this->ipAddress;
        add += ":";
        add += std::to_string(this->port);
        return add;
    };
};

#define BL_TICK "TICK"
#define BL_TOCK "TOCK"
#define BL_UNKNOWN_MSGS "UNKNOWN_MSGS"
#define BL_MATCH_MSGS "MATCH_MSGS"
#define BL_START "START"
#define BL_FINISH "FINISH"
#define BL_STARTED "STARTED"
#define BL_FINISHED "FINISHED"
#define BL_HANDSHAKE "HANDSHAKE"
#define BL_HANDSHAKEN "HANDSHAKEN"

#define BL_MSG_TYPE_BSK 1
#define BL_MSG_TYPE_CCSDS 2

#endif /* hydraZmqSupportTypes_h */
