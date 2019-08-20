#ifndef _QEMU_ROUTER_HH_
#define _QEMU_ROUTER_HH_

#include <string>
#include <stdint.h>

class QemuRouter
{
public:
    QemuRouter(std::string msg_name, int64_t msg_id, uint64_t msg_size);
    ~QemuRouter();
    QemuRouter(){};
    
public:
    std::string get_router_name();
    int64_t get_msgID();
protected:
    std::string msg_name;
    int64_t msg_id;
    uint64_t msg_size;
};

#endif /* _QEMU_ROUTER_HH_ */
