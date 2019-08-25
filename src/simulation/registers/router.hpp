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
    std::string get_router_name(){ return(this->msg_name); }
    
public:
    int64_t msg_id;
    uint64_t msg_size;
protected:
    std::string msg_name;

};

#endif /* _QEMU_ROUTER_HH_ */
