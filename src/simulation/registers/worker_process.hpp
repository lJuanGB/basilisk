#ifndef _WORKER_PROCESS_HH_
#define _WORKER_PROCESS_HH_

#include <string>
#include <set>
#include <algorithm>
#include "zmqpp.hpp"
#include "blZmqSupportTypes.h"
#include "blUtilities.h"

/*********************************** DATA EXCHANGE OBJECT ***********************************/
class DataExchangeObject
{
public:
    DataExchangeObject(int payload_size, std::vector<unsigned char> new_payload);
    virtual ~DataExchangeObject();
    
    int last_update_counter;
    std::string packet_type;
    int payload_size;
    std::vector<unsigned char> payload;
public:
    void update_data(int payload_size, std::vector<unsigned char> new_payload);
    std::string marshall_payload();
};

/*********************************** WORKER PROCESS ***********************************/
class WorkerProcess
{
    
public:
    WorkerProcess();
    ~WorkerProcess();
    
    zmqpp::message prepare_subscribed_message();
    zmqpp::message prepare_matched_message();
    
    void publish();
    void subscribe();
    void step_process();
    std::function<void()> publish_node_callback;
    std::function<void()> step_node_callback;
    std::function<zmqpp::message()> tock_node_callback;
    
    void parse_command(zmqpp::message *message_cmd);
    void parse_start_message(zmqpp::message message);
    void parse_match_message(zmqpp::message message);
    void parse_tick_message(zmqpp::message message);
    void parse_incoming_packet(zmqpp::message message);
    virtual void route_in_packet(std::string msg_name, std::string packet_type, int pay_size, std::string pay_hex){};
    
    
    /* Handshaking */
    void handshake_frontend();
    
    /* Sockets related */
    void close_pub_sub_sockets(); /* disconnects PUB & SUB sockets */
    void close_command_socket();
    void connect_sub_to_backend(const std::string backend_address);
    void connect_pub_to_frontend(const std::string frontend_address);
    void set_backend_address(const std::string backend_str); /* stores address string for SUB socket */
    void set_frontend_address(const std::string frontend_str); /* stores address string for PUB socket */
    void set_master_address(const std::string master_address); /* stores address string for REP socket */
    
    void reset(); /* resets simulation variables*/
    
    /* Sub socket receive functions*/
    std::function<bool()> sub_callback;
    bool handle_sub_callback();
    bool handle_sub_interrupt_callback();
    
    /* Poller functions */
    void start_poller();
    std::function<void()> rep_callback;
    void handle_rep_callback();
    void handle_rep_interrupt_callback();
    
public:
    zmqpp::context *context;
    zmqpp::socket *rep_socket;
    zmqpp::socket *sub_socket;
    zmqpp::socket *pub_socket;
    bool interrupted_node = false;
    bool tickReceived;
    
    /* Poller vars */
    zmqpp::reactor *reactor;
protected:
    std::set<std::string> subscriptions;
    std::set<std::string> publications;
    std::set<std::string> registers_all;
    
private:
    void disconnect_sub_socket();
    void disconnect_pub_socket();
    
private:
    std::string master_address;
    std::string frontend_address;
    std::string backend_address;
    
    int subcribed_msgs_next_count;
    double frame_time;
    double accum_sim_time;
};


#endif /* _WORKER_PROCESS_HH_ */
