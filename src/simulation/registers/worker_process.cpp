#include "worker_process.hpp"

/*********************************** DATA EXCHANGE OBJECT ***********************************/
DataExchangeObject::DataExchangeObject(int payload_size, std::vector<unsigned char> payload)
{
    this->packet_type = "2";
    this->last_update_counter = 0;
    this->payload_size = 0;
    this->payload.clear();
    this->update_data(payload_size, payload);
};
DataExchangeObject::~DataExchangeObject()
{
    this->payload.clear();
    return;
};

void DataExchangeObject::update_data(int payload_size, std::vector<unsigned char> new_payload)
{
    this->payload_size = payload_size;
    this->payload.resize(payload_size);
    this->payload = new_payload;
    this->last_update_counter += 1;
    return;
}

std::string DataExchangeObject::marshall_payload()
{
    //std::string pay_string = std::string(reinterpret_cast<const char*>(&payload[0]), payload.size());
    //std::cout << "payload = " << pay_string << std::endl << std::endl;

    std::vector<unsigned char> cdata(this->payload.size());
    std::vector<std::string> hex_payload_vec;
    int i = 0;
    for (unsigned int x : this->payload)
    {
        cdata[i] = (unsigned char) x;
        std::string curr_hex = BL::int_to_hex(x);
        hex_payload_vec.push_back(curr_hex);
        i ++;
    }
    std::ostringstream hex_temp_str;
    std::copy(hex_payload_vec.begin(), hex_payload_vec.end(),std::ostream_iterator<std::string>(hex_temp_str, ""));
    std::string hex_payload_str = hex_temp_str.str();
    //std::cout << "parsed hex_string = " << hex_payload_str << std::endl << std::endl;

    return(hex_payload_str);
    //return(pay_string);
}


/*********************************** WORKER PROCESS ***********************************/
WorkerProcess::WorkerProcess()
{
    /* Constructor should create the context, the command REP socket, and the PUB & SUB sockets */
    this->context = new zmqpp::context();
    this->rep_socket = new zmqpp::socket(*this->context, zmqpp::socket_type::rep);
    this->sub_socket = new zmqpp::socket(*this->context, zmqpp::socket_type::sub);
    this->pub_socket = new zmqpp::socket(*this->context, zmqpp::socket_type::pub);
   
    //this->poller = new zmqpp::poller();
    
    this->subcribed_msgs_next_count = 0;
    this->frame_time = 0.0;
    this->accum_sim_time = 0.0;
    return;
}

WorkerProcess::~WorkerProcess()
{
    return;
}

/* POLLER HANDLING */
void WorkerProcess::start_poller()
{
    this->rep_socket->connect(this->master_address);
    std::cout << "Rep socket connected to master address: "<< this->master_address << std::endl;
    this->reactor = new zmqpp::reactor();
    this->reactor->add(*this->rep_socket, this->rep_callback);
    return;
}
void WorkerProcess::handle_rep_callback()
{
    zmqpp::message msg;
    this->rep_socket->receive(msg);
    this->parse_command(&msg);
};

void WorkerProcess::handle_rep_interrupt_callback()
{
    zmqpp::message msg;
    bool interrupted = true;
    while(interrupted) {
        interrupted = false;
        try
        {
            this->rep_socket->receive(msg);
            this->parse_command(&msg);
        }
        catch (zmqpp::zmq_internal_exception &ex)
        {
            if (ex.zmq_error() != EINTR)
            {
                throw;
            }  else {
                interrupted = true;
            }
        }
    }
    return;
};

/* SUB RECV HANDLING */
bool WorkerProcess::handle_sub_callback()
{
    std::cout << "WorkerProcess::handle_subscribe()" << std::endl;
    zmqpp::message msg;
    bool msg_received = false;
    if(this->sub_socket->receive(msg))
    {
        this->parse_incoming_packet(msg.copy());
        msg_received = true;
    }
    return msg_received;
};
bool WorkerProcess::handle_sub_interrupt_callback()
{
    std::cout << "WorkerProcess::handle_subscribe()" << std::endl;
    zmqpp::message msg;
    bool msg_received = false;
    bool interrupted = true;
    while(interrupted) {
        interrupted = false;
        try
        {
            if(this->sub_socket->receive(msg))
            {
                this->parse_incoming_packet(msg.copy());
                msg_received = true;
            }
        }
        catch (zmqpp::zmq_internal_exception &ex)
        {
            if (ex.zmq_error() != EINTR)
            {
                throw;
            } else {
                interrupted = true;
            }
        }
    }
    return msg_received;
};

/* SOCKETS */
void WorkerProcess::set_master_address(const std::string master_address)
{
    this->master_address = master_address;
}
/* SOCKETS: Start Calls */
void WorkerProcess::set_backend_address(const std::string backend_str)
{
    this->backend_address = backend_str;
}
void WorkerProcess::set_frontend_address(const std::string frontend_str)
{
    this->frontend_address = frontend_str;
}
void WorkerProcess::connect_sub_to_backend(const std::string backend_address)
{
    this->sub_socket->connect(backend_address);
}
void WorkerProcess::connect_pub_to_frontend(const std::string frontend_address)
{
    this->pub_socket->connect(frontend_address);
}

/* SOCKETS: Finish Calls */
void WorkerProcess::disconnect_sub_socket(){ this->sub_socket->disconnect(this->backend_address); }
void WorkerProcess::disconnect_pub_socket(){ this->pub_socket->disconnect(this->frontend_address); }
void WorkerProcess::close_command_socket(){ this->rep_socket->disconnect(this->master_address); }
void WorkerProcess::close_pub_sub_sockets()
{
    this->disconnect_pub_socket();
    this->disconnect_sub_socket();
}
/* SIM: Finish Calls */
void WorkerProcess::reset()
{
    this->accum_sim_time = 0.0;
    this->frame_time = 0.0;
}

/* PARSE REQUESTS */
void WorkerProcess::parse_command(zmqpp::message *message_cmd)
{
    std::string msg_str = message_cmd->get(0);
    std::cout << "COMMAND: " << msg_str << std::endl;
    tickReceived=false; 
    if (msg_str == BL_START)
    {
        this->parse_start_message(message_cmd->copy());
        this->rep_socket->send(std::string(BL_STARTED));
    }
    else if(msg_str == BL_UNKNOWN_MSGS)
    {
        zmqpp::message reply = this->prepare_subscribed_message();
        this->rep_socket->send(reply);
    }
    
    else if(msg_str == BL_MATCH_MSGS)
    {
        this->parse_match_message(message_cmd->copy());
        zmqpp::message reply = this->prepare_matched_message();
        this->rep_socket->send(reply);
    }
    
    else if(msg_str == BL_TICK)
    {
        this->parse_tick_message(message_cmd->copy());
        this->publish();
        this->subscribe();
        this->step_process();
        zmqpp::message tock_msg = this->tock_node_callback();
        this->rep_socket->send(tock_msg);
        tickReceived=true;
    }
    else if(msg_str == BL_FINISH)
    {
        /* Reset Sim variables*/
        this->reset();
        /* Disconnect PUB & SUB sockets */
        this->close_pub_sub_sockets();
        /* Finish poller */
        this->interrupted_node = true;
        /* Send reply before closing */
        this->rep_socket->send(std::string(BL_FINISHED));
        /* Disconnect REP socket */
        this->close_command_socket();
    }
    else if(msg_str == BL_HANDSHAKE)
    {
        this->rep_socket->send(std::string(BL_HANDSHAKEN));
        this->handshake_frontend();
    }
}


void WorkerProcess::handshake_frontend()
{
    this->pub_socket->send(std::string(BL_HANDSHAKE));
    std::cout << "Node has published msg: "<< std::string(BL_HANDSHAKE) << std::endl;
}

void WorkerProcess::parse_start_message(zmqpp::message message)
{
    std::string backend_str = message.get(1);
    std::string frontend_str = message.get(2);
    if (!backend_str.empty())
    {
        this->set_backend_address(backend_str);
        this->connect_sub_to_backend(backend_str);
        std::cout << "Sub socket connected to backend address: "<< backend_str << std::endl;
    }
    if (!frontend_str.empty())
    {
        this->set_frontend_address(frontend_str);
        this->connect_pub_to_frontend(frontend_str);
        std::cout << "Pub socket connected to fronted address: "<< frontend_str << std::endl;
    }
}

void WorkerProcess::parse_match_message(zmqpp::message message)
{
    for (int i = 0; i < int(message.parts()); i++) {
        std::string msg_name = message.get(i);
        //std::cout << "Checking match for: " << msg_name << std::endl;
        if (find(this->registers_all.begin(), this->registers_all.end(), msg_name)!= this->registers_all.end())
        {
            this->publications.insert(msg_name);
            //std::cout << "Match found!" << std::endl;
        }
    }
}

void WorkerProcess::parse_tick_message(zmqpp::message message)
{
    this->frame_time = std::stod(message.get(1));
    this->subcribed_msgs_next_count = std::stoi(message.get(2));
    //std::cout << "frame_time = " << this->frame_time;
    //std::cout <<"; sub_msg_next_count = " << this->subcribed_msgs_next_count << std::endl;
}

void WorkerProcess::parse_incoming_packet(zmqpp::message message)
{
    std::string msg_name = message.get(0);
    std::string packet_type = message.get(1);
    int pay_size = std::stoi(message.get(2));
    std::string payload_string = message.get(3);
    this->route_in_packet(msg_name, packet_type, pay_size, payload_string);
    return;
}


/* SYNCH CALLS*/
void WorkerProcess::publish()
{
    //std::cout << "PUBLISH() " << std::endl;
    this->publish_node_callback();
    return;
}

void WorkerProcess::subscribe()
{
    //std::cout << "SUBSCRIBE() " << std::endl;
    zmqpp::message sub_msg;
    int local_msg_count = 0;
    while (local_msg_count != this->subcribed_msgs_next_count)
    {
        if (this->sub_callback()) {
            local_msg_count += 1;
            //std::cout << "local_msg_count = " << local_msg_count << std::endl;
        }
    }
    //std::cout << "All subscriptions received. " << std::endl;
    return;
}


void WorkerProcess::step_process()
{
    this->accum_sim_time += this->frame_time;
    //std::cout << "Accumulated sim time = " << this->accum_sim_time << " seconds." << std::endl;
    this->step_node_callback();
}

/* PREPARE REPLIES */
zmqpp::message WorkerProcess::prepare_subscribed_message()
{
    std::set<std::string>::iterator it;
    zmqpp::message msg = std::string(BL_UNKNOWN_MSGS);
    for(it = this->subscriptions.begin(); it!=this->subscriptions.end(); it++)
    {
        this->sub_socket->subscribe(it->data());
        msg.add(it->data());
        //std::cout << "subscription = " << it->data() << std::endl;
    }
    return(msg);
}

zmqpp::message WorkerProcess::prepare_matched_message()
{
    std::set<std::string>::iterator it;
    zmqpp::message msg = std::string(BL_MATCH_MSGS);
    for(it = this->publications.begin(); it!=this->publications.end(); it++)
    {
        msg.add(it->data());
        //std::cout << "publication = " << it->data() << std::endl;
    }
    return(msg);
}
