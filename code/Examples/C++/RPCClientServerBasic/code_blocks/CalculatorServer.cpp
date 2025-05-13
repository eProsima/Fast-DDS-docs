// Copyright 2025 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <csignal>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/qos/ReplierQos.hpp>

#include "types/calculatorServer.hpp"
#include "types/calculatorServerImpl.hpp"

using namespace calculator_example;
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::dds::rpc;

class Server
{

public:

    Server(
            const std::string& service_name)
        : server_(nullptr)
        , participant_(nullptr)
        , service_name_(service_name)
        , stop_(false)
    {
    }

    //!--DESTRUCTOR
    ~Server()
    {
        server_.reset();

        if (participant_)
        {
            // Delete DDS entities contained within the DomainParticipant
            participant_->delete_contained_entities();

            // Delete DomainParticipant
            DomainParticipantFactory::get_shared_instance()->delete_participant(participant_);
        }
    }
    //!--

    //!--INIT
    void init()
    {
        // Create the participant in Domain 0 with default QoS
        auto factory = DomainParticipantFactory::get_shared_instance();

        if (!factory)
        {
            throw std::runtime_error("Failed to get participant factory instance");
        }

        DomainParticipantQos participant_qos;

        participant_ = factory->create_participant(0, participant_qos);

        if (!participant_)
        {
            throw std::runtime_error("Failed to create participant");
        }

        // Create the server with History QoS set to KEEP_ALL
        std::shared_ptr<CalculatorServer_IServerImplementation> server_impl =
            std::make_shared<CalculatorServerImplementation>();

        ReplierQos qos;
        qos.writer_qos.history().kind = KEEP_ALL_HISTORY_QOS;
        qos.reader_qos.history().kind = KEEP_ALL_HISTORY_QOS;

        server_ = create_CalculatorServer(
                        *participant_,
                        service_name_.c_str(),
                        qos,
                        0,
                        server_impl);

        if (!server_)
        {
            throw std::runtime_error("Server initialization failed");
        }

        std::cout << "Server initialized with ID: " << participant_->guid().guidPrefix << std::endl;
    }
    //!--

    //!--STOP
    void stop()
    {
        stop_.store(true);
        server_->stop();

        std::cout << "Server execution stopped" << std::endl;
    }
    //!--

    bool is_stopped()
    {
        return stop_.load();
    }

    //!--RUN
    void run()
    {
        if (is_stopped())
        {
            return;
        }

        server_->run();

        std::cout << "Server running" << std::endl;
    }
    //!--

//!--SERVER_PROTECTED_MEMBERS
protected:

    std::shared_ptr<CalculatorServer> server_;
    DomainParticipant* participant_;
    std::string service_name_;
    std::atomic<bool> stop_;
//!--
};

std::function<void(int)> stop_handler;

void signal_handler(
        int signum)
{
    stop_handler(signum);
}

//!--MAIN
int main(
        int argc,
        char** argv)
{
    // Create the server
    std::shared_ptr<Server> server = std::make_shared<Server>("CalculatorService");

    server->init();

    std::thread thread(&Server::run, server);

    stop_handler = [&](int signum)
    {
        std::cout << "Signal received, stopping execution." << std::endl;
        server->stop();
    };

        signal(SIGINT, signal_handler);
        signal(SIGTERM, signal_handler);
    #ifndef _WIN32
        signal(SIGQUIT, signal_handler);
        signal(SIGHUP, signal_handler);
    #endif // _WIN32

    std::cout << "Server running. Please press Ctrl+C to stop the server at any time." << std::endl;

    thread.join();

    return 0;
}
//!--
