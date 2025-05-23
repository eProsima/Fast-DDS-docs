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

/**
 * @file CalculatorClient.cpp
 *
 */

#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/qos/DomainParticipantQos.hpp>
#include <fastdds/dds/domain/qos/RequesterQos.hpp>
#include <fastdds/dds/rpc/exceptions.hpp>
#include <fastdds/dds/rpc/interfaces/RpcFuture.hpp>

#include "types/calculator.hpp"
#include "types/calculatorClient.hpp"

using namespace calculator_example;
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::dds::rpc;

//!--OPERATION_STATUS
enum class OperationStatus
{
    SUCCESS,
    TIMEOUT,
    ERROR
};
//!--

//!--OPERATION_TYPE
enum class OperationType
{
    ADDITION,
    SUBTRACTION,
    REPRESENTATION_LIMITS
};
//!--

//!--OPERATION_INTERFACE
class Operation
{

public:

    virtual OperationStatus execute() = 0;

};
//!--

//!--REPRESENTATION_LIMITS
class RepresentationLimits : public Operation
{

public:

    RepresentationLimits(
            std::shared_ptr<Calculator> client)
        : client_(client)
    {
    };

    OperationStatus execute() override
    {
        // Send the request to the server and wait for the reply
        if (auto client = client_.lock())
        {
            RpcFuture<void> future = client->representation_limits(min_value_, max_value_);

            if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
            {
                std::cerr << "Operation timed out" << std::endl;

                return OperationStatus::TIMEOUT;
            }

            try
            {
                future.get();

                // Print the results
                std::cout <<
                        "Representation limits received: min_value = " << min_value_
                        << ", max_value = " << max_value_ << std::endl;

                return OperationStatus::SUCCESS;
            }
            catch (const RpcException& e)
            {
                std::cerr << "RPC exception occurred: " << e.what() << std::endl;

                return OperationStatus::ERROR;
            }
        }
        else
        {
            throw std::runtime_error("Client reference expired");
        }
    };

protected:

    std::int32_t min_value_;
    std::int32_t max_value_;
    std::weak_ptr<Calculator> client_;

};
//!--

//!--ADDITION
class Addition : public Operation
{

public:

    Addition(
            std::shared_ptr<Calculator> client,
            std::int32_t x,
            std::int32_t y)
        : x_(x)
        , y_(y)
        , client_(client)
    {
    }

    OperationStatus execute() override
    {
        // Send the request to the server and wait for the reply
        if (auto client = client_.lock())
        {
            RpcFuture<int32_t> future = client->addition(x_, y_);

            if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
            {
                std::cerr << "Operation timed out" << std::endl;

                return OperationStatus::TIMEOUT;
            }

            try
            {
                result_ = future.get();
                // Print the result
                std::cout << "Addition result: " << x_ << " + " << y_ << " = " << result_ << std::endl;

                return OperationStatus::SUCCESS;
            }
            catch (const RpcException& e)
            {
                std::cerr << "RPC exception occurred: " << e.what() << std::endl;

                return OperationStatus::ERROR;
            }
        }
        else
        {
            throw std::runtime_error("Client reference expired");
        }
    };

protected:

    std::int32_t x_;
    std::int32_t y_;
    std::int32_t result_;
    std::weak_ptr<Calculator> client_;

};
//!--

//!--SUBTRACTION
class Subtraction : public Operation
{

public:

    Subtraction(
            std::shared_ptr<Calculator> client,
            std::int32_t x,
            std::int32_t y)
        : x_(x)
        , y_(y)
        , client_(client)
    {
    }

    OperationStatus execute() override
    {
        // Send the request to the server and wait for the reply
        if (auto client = client_.lock())
        {
            RpcFuture<int32_t> future = client->subtraction(x_, y_);

            if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
            {
                std::cerr << "Operation timed out" << std::endl;

                return OperationStatus::TIMEOUT;
            }

            try
            {
                result_ = future.get();

                // Print the result
                std::cout << "Subtraction result: " << x_ << " - " << y_ << " = " << result_ << std::endl;

                return OperationStatus::SUCCESS;
            }
            catch (const RpcException& e)
            {
                std::cerr << "RPC exception occurred: " << e.what() << std::endl;

                return OperationStatus::ERROR;
            }
        }
        else
        {
            throw std::runtime_error("Client reference expired");
        }
    };

protected:

    std::int32_t x_;
    std::int32_t y_;
    std::int32_t result_;
    std::weak_ptr<Calculator> client_;

};
//!--

class Client
{

public:

    Client(
            const std::string& service_name)
        : client_(nullptr)
        , participant_(nullptr)
        , service_name_(service_name)
    {
    }

//!--DESTRUCTOR
    ~Client()
    {
        client_.reset();

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
            throw std::runtime_error("Participant initialization failed");
        }

        // Create the client with default QoS
        RequesterQos qos;

        client_ = create_CalculatorClient(*participant_, service_name_.c_str(), qos);

        if (!client_)
        {
            throw std::runtime_error("Failed to create client");
        }

        std::cout << "Client initialized with ID: " << participant_->guid().guidPrefix << std::endl;
    }
//!--

    //! Send a request to the server and wait for the reply.
    //  Returns true if the reply was successfully received, false otherwise.
//!--SEND_REQUEST
    bool send_request(const OperationType& operation)
    {
        // Set the operation to be executed
        set_operation(operation);

        // Execute the operation
        if (operation_)
        {
            OperationStatus status = operation_->execute();

            return (status == OperationStatus::SUCCESS);
        }

        return false;
    }
//!--

protected:

    //! Set the operation to be executed.
//!--SET_OPERATION
    void set_operation(
            const OperationType& operation)
    {
        switch (operation)
        {
            case OperationType::ADDITION:
                operation_ = std::unique_ptr<Operation>(new Addition(client_, 5, 3));
                break;
            case OperationType::SUBTRACTION:
                operation_ = std::unique_ptr<Operation>(new Subtraction(client_, 5, 3));
                break;
            case OperationType::REPRESENTATION_LIMITS:
                operation_ = std::unique_ptr<Operation>(new RepresentationLimits(client_));
                break;
            default:
                throw std::runtime_error("Invalid operation type");
        }
    }
//!--

//!--CLIENT_PROTECTED_MEMBERS
    std::shared_ptr<Calculator> client_;
    DomainParticipant* participant_;
    std::unique_ptr<Operation> operation_;
    std::string service_name_;
//!--

};

//!--MAIN
int main(
        int argc,
        char** argv)
{
    // Parse operation type from command line arguments
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <operation_type>" << std::endl;
        std::cerr << "Available operations: add, sub, rep" << std::endl;
        return 1;
    }

    OperationType operation;

    if (std::string(argv[1]) == "add")
    {
        operation = OperationType::ADDITION;
    }
    else if (std::string(argv[1]) == "sub")
    {
        operation = OperationType::SUBTRACTION;
    }
    else if (std::string(argv[1]) == "rep")
    {
        operation = OperationType::REPRESENTATION_LIMITS;
    }
    else
    {
        std::cerr << "Invalid operation type" << std::endl;
        return 1;
    }

    // Create the client
    Client client("CalculatorService");

    // Initialize the client
    client.init();

    // Wait for endpoint matching
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    int n_attempts = 10;
    for (int i = 0; i < n_attempts; ++i)
    {
        std::cout << "Attempting to send request, attempt " << (i + 1) << "/" << n_attempts << std::endl;

        // Send a request to the server
        if (client.send_request(operation))
        {
            std::cout << "Request sent successfully" << std::endl;
            break;
        }
        else
        {
            std::cerr << "Failed to send request" << std::endl;
        }

        if (i == n_attempts - 1)
        {
            std::cerr << "Failed to send request after " << n_attempts << " attempts" << std::endl;
            return 1;
        }
        // Wait for a while before trying again
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
//!--
