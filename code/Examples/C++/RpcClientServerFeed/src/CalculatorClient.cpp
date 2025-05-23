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
#include <fastdds/dds/rpc/interfaces/RpcClientReader.hpp>
#include <fastdds/dds/rpc/interfaces/RpcClientWriter.hpp>
#include <fastdds/dds/rpc/interfaces/RpcFuture.hpp>

#include "types/calculator.hpp"
#include "types/calculatorClient.hpp"

using namespace calculator_example;
using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::dds::rpc;

//!--INPUT_FEED_PROCESSOR
class InputFeedProcessor
{
public:

    enum class Status
    {
        VALID_INPUT,
        INVALID_INPUT,
        FEED_CLOSED
    };

    static std::pair<Status, int32_t> get_input()
    {
        std::string line;
        if (!std::getline(std::cin, line))
        {
            EPROSIMA_LOG_ERROR(InputFeedProcessor, "An error occurred while reading the input.");
            return std::make_pair(Status::INVALID_INPUT, 0);
        }

        if (line.empty())
        {
            EPROSIMA_LOG_INFO(InputFeedProcessor, "Empty input received. Closing feed.");
            return std::make_pair(Status::FEED_CLOSED, 0);
        }

        long long value = 0;

        try
        {
            value = std::stoll(line);
        }
        catch (const std::invalid_argument&)
        {
            EPROSIMA_LOG_ERROR(InputFeedProcessor, "Invalid input: " << line);
            return std::make_pair(Status::INVALID_INPUT, 0);
        }
        catch (const std::out_of_range&)
        {
            EPROSIMA_LOG_ERROR(InputFeedProcessor, "Input out of range: " << line);
            return std::make_pair(Status::INVALID_INPUT, 0);
        }

        if (value < std::numeric_limits<int32_t>::min() || value > std::numeric_limits<int32_t>::max())
        {
            return std::make_pair(Status::INVALID_INPUT, 0);
        }

        return std::make_pair(Status::VALID_INPUT, static_cast<int32_t>(value));
    }

    static void print_help()
    {
        std::cout << "Input feed help:" << std::endl;
        std::cout << "  - Enter a number to process it." << std::endl;
        std::cout << "  - Press Enter without typing anything to close the input feed." << std::endl;
    }
};
//!--

//!--OPERATION_STATUS
enum class OperationStatus
{
    SUCCESS,
    TIMEOUT,
    ERROR,
    PENDING
};
//!--

enum class OperationType
{
    ADDITION,
    SUBTRACTION,
    REPRESENTATION_LIMITS,
    FIBONACCI,
    SUM_ALL,
    ACCUMULATOR,
    FILTER
};

class Operation
{

public:

    virtual OperationStatus execute() = 0;

};

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

//!--FIBONACCI_SEQ
class FibonacciSeq : public Operation
{

public:

    FibonacciSeq(
            std::shared_ptr<Calculator> client,
            std::uint32_t n_results)
        : n_results_(n_results)
        , client_(client)
        , reader_(nullptr)
    {
    }

    OperationStatus execute() override
    {
        // If no requests have been sent, send a new request to the server
        // If a request has been sent and the feed is still open, wait for the next value
        if (auto client = client_.lock())
        {
            // Send a new request to the server if no request has been sent yet
            if (!reader_)
            {
                reader_ = client->fibonacci_seq(n_results_);

                if (!reader_)
                {
                    std::cerr << "Failed to create Client Reader" << std::endl;

                    return OperationStatus::ERROR;
                }
            }

            // Read the next value from the feed
            int32_t value;
            Duration_t timeout{1, 0}; // 1s

            try
            {
                if (reader_->read(value, timeout))
                {
                    std::cout << "Fibonacci sequence value: " << value << std::endl;

                    // Output feed not closed yet
                    return OperationStatus::PENDING;
                }
                else
                {
                    std::cout << "Fibonacci sequence feed finished" << std::endl;

                    // Request finished, unset the reader before the next request
                    reader_.reset();

                    return OperationStatus::SUCCESS;
                }
            }
            catch (const RpcTimeoutException& e)
            {
                std::cerr << "Operation timed out " << e.what() << std::endl;

                return OperationStatus::TIMEOUT;
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
    }

protected:

    std::uint32_t n_results_;
    std::weak_ptr<Calculator> client_;
    std::shared_ptr<RpcClientReader<int32_t>> reader_;

};
//!--

//!--SUM_ALL
class SumAll : public Operation
{

public:

    SumAll(
            std::shared_ptr<Calculator> client)
        : client_(client)
        , writer_(nullptr)
        , result_(0)
        , input_feed_closed_(false)
    {
    }

    OperationStatus execute() override
    {
        if (auto client = client_.lock())
        {
            RpcFuture<int32_t> future;
            // Parse the input data and send it to the server
            // until the input feed is closed
            try
            {
                while (!input_feed_closed_)
                {
                    if (!writer_)
                    {
                        future = client->sum_all(writer_);
                        if (!writer_)
                        {
                            std::cerr << "Failed to create Client Writer" << std::endl;

                            return OperationStatus::ERROR;
                        }

                        InputFeedProcessor::print_help();
                    }

                    // Get the input from the user
                    auto input = InputFeedProcessor::get_input();

                    // Check the input status
                    switch (input.first)
                    {
                        // Valid number received
                        case InputFeedProcessor::Status::VALID_INPUT:
                            // Send the number to the server
                            writer_->write(input.second);
                            std::cout << "Input sent: " << input.second << std::endl;
                            break;

                        // Invalid input received
                        case InputFeedProcessor::Status::INVALID_INPUT:
                            std::cerr << "Invalid input. Please enter a valid number." << std::endl;
                            break;

                        // Input feed closed
                        case InputFeedProcessor::Status::FEED_CLOSED:
                            std::cout << "Input feed closed." << std::endl;
                            input_feed_closed_ = true;
                            writer_->finish();
                            break;

                        default:
                            std::cerr << "Unknown input status." << std::endl;
                            break;
                    }
                }

                if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
                {
                    std::cerr << "Operation timed out" << std::endl;

                    return OperationStatus::TIMEOUT;
                }

                result_ = future.get();

                std::cout << "Sum result: " << result_ << std::endl;
                writer_.reset();

                return OperationStatus::SUCCESS;
            }
            catch (const RpcException& e)
            {
                std::cerr << "Exception ocurred: " << e.what() << std::endl;

                return OperationStatus::ERROR;
            }
        }
        else
        {
            throw std::runtime_error("Client reference expired");
        }
    }

protected:

    std::weak_ptr<Calculator> client_;
    std::shared_ptr<RpcClientWriter<int32_t>> writer_;
    std::int32_t result_;
    bool input_feed_closed_;

};
//!--

//!--ACCUMULATOR
class Accumulator : public Operation
{

public:

    Accumulator(
            std::shared_ptr<Calculator> client)
        : client_(client)
        , writer_(nullptr)
        , reader_(nullptr)
        , valid_user_input_(false)
    {
    }

    OperationStatus execute() override
    {
        if (auto client = client_.lock())
        {
            if (!reader_)
            {
                assert(writer_ == nullptr);
                reader_ = client->accumulator(writer_);

                if (!reader_ || !writer_)
                {
                    std::cerr << "Failed to create Client Reader/Writer" << std::endl;

                    return OperationStatus::ERROR;
                }

                InputFeedProcessor::print_help();
            }

            // Send a new value or close the input feed
            try
            {
                while(!valid_user_input_)
                {
                    auto input = InputFeedProcessor::get_input();

                    // Check the input status
                    switch (input.first)
                    {
                        // Valid number received
                        case InputFeedProcessor::Status::VALID_INPUT:
                            // Send the number to the server
                            writer_->write(input.second);
                            std::cout << "Input sent: " << input.second << std::endl;
                            valid_user_input_ = true;
                            break;

                        // Invalid input received
                        case InputFeedProcessor::Status::INVALID_INPUT:
                            std::cerr << "Invalid input. Please enter a valid number." << std::endl;
                            break;

                        // Input feed closed
                        case InputFeedProcessor::Status::FEED_CLOSED:
                            std::cout << "Input feed closed." << std::endl;
                            writer_->finish();
                            valid_user_input_ = true;
                            break;

                        default:
                            std::cerr << "Unknown input status." << std::endl;
                            break;
                    }
                }

                valid_user_input_ = false;

                // Read the next value from the output feed
                int32_t value;

                Duration_t timeout{1, 0}; // 1s

                if (reader_->read(value, timeout))
                {
                    std::cout << "Accumulated sum: " << value << std::endl;

                    // Output feed not closed yet
                    return OperationStatus::PENDING;
                }
                else
                {
                    std::cout << "Accumulator feed finished" << std::endl;

                    reader_.reset();
                    writer_.reset();

                    return OperationStatus::SUCCESS;
                }
            }
            catch (const RpcTimeoutException& e)
            {
                std::cerr << "Operation timed out " << e.what() << std::endl;

                return OperationStatus::TIMEOUT;
            }
            catch(const RpcException& e)
            {
                std::cerr << "RPC exception occurred: " << e.what() << std::endl;

                return OperationStatus::ERROR;
            }
        }
        else
        {
            throw std::runtime_error("Client reference expired");
        }
    }

protected:

    std::weak_ptr<Calculator> client_;
    std::shared_ptr<RpcClientWriter<int32_t>> writer_;
    std::shared_ptr<RpcClientReader<int32_t>> reader_;
    bool valid_user_input_;

};
//!--

//!--FILTER
class Filter : public Operation
{

public:

    Filter(
            std::shared_ptr<Calculator> client)
        : client_(client)
        , writer_(nullptr)
        , reader_(nullptr)
        , input_feed_closed_(false)
    {
    }

    OperationStatus execute() override
    {
        if (auto client = client_.lock())
        {
            // Parse the input data and send it to the server
            // until the input feed is closed
            try
            {
                while (!input_feed_closed_)
                {
                    if (!writer_)
                    {
                        assert(reader_ == nullptr);

                        // Filter the input feed by the selected filter kind
                        reader_ = client->filter(writer_, FilterKind::EVEN);

                        if (!reader_ || !writer_)
                        {
                            std::cerr << "Failed to create Client Reader/Writer" << std::endl;

                            return OperationStatus::ERROR;
                        }

                        InputFeedProcessor::print_help();
                    }

                    // Get the input from the user
                    auto input = InputFeedProcessor::get_input();

                    // Check the input status
                    switch (input.first)
                    {
                        // Valid number received
                        case InputFeedProcessor::Status::VALID_INPUT:
                            // Send the number to the server
                            writer_->write(input.second);
                            std::cout << "Input sent: " << input.second << std::endl;
                            break;

                        // Invalid input received
                        case InputFeedProcessor::Status::INVALID_INPUT:
                            std::cerr << "Invalid input. Please enter a valid number." << std::endl;
                            break;

                        // Input feed closed
                        case InputFeedProcessor::Status::FEED_CLOSED:
                            std::cout << "Input feed closed." << std::endl;
                            input_feed_closed_ = true;
                            writer_->finish();
                            break;

                        default:
                            std::cerr << "Unknown input status." << std::endl;
                            break;
                    }
                }

                // Get the next value from the output feed
                int32_t value;

                Duration_t timeout{1, 0}; // 1s

                if (reader_->read(value, timeout))
                {
                    std::cout << "Filtered sequence value: " << value << std::endl;

                    // Output feed not closed yet
                    return OperationStatus::PENDING;
                }
                else
                {
                    std::cout << "Filtered sequence feed finished" << std::endl;

                    reader_.reset();
                    writer_.reset();
                    input_feed_closed_ = false;

                    return OperationStatus::SUCCESS;
                }
            }
            catch (const RpcTimeoutException& e)
            {
                std::cerr << "Operation timed out " << e.what() << std::endl;

                return OperationStatus::TIMEOUT;
            }
            catch (const RpcException& e)
            {
                std::cerr << "Exception ocurred: " << e.what() << std::endl;

                return OperationStatus::ERROR;
            }
        }
        else
        {
            throw std::runtime_error("Client reference expired");
        }
    }

protected:

    std::weak_ptr<Calculator> client_;
    std::shared_ptr<RpcClientWriter<int32_t>> writer_;
    std::shared_ptr<RpcClientReader<int32_t>> reader_;
    bool input_feed_closed_;

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

    //! Send a request to the server and wait for the reply.
    //  Returns true if the reply was successfully received, false otherwise.
    bool send_request(const OperationType& operation)
    {
        // Set the operation to be executed
        set_operation(operation);

        // Execute the operation
//!--FEED_LOOP
        if (operation_)
        {
            OperationStatus status = operation_->execute();

            while (OperationStatus::PENDING == status)
            {
                // Wait before checking the next value (to see better the feed output)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                // Get the next value of the feed
                status = operation_->execute();
            }

            return (status == OperationStatus::SUCCESS);
        }
//!--

        return false;
    }

protected:

    //! Set the operation to be executed.

//!--SET_OPERATION
    void set_operation(
            const OperationType& operation)
    {
        switch (operation)
        {
            case OperationType::ADDITION:
                std::cout << "Configuring ADDITION operation with inputs: 5, 3" << std::endl;
                operation_ = std::unique_ptr<Operation>(new Addition(client_, 5, 3));
                break;
            case OperationType::SUBTRACTION:
                std::cout << "Configuring SUBTRACTION operation with inputs: 5, 3" << std::endl;
                operation_ = std::unique_ptr<Operation>(new Subtraction(client_, 5, 3));
                break;
            case OperationType::REPRESENTATION_LIMITS:
                std::cout << "Configuring REPRESENTATION_LIMITS operation" << std::endl;
                operation_ = std::unique_ptr<Operation>(new RepresentationLimits(client_));
                break;
            case OperationType::FIBONACCI:
                std::cout << "Configuring FIBONACCI operation with 5 results" << std::endl;
                operation_ = std::unique_ptr<Operation>(new FibonacciSeq(client_, 5));
                break;
            case OperationType::SUM_ALL:
                std::cout << "Configuring SUM_ALL operation" << std::endl;
                operation_ = std::unique_ptr<Operation>(new SumAll(client_));
                break;
            case OperationType::ACCUMULATOR:
                std::cout << "Configuring ACCUMULATOR operation" << std::endl;
                operation_ = std::unique_ptr<Operation>(new Accumulator(client_));
                break;
            case OperationType::FILTER:
                std::cout << "Configuring FILTER operation for even numbers" << std::endl;
                operation_ = std::unique_ptr<Operation>(new Filter(client_));
                break;
            default:
                throw std::runtime_error("Invalid operation type");
        }
    }
//!--

    std::shared_ptr<Calculator> client_;
    DomainParticipant* participant_;
    std::unique_ptr<Operation> operation_;
    std::string service_name_;

};

int main(
        int argc,
        char** argv)
{
    // Parse operation type from command line arguments
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <operation_type>" << std::endl;
        std::cerr << "Available operations: add, sub, rep, fib, sumall, acc, filter" << std::endl;
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
    else if (std::string(argv[1]) == "fib")
    {
        operation = OperationType::FIBONACCI;
    }
    else if (std::string(argv[1]) == "sumall")
    {
        operation = OperationType::SUM_ALL;
    }
    else if (std::string(argv[1]) == "acc")
    {
        operation = OperationType::ACCUMULATOR;
    }
    else if (std::string(argv[1]) == "filter")
    {
        operation = OperationType::FILTER;
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

    // Wait for matching
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
