// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

/*!
 * @file calculatorClient.cxx
 * Client implementation for interfaces
 *
 * This file was generated by the tool fastddsgen (version: 4.1.0).
 */

#include "calculatorClient.hpp"

#include <atomic>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/qos/RequesterQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/rpc/exceptions.hpp>
#include <fastdds/dds/rpc/interfaces.hpp>
#include <fastdds/dds/rpc/RequestInfo.hpp>
#include <fastdds/dds/rpc/Requester.hpp>
#include <fastdds/dds/rpc/Service.hpp>
#include <fastdds/dds/rpc/ServiceTypeSupport.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/rtps/common/Guid.hpp>
#include <fastdds/rtps/common/WriteParams.hpp>

#include "calculator.hpp"
#include "calculator_details.hpp"
#include "calculatorPubSubTypes.hpp"

namespace calculator_example {

//{ interface Calculator

namespace detail {

namespace fdds = eprosima::fastdds::dds;
namespace frpc = eprosima::fastdds::dds::rpc;
namespace frtps = eprosima::fastdds::rtps;

class CalculatorClient : public Calculator
{

    using RequestType = Calculator_Request;
    using ReplyType = Calculator_Reply;

public:

    CalculatorClient(
            fdds::DomainParticipant& part,
            const char* service_name,
            const fdds::RequesterQos& qos)
        : Calculator()
        , participant_(part)
    {
        // Register the service type support
        auto service_type = create_Calculator_service_type_support();
        auto ret = service_type.register_service_type(&participant_, "calculator_example::Calculator");
        if (ret != fdds::RETCODE_OK)
        {
            throw std::runtime_error("Error registering service type");
        }

        // Create the service
        service_ = participant_.create_service(service_name, "calculator_example::Calculator");
        if (nullptr == service_)
        {
            throw std::runtime_error("Error creating service");
        }

        // Create the requester
        requester_ = participant_.create_service_requester(service_, qos);
        if (nullptr == requester_)
        {
            throw std::runtime_error("Error creating requester");
        }

        // Start the processing thread
        start_thread();
    }

    ~CalculatorClient() override
    {
        // Stop the processing thread
        stop_thread();

        // Destroy the requester
        if (nullptr != requester_)
        {
            participant_.delete_service_requester(service_->get_service_name(), requester_);
            requester_ = nullptr;
        }

        // Destroy the service
        if (nullptr != service_)
        {
            participant_.delete_service(service_);
            service_ = nullptr;
        }
    }

private:

    void start_thread()
    {
        stop_thread_ = false;
        processing_thread_ = std::thread(&CalculatorClient::run, this);
    }

    void stop_thread()
    {
        stop_thread_ = true;
        if (processing_thread_.joinable())
        {
            processing_thread_.join();
        }
    }

    void run()
    {
        while (!stop_thread_)
        {
            // Wait for a reply
            if (requester_->get_requester_reader()->wait_for_unread_message({ 0, 100000000 }))
            {
                // Take and process the reply
                frpc::RequestInfo req_info;
                ReplyType reply;
                auto ret = requester_->take_reply(&reply, req_info);
                if (ret == fdds::RETCODE_OK)
                {
                    process_reply(reply, req_info);
                }
            }
        }
    }

    void process_reply(
            const ReplyType& reply,
            const frpc::RequestInfo& req_info)
    {
        auto sample_id = req_info.related_sample_identity;
        {
            std::lock_guard<std::mutex> _(mtx_);
            auto it = pending_results_.find(sample_id);
            if (it != pending_results_.end())
            {
                bool should_erase = false;
                it->second->process_reply(reply, req_info, should_erase);
                if (should_erase)
                {
                    pending_results_.erase(it);
                }
            }
        }
    }

    struct IReplyProcessor
    {
        frpc::RequestInfo info;

        virtual ~IReplyProcessor() = default;

        virtual void process_reply(
                const ReplyType& reply,
                const frpc::RequestInfo& req_info,
                bool& should_remove) = 0;

    };

    fdds::DomainParticipant& participant_;
    frpc::Service* service_ = nullptr;
    frpc::Requester* requester_ = nullptr;
    std::atomic<bool> stop_thread_{false};
    std::thread processing_thread_;
    std::mutex mtx_;
    std::map<frtps::SampleIdentity, std::shared_ptr<IReplyProcessor>> pending_results_;

    //{ request helpers

    template<typename T, typename TResult>
    std::future<TResult> send_request_with_promise(
            const RequestType& request,
            std::shared_ptr<T> result,
            std::promise<TResult>& promise)
    {
        result->info.related_sample_identity.writer_guid(requester_->get_requester_reader()->guid());
        if (fdds::RETCODE_OK == requester_->send_request((void*)&request, result->info))
        {
            std::lock_guard<std::mutex> _ (mtx_);
            pending_results_[result->info.related_sample_identity] = result;
        }
        else
        {
            promise.set_exception(
                std::make_exception_ptr(frpc::RpcBrokenPipeException(false)));
        }

        return promise.get_future();
    }

    template<typename T>
    std::shared_ptr<T> send_request_with_reader(
            const RequestType& request,
            std::shared_ptr<T> result)
    {
        result->info.related_sample_identity.writer_guid(requester_->get_requester_reader()->guid());
        if (fdds::RETCODE_OK == requester_->send_request((void*)&request, result->info))
        {
            std::lock_guard<std::mutex> _ (mtx_);
            pending_results_[result->info.related_sample_identity] = result;
        }
        else
        {
            result->set_exception(
                std::make_exception_ptr(frpc::RpcBrokenPipeException(false)));
        }

        return result;
    }

    //} request helpers

    //{ reply helpers

    template<typename T>
    static void set_invalid_reply(
            T& exception_receiver)
    {
        exception_receiver.set_exception(
            std::make_exception_ptr(frpc::RemoteInvalidArgumentError("An invalid reply was received")));
    }

    template<typename T>
    static void set_remote_exception(
            T& exception_receiver,
            const frpc::RemoteExceptionCode_t& exception)
    {
        switch (exception)
        {
            case frpc::RemoteExceptionCode_t::REMOTE_EX_OK:
                set_invalid_reply(exception_receiver);
                break;
            case frpc::RemoteExceptionCode_t::REMOTE_EX_UNSUPPORTED:
                exception_receiver.set_exception(std::make_exception_ptr(frpc::RemoteUnsupportedError()));
                break;
            case frpc::RemoteExceptionCode_t::REMOTE_EX_INVALID_ARGUMENT:
                exception_receiver.set_exception(std::make_exception_ptr(frpc::RemoteInvalidArgumentError()));
                break;
            case frpc::RemoteExceptionCode_t::REMOTE_EX_OUT_OF_RESOURCES:
                exception_receiver.set_exception(std::make_exception_ptr(frpc::RemoteOutOfResourcesError()));
                break;
            case frpc::RemoteExceptionCode_t::REMOTE_EX_UNKNOWN_OPERATION:
                exception_receiver.set_exception(std::make_exception_ptr(frpc::RemoteUnknownOperationError()));
                break;
            default: // REMOTE_EX_UNKNOWN_EXCEPTION
                exception_receiver.set_exception(std::make_exception_ptr(frpc::RemoteUnknownExceptionError()));
                break;
        }
    }

    static size_t count_reply_fields(
            const ReplyType& reply)
    {
        size_t n_fields = 0;
        n_fields += reply.representation_limits.has_value() ? 1 : 0;
        n_fields += reply.addition.has_value() ? 1 : 0;
        n_fields += reply.subtraction.has_value() ? 1 : 0;
        n_fields += reply.remoteEx.has_value() ? 1 : 0;
        return n_fields;
    }

    template<typename T>
    static bool validate_reply(
            std::promise<T>& promise,
            const ReplyType& reply)
    {
        // Check if the reply has one and only one field set
        size_t n_fields = count_reply_fields(reply);
        if (n_fields != 1u)
        {
            set_invalid_reply(promise);
            return false;
        }

        return true;
    }

    struct IExceptionHolder
    {
        virtual ~IExceptionHolder() = default;

        virtual void set_exception(
                std::exception_ptr exception) = 0;
    };

    static bool validate_reply(
            IExceptionHolder& reader,
            const ReplyType& reply)
    {
        // Check if the reply has one and only one field set
        size_t n_fields = count_reply_fields(reply);
        if (n_fields != 1u)
        {
            set_invalid_reply(reader);
            return false;
        }
        return true;
    }

    //} reply helpers

    //{ operation representation_limits

public:

    eprosima::fastdds::dds::rpc::RpcFuture<calculator_example::detail::Calculator_representation_limits_Out> representation_limits(
) override
    {
        // Create a promise to hold the result
        auto result = std::make_shared<representation_limits_promise>();

        // Create and send the request
        RequestType request;
        request.representation_limits = calculator_example::detail::Calculator_representation_limits_In{};
        return send_request_with_promise(request, result, result->promise);
    }

private:

    struct representation_limits_promise : public IReplyProcessor
    {
        std::promise<calculator_example::detail::Calculator_representation_limits_Out> promise;

        void process_reply(
                const ReplyType& reply,
                const frpc::RequestInfo& req_info,
                bool& should_remove) override
        {
            should_remove = false;
            if (req_info.related_sample_identity != info.related_sample_identity)
            {
                return;
            }

            should_remove = true;
            if (!validate_reply(promise, reply))
            {
                return;
            }

            if (reply.remoteEx.has_value())
            {
                set_remote_exception(promise, reply.remoteEx.value());
                return;
            }

            if (reply.representation_limits.has_value())
            {
                const auto& result = reply.representation_limits.value();
                if (result.result.has_value())
                {
                    const auto& out = result.result.value();
                    promise.set_value(out);
                    return;
                }
            }

            // If we reach this point, the reply is for another operation
            set_invalid_reply(promise);
        }

    };



    //} operation representation_limits
 
    //{ operation addition

public:

    eprosima::fastdds::dds::rpc::RpcFuture<int32_t> addition(
            /*in*/ int32_t value1,
            /*in*/ int32_t value2) override
    {
        // Create a promise to hold the result
        auto result = std::make_shared<addition_promise>();

        // Create and send the request
        RequestType request;
        request.addition = calculator_example::detail::Calculator_addition_In{};
        request.addition->value1 = value1;
        request.addition->value2 = value2;

        return send_request_with_promise(request, result, result->promise);
    }

private:

    struct addition_promise : public IReplyProcessor
    {
        std::promise<int32_t> promise;

        void process_reply(
                const ReplyType& reply,
                const frpc::RequestInfo& req_info,
                bool& should_remove) override
        {
            should_remove = false;
            if (req_info.related_sample_identity != info.related_sample_identity)
            {
                return;
            }

            should_remove = true;
            if (!validate_reply(promise, reply))
            {
                return;
            }

            if (reply.remoteEx.has_value())
            {
                set_remote_exception(promise, reply.remoteEx.value());
                return;
            }

            if (reply.addition.has_value())
            {
                const auto& result = reply.addition.value();
                if (result.result.has_value())
                {
                    const auto& out = result.result.value();
                    promise.set_value(out.return_);
                    return;
                }
                if (result.calculator_example_OverflowException_ex.has_value())
                {
                    promise.set_exception(
                        std::make_exception_ptr(result.calculator_example_OverflowException_ex.value()));
                    return;
                }
            }

            // If we reach this point, the reply is for another operation
            set_invalid_reply(promise);
        }

    };



    //} operation addition
 
    //{ operation subtraction

public:

    eprosima::fastdds::dds::rpc::RpcFuture<int32_t> subtraction(
            /*in*/ int32_t value1,
            /*in*/ int32_t value2) override
    {
        // Create a promise to hold the result
        auto result = std::make_shared<subtraction_promise>();

        // Create and send the request
        RequestType request;
        request.subtraction = calculator_example::detail::Calculator_subtraction_In{};
        request.subtraction->value1 = value1;
        request.subtraction->value2 = value2;

        return send_request_with_promise(request, result, result->promise);
    }

private:

    struct subtraction_promise : public IReplyProcessor
    {
        std::promise<int32_t> promise;

        void process_reply(
                const ReplyType& reply,
                const frpc::RequestInfo& req_info,
                bool& should_remove) override
        {
            should_remove = false;
            if (req_info.related_sample_identity != info.related_sample_identity)
            {
                return;
            }

            should_remove = true;
            if (!validate_reply(promise, reply))
            {
                return;
            }

            if (reply.remoteEx.has_value())
            {
                set_remote_exception(promise, reply.remoteEx.value());
                return;
            }

            if (reply.subtraction.has_value())
            {
                const auto& result = reply.subtraction.value();
                if (result.result.has_value())
                {
                    const auto& out = result.result.value();
                    promise.set_value(out.return_);
                    return;
                }
                if (result.calculator_example_OverflowException_ex.has_value())
                {
                    promise.set_exception(
                        std::make_exception_ptr(result.calculator_example_OverflowException_ex.value()));
                    return;
                }
            }

            // If we reach this point, the reply is for another operation
            set_invalid_reply(promise);
        }

    };



    //} operation subtraction
 

};

}  // namespace detail

std::shared_ptr<Calculator> create_CalculatorClient(
        eprosima::fastdds::dds::DomainParticipant& part,
        const char* service_name,
        const eprosima::fastdds::dds::RequesterQos& qos)
{
    return std::make_shared<detail::CalculatorClient>(part, service_name, qos);
}

//} interface CalculatorClient


} // namespace calculator_example
