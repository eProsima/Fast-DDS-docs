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
 * @file ServerImplementation.hpp
 * File containing the implementation of the server operations
 */

#ifndef EXAMPLES_CPP_RPC_CLIENT_SERVER_FEED__SERVER_IMPLEMENTATION_HPP
#define EXAMPLES_CPP_RPC_CLIENT_SERVER_FEED__SERVER_IMPLEMENTATION_HPP

#include "types/calculatorServerImpl.hpp"
#include "types/calculator.hpp"
#include "types/calculatorServer.hpp"


struct ServerImplementation :
    public calculator_example::CalculatorServerImplementation
{

    calculator_example::detail::Calculator_representation_limits_Out representation_limits(
            const eprosima::fastdds::dds::rpc::RpcRequest& info) override
    {
        static_cast<void>(info);

        calculator_example::detail::Calculator_representation_limits_Out limits;
        limits.min_value = std::numeric_limits<int32_t>::min();
        limits.max_value = std::numeric_limits<int32_t>::max();

        return limits;
    }

    int32_t addition(
            const eprosima::fastdds::dds::rpc::RpcRequest& info,
            /*in*/ int32_t value1,
            /*in*/ int32_t value2) override
    {
        static_cast<void>(info);

        int32_t result = value1 + value2;
        bool negative_1 = value1 < 0;
        bool negative_2 = value2 < 0;
        bool negative_result = result < 0;

        if ((negative_1 == negative_2) && (negative_result != negative_1))
        {
            throw calculator_example::OverflowException();
        }

        return result;
    }

    int32_t subtraction(
            const eprosima::fastdds::dds::rpc::RpcRequest& info,
            /*in*/ int32_t value1,
            /*in*/ int32_t value2) override
    {
        static_cast<void>(info);

        int32_t result = value1 - value2;
        bool negative_1 = value1 < 0;
        bool negative_2 = value2 < 0;
        bool negative_result = result < 0;

        if ((negative_1 != negative_2) && (negative_result != negative_1))
        {
            throw calculator_example::OverflowException();
        }

        return result;
    }

    void fibonacci_seq(
            const eprosima::fastdds::dds::rpc::RpcRequest& info,
            /*in*/ uint32_t n_results,
            /*result*/ eprosima::fastdds::dds::rpc::RpcServerWriter<int32_t>& result_writer) override
    {
        static_cast<void>(info);

        int32_t a = 1;
        int32_t b = 1;
        int32_t c = 0;

        for (uint32_t i = 0; i < n_results; ++i)
        {
            if (a < 0)
            {
                throw calculator_example::OverflowException(
                    "Overflow in Fibonacci sequence. "
                    "The result is too large to be represented.");
            }

            result_writer.write(a);
            c = a + b;
            a = b;
            b = c;
        }
    }

    int32_t sum_all(
            const eprosima::fastdds::dds::rpc::RpcRequest& info,
            /*in*/ eprosima::fastdds::dds::rpc::RpcServerReader<int32_t>& value) override
    {
        static_cast<void>(info);

        int32_t sum_all_result = 0;

        try
        {
            int32_t value_to_add = 0;
            while (value.read(value_to_add))
            {
                int32_t new_sum = sum_all_result + value_to_add;
                bool current_negative = sum_all_result < 0;
                bool new_negative = value_to_add < 0;
                bool result_negative = new_sum < 0;
                if ((current_negative == new_negative) && (result_negative != current_negative))
                {
                    throw calculator_example::OverflowException();
                }
                sum_all_result = new_sum;
            }
        }
        catch (const eprosima::fastdds::dds::rpc::RpcFeedCancelledException& ex)
        {
            static_cast<void>(ex);
            // Feed was cancelled, do nothing and return the current sum
        }

        return sum_all_result;
    }

    void accumulator(
            const eprosima::fastdds::dds::rpc::RpcRequest& info,
            /*in*/ eprosima::fastdds::dds::rpc::RpcServerReader<int32_t>& value,
            /*result*/ eprosima::fastdds::dds::rpc::RpcServerWriter<int32_t>& result_writer) override
    {
        static_cast<void>(info);

        int32_t current_sum = 0;

        try
        {
            int32_t value_to_add = 0;
            while (value.read(value_to_add))
            {
                int32_t new_sum = current_sum + value_to_add;
                bool current_negative = current_sum < 0;
                bool new_negative = value_to_add < 0;
                bool result_negative = new_sum < 0;
                if ((current_negative == new_negative) && (result_negative != current_negative))
                {
                    throw calculator_example::OverflowException();
                }
                current_sum = new_sum;
                result_writer.write(current_sum);
            }
        }
        catch (const eprosima::fastdds::dds::rpc::RpcFeedCancelledException& ex)
        {
            static_cast<void>(ex);
            // Feed was cancelled, do nothing
        }
    }

    void filter(
            const eprosima::fastdds::dds::rpc::RpcRequest& info,
            /*in*/ eprosima::fastdds::dds::rpc::RpcServerReader<int32_t>& value,
            /*in*/ calculator_example::FilterKind filter_kind,
            /*result*/ eprosima::fastdds::dds::rpc::RpcServerWriter<int32_t>& result_writer) override
    {
        static_cast<void>(info);

        try
        {
            int32_t value_to_filter = 0;
            while (value.read(value_to_filter))
            {
                switch (filter_kind)
                {
                    case calculator_example::FilterKind::EVEN:
                        if (value_to_filter % 2 == 0)
                        {
                            result_writer.write(value_to_filter);
                        }
                        break;

                    case calculator_example::FilterKind::ODD:
                        if (value_to_filter % 2 != 0)
                        {
                            result_writer.write(value_to_filter);
                        }
                        break;

                    case calculator_example::FilterKind::PRIME:
                        throw eprosima::fastdds::dds::rpc::RemoteUnsupportedError(
                            "Prime filter is not implemented in this example.");

                    default:
                        // Invalid filter kind
                        throw eprosima::fastdds::dds::rpc::RemoteInvalidArgumentError();
                }
            }
        }
        catch (const eprosima::fastdds::dds::rpc::RpcFeedCancelledException& ex)
        {
            static_cast<void>(ex);
            // Feed was cancelled, do nothing
        }
    }

};

//} interface Calculator

#endif  // EXAMPLES_CPP_RPC_CLIENT_SERVER_FEED__SERVER_IMPLEMENTATION_HPP
