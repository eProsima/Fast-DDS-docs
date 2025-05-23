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

/*!
 * @file ServerImplementation.hpp
 * File containing the implementation of the server operations
 */

#ifndef EXAMPLES_CPP_RPC_CLIENT_SERVER_BASIC__SERVER_IMPLEMENTATION_HPP
#define EXAMPLES_CPP_RPC_CLIENT_SERVER_BASIC__SERVER_IMPLEMENTATION_HPP

#include "types/calculatorServerImpl.hpp"
#include "types/calculator.hpp"
#include "types/calculatorServer.hpp"


struct ServerImplementation :
    public calculator_example::CalculatorServerImplementation
{

    void representation_limits(
            const calculator_example::CalculatorServer_ClientContext& info,
            /*out*/ int32_t& min_value,
            /*out*/ int32_t& max_value) override
    {
        static_cast<void>(info);

        min_value = std::numeric_limits<int32_t>::min();
        max_value = std::numeric_limits<int32_t>::max();
    }

    int32_t addition(
            const calculator_example::CalculatorServer_ClientContext& info,
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
            const calculator_example::CalculatorServer_ClientContext& info,
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

};

#endif  // EXAMPLES_CPP_RPC_CLIENT_SERVER_BASIC__SERVER_IMPLEMENTATION_HPP
