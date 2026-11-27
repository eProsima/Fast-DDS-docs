<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="fastddsgen-rpc-calculator-feed-app-intro"></a>

<a id="building-a-rpc-client-server-application-with-data-streaming"></a>

# Building a RPC Client/Server application with data streaming

*Fast DDS-Gen* supports the generation of source code for interfaces that specify
data streaming operations using the `@feed` builtin annotation.

This section extends the previously discussed example of a calculator service
(see [Building a RPC Client/Server application](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-app-intro)) to include the following data streaming operations:

* **fibonacci_seq**: Returns a feed of results with the n_results first elements of the Fibonacci sequence.
* **sum_all**: Returns the sum of all the received values through a feed when it is closed.
* **accumulator**: Returns a feed of results with the sum of all received values.
* **filter**: Returns a feed of results with the received values that match an input filter.

The user-written source code of this example is available in the following files:
[CalculatorServer.cpp](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerFeed/src/CalculatorServer.cpp),
[CalculatorClient.cpp](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerFeed/src/CalculatorClient.cpp),
[ServerImplementation.hpp](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerFeed/src/ServerImplementation.hpp) and
[CMakeLists.txt](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerFeed/CMakeLists.txt).
The interface support code is not included: it is generated with *Fast DDS-Gen* from the IDL file, as described
below.

<a id="background"></a>

## Background

As this example is based on the basic calculator example, reading the
[basic example](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-app-intro) is recommended to understand
how the application is structured and how the code is generated.
In this section, we only will discuss in depth the source code
related to the new operations and the main differences with the generated source code of the basic example.
In any case, all the source code required to build the application is also provided here.

Please also check the [Prerequisites](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-app-prerequisites) and
[Dependencies](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-app-dependencies) sections of the basic example
for more information about the requirements and dependencies of the RPC client/server applications.

<a id="fastddsgen-rpc-calculator-feed-app-workspace"></a>

<a id="create-the-application-workspace"></a>

## Create the application workspace

First, create a directory named *workspace_CalculatorFeed*, which will represent the workspace of the application.

The workspace will have the following structure at the end of the project.
The files `build/client` and `build/server` corresponds to
the generated *Fast DDS* client and server applications, respectively:

```shell-session
.
├── build
│   ├── feed_client
│   ├── feed_server
│   ├── ...
├── CMakeLists.txt
└── src
    ├── CalculatorClient.cpp
    ├── CalculatorServer.cpp
    ├── ServerImplementation.hpp
    └── types
        ├── calculatorCdrAux.hpp
        ├── calculatorCdrAux.ipp
        ├── calculatorClient.cxx
        ├── calculatorClient.hpp
        ├── calculator_details.hpp
        ├── calculator.hpp
        ├── calculator.idl
        ├── calculatorPubSubTypes.cxx
        ├── calculatorPubSubTypes.hpp
        ├── calculatorServer.cxx
        ├── calculatorServer.hpp
        ├── calculatorServerImpl.hpp
        ├── calculatorTypeObjectSupport.cxx
        └── calculatorTypeObjectSupport.hpp
```

<a id="configure-the-cmake-project"></a>

## Configure the CMake project

We will use the CMake tool to manage the building of the project.
With your preferred text editor, create a new file called `CMakeLists.txt`
and copy and paste the following code snippet.
Save this file in the root directory of your workspace. If you have followed these steps, it should
be workspace_CalculatorFeed.

```cmake
cmake_minimum_required(VERSION 3.20)

project(RpcClientServerFeed)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr 2 REQUIRED)
endif()

if(NOT fastdds_FOUND)
    find_package(fastdds 3.2.0 REQUIRED)
endif()

# Set C++11
include(CheckCXXCompilerFlag)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_COMPILER_IS_CLANG OR
        CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    check_cxx_compiler_flag(-std=c++11 SUPPORTS_CXX11)
    if(SUPPORTS_CXX11)
        add_compile_options(-std=c++11)
    else()
        message(FATAL_ERROR "Compiler doesn't support C++11")
    endif()
endif()

message(STATUS "Configuring client server feed example...")
file(GLOB CLIENT_SERVER_FEED_TYPES_SOURCES_CXX "src/types/*.cxx")
file(GLOB CLIENT_SERVER_FEED_TYPES_SOURCES_IPP "src/types/*.ipp")
```

In each section we will complete this file to include the specific generated files.

<a id="build-the-client-server-interface"></a>

## Build the client/server interface

The operations of the calculator service should be specified in an IDL file using an interface and
following the syntax explained in [Defining an IDL interface](../interfaces/interfaces.md#fastddsgen-interfaces-definition).

In the workspace directory, run the following commands:

```bash
mkdir src && cd src
mkdir types && cd types
touch calculator.idl
cd ../..
```

We have created a separated `types` subdirectory inside the workspace source directory to separate the source code
generated by *Fast DDS-Gen* from the rest of the application code. Now open the *calculator.idl* file
with a text editor and copy the following content inside it:

```omg-idl
module calculator_example
{
    // This exception will be thrown when an operation result cannot be represented in a long
    exception OverflowException
    {
    };

        // For the filter operation
    enum FilterKind
    {
        EVEN,  // return even numbers
        ODD,   // return odd numbers
        PRIME  // return positive prime numbers
    };

    interface Calculator
    {
        // Returns the minimum and maximum representable values
        void representation_limits(out long min_value, out long max_value);

        // Returns the result of value1 + value2
        long addition(in long value1, in long value2) raises (OverflowException);

        // Returns the result of value1 - value2
        long subtraction(in long value1, in long value2) raises (OverflowException);

        // Returns a feed of results with the n_results first elements of the Fibonacci sequence
        // E.g. for an input of 5, returns a feed with {1, 1, 2, 3, 5}
        @feed long fibonacci_seq(in unsigned long n_results) raises (OverflowException);

        // Waits for an input feed to finish and returns the sum of all the received values
        // E.g. for an input of {1, 2, 3, 4, 5} returns 15
        long sum_all(@feed in long value) raises (OverflowException);

        // Returns a feed of results with the sum of all received values
        // E.g. for an input of {1, 2, 3, 4, 5}, returns a feed with {1, 3, 6, 10, 15}
        @feed long accumulator(@feed in long value) raises (OverflowException);

        // Returns a feed of results with the received values that match the input filter kind
        @feed long filter(@feed in long value, in FilterKind filter_kind);
    };
};
```

Similarly to the basic example, operations that imply some kind of arithmetic operation on the data
can raise an `OverflowException` in case of numbers exceeding the limits of the `long` type.

Note that the new operations are defined with the `@feed` annotation,
and it is specified on a return type or an *in* parameter.
On one hand, adding `@feed` to the return type means that the client
expects multiple results from the server (more specifically,
an unbounded number of results before the server closes the feed), *i.e*, multiple replies to the same request.

On the other hand, adding `@feed` to an *in* parameter means that the client will send multiple data values
associated to the same operation parameter, and that all these values should be used by the server to compute the result
of the operation. Thus, the server waits to receive all the values from the client
(*i.e:* waits to the feed to be closed by the client) before computing the result of the operation.

<a id="generate-the-fast-dds-source-code-from-the-idl-file"></a>

## Generate the Fast DDS source code from the IDL file

Source code can be generated in the *workspace_CalculatorFeed/types* directory using the *Fast DDS-Gen* tool
in the same way as in the [basic example](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-app-code-generation).

<a id="files-description"></a>

### Files description

The files generated by the *Fast DDS-Gen* tool are similar to the ones generated in the
[basic example](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-app-code-generation).

We will only discuss the changes in each file by the addition of the new operations:

<a id="calculator"></a>

#### calculator

Note that operations with a `@feed` annotated return type return an object of type
`RpcClientReader<return_type>`, instead of an `RpcFuture<return_type>` object.

Due to the fact that, when calling this kind of operations, client does not know the number of results
that will be received, it uses an `RpcClientReader` object to read results of type `<return_type>`,
until the feed is closed by the server, cancelled by the client using `RpcClientReader::cancel()` method
(*i.e:* the client decides to stop reading results) or an error occurs.

Similarly, input `<param_type>` parameters with a `@feed` annotation have a type of
`RpcClientWriter<param_type>`. `RpcClientWriter` objects allow the client to send multiple values of type
`<param_type>` to the server using the `RpcClientWriter::write()` method or notify the server that the feed is
finished using the `RpcClientWriter::finish()` method.

For more information about input and output feeds, see [Data Streaming interfaces](../../fastdds/rpc_dds/data_streaming/rpc_data_streaming_intro.md#rpc-data-streaming-interfaces).

<a id="calculator-details"></a>

#### calculator_details

Contains additional types used to transport the input and output feed values between client and server.
These types are used internally by the generated client and server code, and must not be used directly by the
application.

<a id="writing-the-application-source-code"></a>

## Writing the application source code

Now that the interface source code has been generated, the next step is to generate the application source code.
All the following files should be created in the *workspace_CalculatorFeed/src* directory.

<a id="server-application"></a>

### Server application

The server application that we will create is exactly the same as the one created in the basic example.
Thus, copy the code provided in [Server application](../rpc_calculator_basic_app/intro.md#fastddsgen-rpc-calculator-basic-server-application)
into the `CalculatorServer.cpp` file.

Additionally, create a `ServerImplementation.hpp` file with the implementation of the server-side operations:

```cpp
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
```

#### WARNING
The user implementation of the server-side operations should be placed in a separate file
and inherit from the `CalculatorServerImplementation` struct to avoid being overridden
by *Fast DDS-Gen* when IDL interface’s source code is regenerated. User should use this derived
class when creates the server instance.

<a id="client-application"></a>

### Client application

The client application extends the functionality of the basic example by adding the new operations
defined in the IDL file. Create a `CalculatorClient.cpp` file with this
[content](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerFeed/src/CalculatorClient.cpp):

<a id="examining-the-code"></a>

#### Examining the code

The `CalculatorClient.cpp` file extends the code of the basic example by adding the new operations
defined in the IDL file, following the same inheritance schema.

Notice the following facts:

* The operations that expect an output feed (*i.e* `FibonacciSeq`, `Accumulator` and `Filter` operations)
  store internally an `RpcClientReader` reference. As the client expects multiple replies from the server for
  the same request, it will use the `RpcClientReader::read()` method to read the results until the output feed
  is closed by the server.
* The operations that expect an input feed for some of their parameters
  (*i.e* `SumAll`, `Accumulator` and `Filter` operations) store internally an
  `RpcClientWriter` reference. As the client will send multiple values to the server, it will use the
  `RpcClientWriter::write()` method to send the values and the `RpcClientWriter::finish()` method to notify
  the server that the input feed is finished.

For input feed operations, a simple `InputFeedProcessor` class is used to parse the input user data from terminal.
It allows the user to send a new number or close the input feed by accepting the dialog with empty data:

```cpp
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
```

For output feed operations, each time that `Operation::execute()` is called, the client will process the data
of only one reply, so multiple calls will be required to read all the results. To address this, the
`Operation::execute()` method will return an enum `OperationStatus::PENDING` when a new output feed value
is read, indicating that the output feed has not been closed yet and that client should process more data.
When the output feed is closed, the `Operation::execute()` method will return an enum
`OperationStatus::SUCCESS`, indicating that the operation has successfully read each of the output feed values:

```cpp
enum class OperationStatus
{
    SUCCESS,
    TIMEOUT,
    ERROR,
    PENDING
};
```

A more detailed description of each operation execution is provided below:

* `FibonacciSeq` stores the number of requested numbers using using a `n_results_` member.
  The request is sent by calling the `client_->fibonacci_seq()` method, which returns an
  `RpcClientReader` object. The client will read the results using the `RpcClientReader::read()` method
  in each `FibonacciSeq::execute()` call, until the output feed is closed by the server.

```cpp
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
```

* In the first call of the `SumAll::execute()` method, the client will create an
  `RpcClientWriter` object by calling the `client_->sum_all()` method and passing it as an output parameter.
  Then, the input feed is parsed from the user terminal using the `InputFeedProcessor` class, sending each
  input data value to the server using the `RpcClientWriter::write()` method. When the user finish the input feed
  (by accepting the terminal dialog with an empty value),
  the client closes the input feed using the `RpcClientWriter::finish()`
  and waits for the reply. When the reply is received,
  the result is stored in `result_` member and printed on the screen.

```cpp
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
```

* For the `Accumulator` operation, both `RpcClientReader` and `RpcClientWriter` objects are created
  by calling the `client_->accumulator()` method. The client will read the results using the
  `RpcClientReader::read()` method in each `Accumulator::execute()` call, until the output feed is closed
  by the server. The input feed is parsed from the user terminal using the `InputFeedProcessor` class,
  sending each input data value to the server using the `RpcClientWriter::write()` method.
  When the user finish the input feed (by accepting the terminal dialog with an empty value),
  the client closes the input feed using the `RpcClientWriter::finish()`
  and waits for the reply.
  Each time that `Accumulator::execute()` is called, the client sends a new input value to the server
  and waits for the accumulated sum result, until the input and output feeds are closed.

```cpp
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
```

* For the `Filter` operation, both `RpcClientReader` and `RpcClientWriter` objects are created
  by calling the `client_->filter()` method. First, the client will send to the server all the input feed
  data values and the filter kind using the `RpcClientWriter::write()` method.
  Then, each result is processed in a `Filter::execute()` call, until the output feed is closed by the server.
  To simplify the input parsing, the filter is fixed to be `FilterKind::EVEN`, *i.e* the input feed is filtered
  to only return even numbers:

```cpp
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
```

`ClientApp::set_operation()` has also been extended to include the new operations:

```cpp
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
```

Finally, a minimal change is required in the `send_request()` method to handle the feed operations:

```cpp
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
```

Note that this change does not modify the behavior of the non-feed operations, as they never return
`OperationStatus::PENDING` status.

<a id="update-the-cmakelists-txt-file"></a>

### Update the *CMakeLists.txt* file

Before building the application, we need to update the *CMakeLists.txt* file of the workspace
to add the executable and the required libraries. The following code should be added at the end
of the file:

```cmake
add_executable(feed_client src/CalculatorClient.cpp
    ${CLIENT_SERVER_FEED_TYPES_SOURCES_CXX}
    ${CLIENT_SERVER_FEED_TYPES_SOURCES_IPP}
)

target_link_libraries(feed_client fastdds fastcdr)

add_executable(feed_server src/CalculatorServer.cpp
    ${CLIENT_SERVER_FEED_TYPES_SOURCES_CXX}
    ${CLIENT_SERVER_FEED_TYPES_SOURCES_IPP}
)

target_link_libraries(feed_server fastdds fastcdr)
```

<a id="build-the-application"></a>

## Build the application

Now that all the files are created, the application can be built.
To do so, open a terminal in the *workspace_CalculatorFeed* directory and run the following commands:

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

The generated executable will be located in the *build* subdirectory.

<a id="run-the-application"></a>

## Run the application

To test the application, open two terminals in the *workspace_CalculatorFeed* directory
and execute the following commands:

* In the first terminal, run the server application:

```bash
./build/feed_server
```

* In the second terminal, run the client application and specify the operation to be performed.
  For example, to get the first five numbers of the Fibonacci sequence, run the following command:

```bash
./build/feed_client fib
```

You should see the result of the operation printed on the screen:

```shell-session
Attempting to send request, attempt 1/10
Configuring FIBONACCI operation with 5 results
Fibonacci sequence value: 1
Fibonacci sequence value: 1
Fibonacci sequence value: 2
Fibonacci sequence value: 3
Fibonacci sequence value: 5
Fibonacci sequence feed finished
Request sent successfully
```

The output of the rest operations should be similar to the following:

* SumAll:

```shell-session
./build/feed_client sumall

Configuring SUM_ALL operation
Input feed help:
  - Enter a number to process it.
  - Press Enter without typing anything to close the input feed.
2
Input sent: 2
3
Input sent: 3
-32
Input sent: -32
-25
Input sent: -25
-12
Input sent: -12

Input feed closed.
Sum result: -64
Request sent successfully
```

* Accumulator:

```shell-session
./build/feed_client acc

Configuring ACCUMULATOR operation
Input feed help:
  - Enter a number to process it.
  - Press Enter without typing anything to close the input feed.
16
Input sent: 16
Accumulated sum: 16
32
Input sent: 32
Accumulated sum: 48
-13
Input sent: -13
Accumulated sum: 35
-22
Input sent: -22
Accumulated sum: 13
-1
Input sent: -1
Accumulated sum: 12

Input feed closed.
Accumulator feed finished
Request sent successfully
```

* Filter:

```shell-session
./build/feed_client filter

Configuring FILTER operation for even numbers
Input feed help:
  - Enter a number to process it.
  - Press Enter without typing anything to close the input feed.
2
Input sent: 2
11
Input sent: 11
32
Input sent: 32
15
Input sent: 15
-23
Input sent: -23
-15
Input sent: -15
4
Input sent: 4

Input feed closed.
Filtered sequence value: 2
Filtered sequence value: 32
Filtered sequence value: 4
Filtered sequence feed finished
Request sent successfully
```
