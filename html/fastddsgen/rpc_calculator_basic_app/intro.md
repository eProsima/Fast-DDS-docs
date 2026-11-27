<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="fastddsgen-rpc-calculator-basic-app-intro"></a>

<a id="building-a-rpc-client-server-application"></a>

# Building a RPC Client/Server application

*Fast DDS-Gen* can be used to generate the source code required to
build a *Remote Procedure Calls* (RPC) Client/Server application from an IDL file.

This section provides an overview of the steps required to build a *Fast DDS* RPC application from scratch,
following the [RPC over DDS specification](https://www.omg.org/spec/DDS-RPC/1.0/PDF), using the *Fast DDS-Gen* tool.

The example consists of a simple CLI tool for a calculator service that allows the client
to call asynchronously the following operations:

* **addition**: Adds two 32-bit integers and returns the result.
* **subtraction**: Subtracts two 32-bit integers and returns the result.
* **representation_limits**: Returns the minimum and maximum representable values for a 32-bit integer.

The user-written source code of this example is available in the following files:
[CalculatorServer.cpp](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerBasic/src/CalculatorServer.cpp),
[CalculatorClient.cpp](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerBasic/src/CalculatorClient.cpp),
[ServerImplementation.hpp](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerBasic/src/ServerImplementation.hpp) and
[CMakeLists.txt](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerBasic/CMakeLists.txt).
The interface support code is not included: it is generated with *Fast DDS-Gen* from the IDL file, as described
below.

<a id="background"></a>

## Background

*eProsima Fast DDS-Gen* is a Java application that generates eProsima Fast DDS source code using
the data types and interfaces defined in an IDL (Interface Definition Language) file.
This generated source code can be used in any Fast DDS application in order to define the data type of a topic,
or in a RPC application (see [RPC over DDS](../../fastdds/rpc_dds/rpc_dds_intro.md#rpc-dds-intro)) to define the service and its operations.

In a RPC over DDS application (see [RPC over DDS](../../fastdds/rpc_dds/rpc_dds_intro.md#rpc-dds-intro)), user provides
through the IDL file the common interfaces that will be used by the client and server applications.
An interface contains the collection of operations that can be invoked by the client on the server.
Please refer to [IDL interfaces introduction](../interfaces/introduction.md#fastddsgen-interfaces-introduction) for more information.

Each interface defines a service, and *eProsima Fast DDS-Gen* generates the source code required to communicate
a client with a server through that service on top of the
*eProsima Fast DDS* [request-reply API](../../fastdds/rpc_dds/request_reply/request_reply_api_intro.md#request-reply-api-intro).

<a id="fastddsgen-rpc-calculator-basic-app-prerequisites"></a>

<a id="prerequisites"></a>

## Prerequisites

First of all, follow the steps outlined in the Installation Manual for the installation of
*eProsima Fast DDS* and all its dependencies.
Moreover, perform the steps outlined in [Linux installation of Fast DDS-Gen](../../installation/sources/sources_linux.md#fastddsgen-sl) or in
[Window installation of Fast DDS-Gen](../../installation/sources/sources_windows.md#fastddsgen-sw), depending on the operating system,
for the installation of the *eProsima Fast DDS-Gen* tool.

#### WARNING
RPC over DDS request-reply API is only available in the *eProsima Fast DDS* version 3.2.0 or later.
Please, make sure to have the correct version installed.

<a id="fastddsgen-rpc-calculator-basic-app-workspace"></a>

<a id="create-the-application-workspace"></a>

## Create the application workspace

First, create a directory named *workspace_CalculatorBasic*, which will represent the workspace of the application.

The workspace will have the following structure at the end of the project.
The files `build/client` and `build/server` corresponds to
the generated *Fast DDS* client and server applications, respectively:

```shell-session
.
├── build
│   ├── basic_client
│   ├── basic_server
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

<a id="fastddsgen-rpc-calculator-basic-app-dependencies"></a>

<a id="import-linked-libraries-and-its-dependencies"></a>

## Import linked libraries and its dependencies

The DDS application requires the *Fast DDS* and *Fast CDR* libraries.
The way of making these accessible from the
workspace depends on the installation procedure followed in the Installation Manual.

<a id="installation-from-binaries"></a>

### Installation from binaries

If the installation from binaries has been followed, these libraries are already accessible from the workspace.

* On Linux: The header files can be found in directories `/usr/include/fastdds/` and
  `/usr/include/fastcdr/` for *Fast DDS* and *Fast CDR* respectively.
  The compiled libraries of both can be found in the directory `/usr/lib/`.
* On Windows: The header files can be found in directories
  `C:\Program Files\eProsima\fastdds <major>.<minor>.<patch>\include\fastdds` and
  `C:\Program Files\eProsima\fastdds <major>.<minor>.<patch>\include\fastcdr\` for *Fast DDS* and *Fast CDR*
  respectively.
  The compiled libraries of both can be found in the directory `C:\Program Files\eProsima\fastdds
  <major>.<minor>.<patch>\lib\`.

<a id="colcon-installation"></a>

### Colcon installation

If the Colcon installation has been followed, there are several ways to import the libraries.
To make these accessible only from the current shell session, run one of the following two commands.

* On Linux:

```bash
source <path/to/Fast-DDS/workspace>/install/setup.bash
```

* On Windows:

```bash
<path/to/Fast-DDS/workspace>/install/setup.bat
```

However, to make these accessible from any session, add the *Fast DDS* installation directory to the `$PATH`
variable in the shell configuration files running the following command.

* On Linux:

```bash
echo 'source <path/to/Fast-DDS/workspace>/install/setup.bash' >> ~/.bashrc
```

* On Windows: Open the Edit the system environment variables control panel and add
  `<path/to/Fast-DDS/workspace>/install/setup.bat` to the `PATH`.

<a id="configure-the-cmake-project"></a>

## Configure the CMake project

We will use the CMake tool to manage the building of the project.
With your preferred text editor, create a new file called `CMakeLists.txt`
and copy and paste the following code snippet.
Save this file in the root directory of your workspace. If you have followed these steps, it should
be workspace_CalculatorBasic.

```cmake
cmake_minimum_required(VERSION 3.20)

project(RpcClientServerBasic)

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

message(STATUS "Configuring client server basic example...")
file(GLOB CLIENT_SERVER_BASIC_TYPES_SOURCES_CXX "src/types/*.cxx")
file(GLOB CLIENT_SERVER_BASIC_TYPES_SOURCES_IPP "src/types/*.ipp")

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

    interface Calculator
    {
        // Returns the minimum and maximum representable values
        void representation_limits(out long min_value, out long max_value);

        // Returns the result of value1 + value2
        long addition(in long value1, in long value2) raises (OverflowException);

        // Returns the result of value1 - value2
        long subtraction(in long value1, in long value2) raises (OverflowException);
    };
};
```

For additions and subtractions, an overflow exception is raised in case of working with
operands which produce a result that cannot be represented in a 32-bit integer.
For more information about how to define exceptions in IDL files, please check [Exceptions](../interfaces/exceptions.md#fastddsgen-interfaces-exceptions).

<a id="fastddsgen-rpc-calculator-basic-app-code-generation"></a>

<a id="generate-the-fast-dds-source-code-from-the-idl-file"></a>

## Generate the Fast DDS source code from the IDL file

Once the IDL file is created, the application files can be generated using **Fast DDS-Gen**.
In the workspace (*workspace_CalculatorBasic/types* directory), execute one of the following commands according to the
installation followed and the operating system:

* On Linux:
  - For an **installation from binaries** or a **colcon installation**:

  ```bash
  <path-to-Fast-DDS-workspace>/src/fastddsgen/scripts/fastddsgen calculator.idl
  ```

  - For a **stand-alone installation**, run:

  ```bash
  <path-to-Fast-DDS-Gen>/scripts/fastddsgen calculator.idl
  ```
* On Windows:
  - For a **colcon installation**:

  ```winbatch
  <path-to-Fast-DDS-workspace>/src/fastddsgen/scripts/fastddsgen.bat calculator.idl
  ```

  - For a **stand-alone installation**, run:

  ```winbatch
  <path-to-Fast-DDS-Gen>/scripts/fastddsgen.bat calculator.idl
  ```

  - For an **installation from binaries**, run:

  ```winbatch
  fastddsgen.bat calculator.idl
  ```

#### WARNING
The colcon installation does not build the `fastddsgen.jar` file although it does download the Fast DDS-Gen
repository. The following commands must be executed to build the Java executable:

```bash
cd <path-to-Fast-DDS-workspace>/src/fastddsgen
gradle assemble
```

After executing the command, the workspace directory will have the following structure:

```shell-session
.
├── CMakeLists.txt
└── src
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

<a id="fastddsgen-rpc-code-generation-basic-example-files-description"></a>

<a id="files-description"></a>

### Files description

A description of the generated files is as follows:

<a id="calculator"></a>

#### calculator

Contains the definition of the interface and its operations:

* `OverflowException` class represents the exception defined in the IDL file. It inherits
  from the *eProsima* exception class `RpcOperationError`, which is
  the base class for all exceptions raised by the *Fast DDS* RPC API when the server communicates an error.
* `Calculator` class represents the interface defined in the IDL file. Each operation is defined as
  a pure virtual function, expecting the client to implement it.

Additionally, for operation containing *out* parameters, a `calculator_<operation_name>_Out` structure is defined,
which is used to return the values of the *out* parameters after calling the operation.

Note that, due to the asynchronous nature of *Remote Procedure Calls*, operation calls return a
`RpcFuture` object, which can be used to retrieve the result of the operation when it is ready.

<a id="calculator-details"></a>

#### calculator_details

According to the
[RPC over DDS specification](https://www.omg.org/spec/DDS-RPC/1.0/PDF) (sections 7.5.1.1.4 and 7.5.1.1.5),
each operation defined in the interface should be mapped to a request type and a reply type,
used in the request/reply topics:

* On one hand, the request type is defined by a `calculator_<operation_name>_In` structure,
  containing the *in* and *inout*
  parameters of the operation, in the same order as defined in the IDL file.
* On the other hand, the reply type is defined by `calculator_<operation_name>_Out`
  and `calculator_<operator_name>_Result` structures.
  The first one contains the result of the operation.
  The second one contains optional members for the `_Out` structure and for each exception that
  can be raised.

In the top level, two structures `Calculator_Request` and `Calculator_Reply` are defined,
which are the types used to publish messages in the request and reply topics.
They contain the previously explained members for each operation.

<a id="calculatorclient"></a>

#### calculatorClient

Contains the `CalculatorClient` class,
which represents a client in the RPC communication and can be instantiated
calling `create_CalculatorClient` function.

Custom QoS can be passed when the client is created.
It implements the `Calculator` interface, so the user can call the operations directly: each call sends a
request to the server and waits for its reply.

<a id="calculatorserver"></a>

#### calculatorServer

Contains the `CalculatorServerLogic` class which implements the generic [`RpcServer`](../../fastdds/api_reference/dds_pim/rpc/rpcserver.md#_CPPv4N8eprosima7fastdds3dds3rpc9RpcServerE) interface, representing
the public API of the server.
User can run or stop a server calling [`run()`](../../fastdds/api_reference/dds_pim/rpc/rpcserver.md#_CPPv4N8eprosima7fastdds3dds3rpc9RpcServer3runEv) or [`stop()`](../../fastdds/api_reference/dds_pim/rpc/rpcserver.md#_CPPv4N8eprosima7fastdds3dds3rpc9RpcServer4stopEv) methods, respectively.

The generated class can be instantiated by the user calling `create_CalculatorServer` function.
Two overloads of this function are created, allowing the user to
[customize the scheduling](../rpc_server_schedule/intro.md#fastddsgen-rpc-server-schedule-intro) of the incoming requests.

<a id="calculatorserverimpl"></a>

#### calculatorServerImpl

Contains the base class for an implementation of the interface methods in the server side.
By default, the server implementation is empty and the user must implement the methods inheriting
from this class.

<a id="calculatorcdraux"></a>

#### calculatorCdrAux

Contains a set of *Fast CDR* serialization
and deserialization utilities for the types defined in the `calculator_details.hpp` file.

<a id="calculatorpubsubtypes"></a>

#### calculatorPubSubTypes

Contains the implementation of the methods
required to serialize and deserialize Request and Reply data types.

<a id="writing-the-application-source-code"></a>

## Writing the application source code

Now that the interface source code has been generated, the next step is to generate the application source code.
All the following files should be created in the `workspace_CalculatorBasic/src` directory.

<a id="fastddsgen-rpc-calculator-basic-server-application"></a>

<a id="server-application"></a>

### Server application

The first step is to create the server application. Running it, the user can run an RPC server
ready to process requests from client applications and send replies.

Create a `CalculatorServer.cpp` file and
copy the following code into the file:

```cpp
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

#include "ServerImplementation.hpp"
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

        // Create the server with default QoS, using the user-implemented operations
        std::shared_ptr<CalculatorServer_IServerImplementation> server_impl =
            std::make_shared<ServerImplementation>();

        ReplierQos qos;

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

    std::shared_ptr<RpcServer> server_;
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
```

Additionally, create a `ServerImplementation.hpp` file with the implementation of the server-side operations:

```cpp
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

};

#endif  // EXAMPLES_CPP_RPC_CLIENT_SERVER_BASIC__SERVER_IMPLEMENTATION_HPP
```

#### WARNING
The user implementation of the server-side operations should be placed in a separate file
and inherit from the `CalculatorServerImplementation` struct to avoid being overridden
by *Fast DDS-Gen* when IDL interface’s source code is regenerated. User should use this derived
class when creates the server instance.

<a id="examining-the-code"></a>

#### Examining the code

The `CalculatorServer.cpp` file contains the implementation of the `Server` class, formed by
a Server instance and its related DomainParticipant instance:

```cpp
protected:

    std::shared_ptr<RpcServer> server_;
    DomainParticipant* participant_;
    std::string service_name_;
    std::atomic<bool> stop_;
```

When a `Server` instance is initialized, both `DomainParticipant` and `CalculatorServer` instances
are created. The `CalculatorServer` instance is created using the previously created
`DomainParticipant` instance, the implementation of the server-side IDL interface’s operations
and the name of the RPC service:

```cpp
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

        // Create the server with default QoS, using the user-implemented operations
        std::shared_ptr<CalculatorServer_IServerImplementation> server_impl =
            std::make_shared<ServerImplementation>();

        ReplierQos qos;

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
```

Once all the Server’s entities are created, the RPC server is started using the
`CalculatorServer::run()` method:

```cpp
    void run()
    {
        if (is_stopped())
        {
            return;
        }

        server_->run();

        std::cout << "Server running" << std::endl;
    }
```

Similarly, when the application is stopped, the
RPC server is also stopped using the `CalculatorServer::stop()` method:

```cpp
    void stop()
    {
        stop_.store(true);
        server_->stop();

        std::cout << "Server execution stopped" << std::endl;
    }
```

Before finishing the application, all the internal DDS and RPC entities involved in the RPC
communication are deleted by calling `CalculatorServer` and `DomainParticipant` destructors:

```cpp
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
```

The `main()` function contains all the steps described above. It runs the `Server::run()` method
in a different thread to allow the user to stop the server by sending a signal (for example, Ctrl+C):

```cpp
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
```

<a id="client-application"></a>

### Client application

Now, we will create the client application. Running it, a new RPC client will be created
and ready to send requests to the server application. User can send requests
by specifying the operation name and through the CLI, and the results
will be printed on the screen after receiving the reply from the server.

Create a `CalculatorClient.cpp` file with this
[content](https://github.com/eProsima/Fast-DDS-docs/tree/master/code/Examples/C++/RpcClientServerBasic/src/CalculatorClient.cpp):

<a id="id2"></a>

#### Examining the code

The `CalculatorClient.cpp` file contains the implementation of the `Client` class, formed by
a `Calculator` client instance and its related `DomainParticipant`.
Additionally, the operation to be performed is also stored:

```cpp
    std::shared_ptr<Calculator> client_;
    DomainParticipant* participant_;
    std::unique_ptr<Operation> operation_;
    std::string service_name_;
```

All this members are initialized calling `init()` method, in the same way as in the `Server` class:

```cpp
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
```

When the client performs an operation (*i.e*: sends a request), three different situations can happen:

* The operation is successful (*i.e* the client sends the request and receives the reply
  from the server).
* The operation fails (*i.e* the client sends the request but an exception occurs). For example, if the
  operation is not implemented in the server side or an RPC exception occurs (for example, if computing
  the result raises an `OverflowException`).
* A timeout is configured to avoid blocking the thread infinitely if no replies are received for a given
  request and the maximum time is reached.
  In this case, the client will stop waiting for the reply and return a timeout error.

To process each operation in the same way, the following design pattern is used: each operation implements
a `Operation` abstract class, which contains a `execute()` method:

```cpp
class Operation
{

public:

    virtual OperationStatus execute() = 0;

};
```

After calling this method, it will
return an enum `OperationStatus` indicating the result of the operation (and addressing each of the
cases previously described):

```cpp
enum class OperationStatus
{
    SUCCESS,
    TIMEOUT,
    ERROR
};
```

It makes easier to add new operations (for example, `@feed` operations) without modifying the
main execution flow. Each operation stores the data required to execute the operation,
for example, a reference to the client used to send the request, as well as the operation input data.

When `RepresentationLimits` operation is executed, client sends a request to the server and waits
for the server reply, printing the received result. If something fails or the timeout is exceeded (for example,
if no servers are available), the operation fails:

```cpp
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
            RpcFuture<Calculator_representation_limits_Out> future = client->representation_limits();

            if (future.wait_for(std::chrono::milliseconds(1000)) != std::future_status::ready)
            {
                std::cerr << "Operation timed out" << std::endl;

                return OperationStatus::TIMEOUT;
            }

            try
            {
                out_param_values_ = future.get();

                // Print the results
                std::cout <<
                        "Representation limits received: min_value = " << out_param_values_.min_value
                        << ", max_value = " << out_param_values_.max_value << std::endl;

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

    Calculator_representation_limits_Out out_param_values_;
    std::weak_ptr<Calculator> client_;

};
```

Similarly for `Addition` operation:

```cpp
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
```

and `Subtraction` operation:

```cpp
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
```

The operation to be performed is configured from the parsed CLI input
using the `set_operation()` factory method. Input operands are hardcoded to simply
the input parsing:

```cpp
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
```

When `send_request()` method is called, the input operation is configured and the client executes it.
A boolean is returned, `true` if the operation is successful or `false` otherwise:

```cpp
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
```

Finally, the `main()` function process the user input (specifying the operation to be performed),
initializes a new client and tries to execute the operation until a max number of attempts `n_attempts`:

```cpp
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
```

Note that, before sending the first request, we are waiting some time to make sure
that all internal DDS entities are matched. This way, we avoid losing requests by sending them
before client and server are discovered each other.

<a id="update-the-cmakelists-txt-file"></a>

### Update the *CMakeLists.txt* file

Before building the application, we need to update the *CMakeLists.txt* file of the workspace
to add the executable and the required libraries. The following code should be added at the end
of the file:

```cmake
add_executable(basic_client src/CalculatorClient.cpp
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_CXX}
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_IPP}
)

target_link_libraries(basic_client fastdds fastcdr)

add_executable(basic_server src/CalculatorServer.cpp
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_CXX}
    ${CLIENT_SERVER_BASIC_TYPES_SOURCES_IPP}
)

target_link_libraries(basic_server fastdds fastcdr)
```

<a id="build-the-application"></a>

## Build the application

Now that all the files are created, the application can be built.
To do so, open a terminal in the *workspace_CalculatorBasic* directory and run the following commands:

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

The generated executable will be located in the *build* subdirectory.

<a id="run-the-application"></a>

## Run the application

To test the application, open two terminals in the *workspace_CalculatorBasic* directory
and execute the following commands:

* In the first terminal, run the server application:

```bash
./build/basic_server
```

* In the second terminal, run the client application and specify the operation to be performed.
  For example, to perform an addition of two numbers (5 and 3), run the following command:

```bash
./build/basic_client add
```

You should see the result of the operation printed on the screen:

```shell-session
Attempting to send request, attempt 2/10
Addition result: 5 + 3 = 8
Request sent successfully
```

The output of the rest operations should be similar to the following:

* Subtraction:

```shell-session
./build/basic_client sub

Attempting to send request, attempt 2/10
Subtraction result: 5 - 3 = 2
Request sent successfully
```

* Representation limits:

```shell-session
./build/basic_client rep

Attempting to send request, attempt 2/10
Representation limits received: min_value = -2147483648, max_value = 2147483647
Request sent successfully
```

<a id="next-steps"></a>

## Next steps

The application that we have created only contains basic asynchronous RPC operations.
This example can be extended to include streaming of input and output data by defining
`@feed` annotated operations in the interface of the IDL file. An example of this can be seen
in the next section ([Building a RPC Client/Server application with data streaming](../rpc_calculator_feed_app/intro.md#fastddsgen-rpc-calculator-feed-app-intro)).
