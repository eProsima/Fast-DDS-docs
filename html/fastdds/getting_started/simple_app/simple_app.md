<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="writing-pubsub-app"></a>

<a id="writing-a-simple-c-publisher-and-subscriber-application"></a>

# Writing a simple C++ publisher and subscriber application

This section details how to create a simple Fast DDS application with a publisher and a subscriber
using C++ API step by step.
It is also possible to self-generate a similar example to the one implemented in this section by using the
*eProsima Fast DDS-Gen* tool.
This additional approach is explained in [Building a publish/subscribe application](../../../fastddsgen/pubsub_app/pubsub_app.md#fastddsgen-pubsub-app).

<a id="background"></a>

## Background

DDS is a data-centric communications middleware that implements the DCPS model.
This model is based on the development of a publisher, a data generating element; and a subscriber, a
data consuming element.
These entities communicate by means of the topic, an element that binds both DDS entities.
Publishers generate information under a topic and subscribers subscribe to this same topic to receive information.

<a id="prerequisites"></a>

## Prerequisites

First of all, you need to follow the steps outlined in the Installation Manual for the installation of
*eProsima Fast DDS* and all its dependencies.
You also need to have completed the steps outlined in the Installation Manual for the installation of the
*eProsima Fast DDS-Gen* tool. Moreover, all the commands provided in this tutorial are outlined for a Linux
environment.

<a id="create-the-application-workspace"></a>

## Create the application workspace

The application workspace will have the following structure at the end of the project.
Files `build/DDSHelloWorldPublisher` and `build/DDSHelloWorldSubscriber` are the Publisher application and
Subscriber application respectively.

```shell-session
.
└── workspace_DDSHelloWorld
    ├── build
    │   ├── CMakeCache.txt
    │   ├── CMakeFiles
    │   ├── cmake_install.cmake
    │   ├── DDSHelloWorldPublisher
    │   ├── DDSHelloWorldSubscriber
    │   └── Makefile
    ├── CMakeLists.txt
    └── src
        ├── HelloWorld.hpp
        ├── HelloWorld.idl
        ├── HelloWorldCdrAux.hpp
        ├── HelloWorldCdrAux.ipp
        ├── HelloWorldPublisher.cpp
        ├── HelloWorldPubSubTypes.cxx
        ├── HelloWorldPubSubTypes.hpp
        ├── HelloWorldSubscriber.cpp
        ├── HelloWorldTypeObjectSupport.cxx
        └── HelloWorldTypeObjectSupport.hpp
```

Let’s create the directory tree first.

```bash
mkdir workspace_DDSHelloWorld && cd workspace_DDSHelloWorld
mkdir src build
```

<a id="import-linked-libraries-and-its-dependencies"></a>

## Import linked libraries and its dependencies

The DDS application requires the Fast DDS and Fast CDR libraries.
Depending on the installation procedure followed, the process of making these libraries available for our DDS
application will be slightly different.

<a id="installation-from-binaries-and-manual-installation"></a>

### Installation from binaries and manual installation

If we have followed the installation from binaries or the manual installation, these libraries are already accessible
from the workspace.
On Linux, the header files can be found in directories /usr/include/fastdds/ and
/usr/include/fastcdr/ for Fast DDS and Fast CDR respectively. The compiled libraries of both can be found in
the directory /usr/lib/.

<a id="colcon-installation"></a>

### Colcon installation

From a Colcon installation there are several ways to import the libraries.
If the libraries need to be available just for the current session, run the following command.

```bash
source <path/to/Fast-DDS/workspace>/install/setup.bash
```

They can be made accessible from any session by adding the Fast DDS installation directory to your `$PATH`
variable in the shell configuration files for the current user running the following command.

```bash
echo 'source <path/to/Fast-DDS/workspace>/install/setup.bash' >> ~/.bashrc
```

This will set up the environment after each of this user’s logins.

<a id="configure-the-cmake-project"></a>

## Configure the CMake project

We will use the CMake tool to manage the building of the project.
With your preferred text editor, create a new file called CMakeLists.txt and copy and paste the following code snippet.
Save this file in the root directory of your workspace. If you have followed these steps, it should
be workspace_DDSHelloWorld.

```cmake
cmake_minimum_required(VERSION 3.20)

project(DDSHelloWorld)

# Find requirements
if(NOT fastcdr_FOUND)
    find_package(fastcdr 2 REQUIRED)
endif()

if(NOT fastdds_FOUND)
    find_package(fastdds 3 REQUIRED)
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

message(STATUS "Configuring HelloWorld publisher/subscriber example...")
file(GLOB DDS_HELLOWORLD_SOURCES_CXX "src/*.cxx")

```

In each section we will complete this file to include the specific generated files.

<a id="build-the-topic-data-type"></a>

## Build the topic data type

*eProsima Fast DDS-Gen* is a Java application that generates source code using the data types defined in an
Interface Description Language (IDL) file. This application can do two different things:

1. Generate C++ definitions for your custom topic.
2. Generate a functional example that uses your topic data.

It will be the former that will be followed in this tutorial.
To see an example of application of the latter you can check this other [example](../../../fastddsgen/pubsub_app/pubsub_app.md#fastddsgen-pubsub-app).
See [Introduction](../../../fastddsgen/introduction/introduction.md#fastddsgen-intro) for further details.
For this project, we will use the Fast DDS-Gen application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

```bash
cd src && touch HelloWorld.idl
```

This creates the HelloWorld.idl file in the src directory.
Open the file in a text editor and copy and paste the following snippet of code.

```omg-idl
struct HelloWorld
{
    unsigned long index;
    string message;
};
```

By doing this we have defined the `HelloWorld` data type, which has two elements: an *index* of type `uint32_t`
and a *message* of type `std::string`.
All that remains is to generate the source code that implements this data type in C++11.
To do this, run the following command from the `src` directory.

```bash
<path/to/Fast DDS-Gen>/scripts/fastddsgen HelloWorld.idl
```

This must have generated the following files:

* HelloWorld.hpp: HelloWorld type definition.
* HelloWorldPubSubTypes.cxx: Interface used by Fast DDS to support HelloWorld type.
* HelloWorldPubSubTypes.hpp: Header file for HelloWorldPubSubTypes.cxx.
* HelloWorldCdrAux.ipp: Serialization and Deserialization code for the HelloWorld type.
* HelloWorldCdrAux.hpp: Header file for HelloWorldCdrAux.ipp.
* HelloWorldTypeObjectSupport.cxx: [TypeObject representation](../../xtypes/discovery_matching.md#xtypes-discovery-matching) registration code.
* HelloWorldTypeObjectSupport.hpp: Header file for HelloWorldTypeObjectSupport.cxx.

<a id="write-the-fast-dds-publisher"></a>

## Write the Fast DDS publisher

From the src directory in the workspace, run the following command to download the HelloWorldPublisher.cpp file.

```bash
wget -O HelloWorldPublisher.cpp \
    https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
```

This is the C++ source code for the publisher application.
It is going to send 10 publications under the topic HelloWorldTopic.

```C++
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

/**
 * @file HelloWorldPublisher.cpp
 *
 */

#include "HelloWorldPubSubTypes.hpp"

#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

class HelloWorldPublisher
{
private:

    HelloWorld hello_;

    DomainParticipant* participant_;

    Publisher* publisher_;

    Topic* topic_;

    DataWriter* writer_;

    TypeSupport type_;

    class PubListener : public DataWriterListener
    {
    public:

        PubListener()
            : matched_(0)
        {
        }

        ~PubListener() override
        {
        }

        void on_publication_matched(
                DataWriter*,
                const PublicationMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                        << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
            }
        }

        std::atomic_int matched_;

    } listener_;

public:

    HelloWorldPublisher()
        : participant_(nullptr)
        , publisher_(nullptr)
        , topic_(nullptr)
        , writer_(nullptr)
        , type_(new HelloWorldPubSubType())
    {
    }

    virtual ~HelloWorldPublisher()
    {
        if (writer_ != nullptr)
        {
            publisher_->delete_datawriter(writer_);
        }
        if (publisher_ != nullptr)
        {
            participant_->delete_publisher(publisher_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the publisher
    bool init()
    {
        hello_.index(0);
        hello_.message("HelloWorld");

        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the publications Topic
        topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Publisher
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // Create the DataWriter
        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
        return true;
    }

    //!Send a publication
    bool publish()
    {
        if (listener_.matched_ > 0)
        {
            hello_.index(hello_.index() + 1);
            writer_->write(&hello_);
            return true;
        }
        return false;
    }

    //!Run the Publisher
    void run(
            uint32_t samples)
    {
        uint32_t samples_sent = 0;
        while (samples_sent < samples)
        {
            if (publish())
            {
                samples_sent++;
                std::cout << "Message: " << hello_.message() << " with index: " << hello_.index()
                            << " SENT" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }
};

int main(
        int argc,
        char** argv)
{
    std::cout << "Starting publisher." << std::endl;
    uint32_t samples = 10;

    HelloWorldPublisher* mypub = new HelloWorldPublisher();
    if(mypub->init())
    {
        mypub->run(samples);
    }

    delete mypub;
    return 0;
}
```

<a id="examining-the-code"></a>

### Examining the code

At the beginning of the file we have a Doxygen style comment block with the `@file` field that tells us the name of
the file.

```C++
/**
 * @file HelloWorldPublisher.cpp
 *
 */
```

Below are the includes of the C++ headers.
The first one includes the HelloWorldPubSubTypes.hpp file with the serialization and deserialization functions of the
data type that we have defined in the previous section.

```C++
#include "HelloWorldPubSubTypes.hpp"
```

The next block includes the C++ header files that allow the use of the Fast DDS API.

* [`DomainParticipantFactory`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE).
  Allows for the creation and destruction of DomainParticipant objects.
* [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
  Acts as a container for all other Entity objects and as a factory for the Publisher, Subscriber,
  and Topic objects.
* [`TypeSupport`](../../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE).
  Provides the participant with the functions to serialize, deserialize and get the key of a
  specific data type.
* [`Publisher`](../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE).
  It is the object responsible for the creation of DataWriters.
* [`DataWriter`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE).
  Allows the application to set the value of the data to be published under a given Topic.
* [`DataWriterListener`](../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE).
  Allows the redefinition of the functions of the DataWriterListener.

```C++
#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
```

Next, we define the namespace that contains the eProsima Fast DDS classes and functions that we are going to use in
our application.

```C++
using namespace eprosima::fastdds::dds;
```

The next line creates the `HelloWorldPublisher` class that implements a publisher.

```C++
class HelloWorldPublisher
```

Continuing with the private data members of the class, the `hello_` data member is defined as an object of the
`HelloWorld` class that defines the data type
we created with the IDL file.
Next, the private data members corresponding to the participant, publisher, topic, DataWriter and data type are
defined.
The `type_` object of the [`TypeSupport`](../../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) class is the object that will be used to register the topic data type
in the DomainParticipant.

```C++
private:

    HelloWorld hello_;

    DomainParticipant* participant_;

    Publisher* publisher_;

    Topic* topic_;

    DataWriter* writer_;

    TypeSupport type_;
```

Then, the `PubListener` class is defined by inheriting from the [`DataWriterListener`](../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListenerE) class.
This class overrides the default DataWriter listener callbacks, which allows the execution of routines in case of an
event.
The overridden callback [`on_publication_matched()`](../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus)
allows the definition of a series of actions when a new DataReader
is detected listening to the topic under which the DataWriter is publishing.
The `info.current_count_change` detects these changes of DataReaders that are matched to the
DataWriter.
This is a member in the [`MatchedStatus`](../../api_reference/dds_pim/core/status/matchedstatus.md#_CPPv4N8eprosima7fastdds3dds13MatchedStatusE) structure that allows tracking changes in the status of subscriptions.
Finally, the `listener_` object of the class is defined as an instance of `PubListener`.

```C++
class PubListener : public DataWriterListener
{
public:

    PubListener()
        : matched_(0)
    {
    }

    ~PubListener() override
    {
    }

    void on_publication_matched(
            DataWriter*,
            const PublicationMatchedStatus& info) override
    {
        if (info.current_count_change == 1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher matched." << std::endl;
        }
        else if (info.current_count_change == -1)
        {
            matched_ = info.total_count;
            std::cout << "Publisher unmatched." << std::endl;
        }
        else
        {
            std::cout << info.current_count_change
                    << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
        }
    }

    std::atomic_int matched_;

} listener_;
```

The public constructor and destructor of the `HelloWorldPublisher` class are defined below.
The constructor initializes the private data members of the class to `nullptr`, with the exception of the TypeSupport
object, that is initialized as an instance of the `HelloWorldPubSubType` class.
The class destructor removes these data members and thus cleans the system memory.

```C++
HelloWorldPublisher()
    : participant_(nullptr)
    , publisher_(nullptr)
    , topic_(nullptr)
    , writer_(nullptr)
    , type_(new HelloWorldPubSubType())
{
}

virtual ~HelloWorldPublisher()
{
    if (writer_ != nullptr)
    {
        publisher_->delete_datawriter(writer_);
    }
    if (publisher_ != nullptr)
    {
        participant_->delete_publisher(publisher_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
}
```

Continuing with the public member functions of the `HelloWorldPublisher` class, the next snippet of code defines
the public publisher’s initialization member function.
This function performs several actions:

1. Initializes the content of the HelloWorld type `hello_` structure members.
2. Assigns a name to the participant through the QoS of the DomainParticipant.
3. Uses the [`DomainParticipantFactory`](../../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE) to create the participant.
4. Registers the data type defined in the IDL.
5. Creates the topic for the publications.
6. Creates the publisher.
7. Creates the DataWriter with the listener previously created.

As you can see, the QoS configuration for all entities, except for the participant’s name, is the default configuration
([`PARTICIPANT_QOS_DEFAULT`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds23PARTICIPANT_QOS_DEFAULTE), [`PUBLISHER_QOS_DEFAULT`](../../api_reference/dds_pim/publisher/publisherqos.md#_CPPv4N8eprosima7fastdds3dds21PUBLISHER_QOS_DEFAULTE), [`TOPIC_QOS_DEFAULT`](../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds17TOPIC_QOS_DEFAULTE), [`DATAWRITER_QOS_DEFAULT`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds22DATAWRITER_QOS_DEFAULTE)).
The default value of the QoS of each DDS Entity can be checked in the
[DDS standard](https://www.omg.org/spec/DDS/About-DDS/).

```C++
//!Initialize the publisher
bool init()
{
    hello_.index(0);
    hello_.message("HelloWorld");

    DomainParticipantQos participantQos;
    participantQos.name("Participant_publisher");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

    if (participant_ == nullptr)
    {
        return false;
    }

    // Register the Type
    type_.register_type(participant_);

    // Create the publications Topic
    topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);

    if (topic_ == nullptr)
    {
        return false;
    }

    // Create the Publisher
    publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

    if (publisher_ == nullptr)
    {
        return false;
    }

    // Create the DataWriter
    writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

    if (writer_ == nullptr)
    {
        return false;
    }
    return true;
}
```

To make the publication, the public member function `publish()` is implemented.
In the DataWriter’s listener callback which states that the DataWriter has matched with a DataReader
that listens to the publication topic, the data member `matched_` is updated. It contains the number of DataReaders
discovered.
Therefore, when the first DataReader has been discovered, the application starts to publish.
This is simply the writing of a change by the DataWriter object.

```C++
//!Send a publication
bool publish()
{
    if (listener_.matched_ > 0)
    {
        hello_.index(hello_.index() + 1);
        writer_->write(&hello_);
        return true;
    }
    return false;
}
```

The public *run* function executes the action of publishing a given number of times, waiting for 1 second between
publications.

```C++
//!Run the Publisher
void run(
        uint32_t samples)
{
    uint32_t samples_sent = 0;
    while (samples_sent < samples)
    {
        if (publish())
        {
            samples_sent++;
            std::cout << "Message: " << hello_.message() << " with index: " << hello_.index()
                        << " SENT" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}
```

Finally, the HelloWorldPublisher is initialized and run in main.

```C++
int main(
        int argc,
        char** argv)
{
    std::cout << "Starting publisher." << std::endl;
    uint32_t samples = 10;

    HelloWorldPublisher* mypub = new HelloWorldPublisher();
    if(mypub->init())
    {
        mypub->run(samples);
    }

    delete mypub;
    return 0;
}
```

<a id="cmakelists-txt"></a>

### CMakeLists.txt

Include at the end of the CMakeList.txt file you created earlier the following code snippet. This adds all the source
files needed to build the executable, and links the executable and the library together.

```bash
add_executable(DDSHelloWorldPublisher src/HelloWorldPublisher.cpp ${DDS_HELLOWORLD_SOURCES_CXX})
target_link_libraries(DDSHelloWorldPublisher fastdds fastcdr)
```

At this point the project is ready for building, compiling and running the publisher application.
From the build directory in the workspace, run the following commands.

```bash
cmake ..
cmake --build .
./DDSHelloWorldPublisher
```

<a id="write-the-fast-dds-subscriber"></a>

## Write the Fast DDS subscriber

From the src directory in the workspace, execute the following command to download the HelloWorldSubscriber.cpp file.

```bash
wget -O HelloWorldSubscriber.cpp \
    https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
```

This is the C++ source code for the subscriber application.
The application runs a subscriber until it receives 10 samples under the topic HelloWorldTopic.
At this point the subscriber stops.

```C++
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

/**
 * @file HelloWorldSubscriber.cpp
 *
 */

#include "HelloWorldPubSubTypes.hpp"

#include <chrono>
#include <thread>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

using namespace eprosima::fastdds::dds;

class HelloWorldSubscriber
{
private:

    DomainParticipant* participant_;

    Subscriber* subscriber_;

    DataReader* reader_;

    Topic* topic_;

    TypeSupport type_;

    class SubListener : public DataReaderListener
    {
    public:

        SubListener()
            : samples_(0)
        {
        }

        ~SubListener() override
        {
        }

        void on_subscription_matched(
                DataReader*,
                const SubscriptionMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                std::cout << "Subscriber matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                std::cout << "Subscriber unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                          << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
            }
        }

        void on_data_available(
                DataReader* reader) override
        {
            SampleInfo info;
            if (reader->take_next_sample(&hello_, &info) == eprosima::fastdds::dds::RETCODE_OK)
            {
                if (info.valid_data)
                {
                    samples_++;
                    std::cout << "Message: " << hello_.message() << " with index: " << hello_.index()
                              << " RECEIVED." << std::endl;
                }
            }
        }

        HelloWorld hello_;

        std::atomic_int samples_;

    }
    listener_;

public:

    HelloWorldSubscriber()
        : participant_(nullptr)
        , subscriber_(nullptr)
        , topic_(nullptr)
        , reader_(nullptr)
        , type_(new HelloWorldPubSubType())
    {
    }

    virtual ~HelloWorldSubscriber()
    {
        if (reader_ != nullptr)
        {
            subscriber_->delete_datareader(reader_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        if (subscriber_ != nullptr)
        {
            participant_->delete_subscriber(subscriber_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the subscriber
    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_subscriber");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the subscriptions Topic
        topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Subscriber
        subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

        if (subscriber_ == nullptr)
        {
            return false;
        }

        // Create the DataReader
        reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &listener_);

        if (reader_ == nullptr)
        {
            return false;
        }

        return true;
    }

    //!Run the Subscriber
    void run(
            uint32_t samples)
    {
        while (listener_.samples_ < samples)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

};

int main(
        int argc,
        char** argv)
{
    std::cout << "Starting subscriber." << std::endl;
    uint32_t samples = 10;

    HelloWorldSubscriber* mysub = new HelloWorldSubscriber();
    if (mysub->init())
    {
        mysub->run(samples);
    }

    delete mysub;
    return 0;
}
```

<a id="id1"></a>

### Examining the code

Since the source code of both the publisher and subscriber applications is mostly identical, this document will focus
on the main differences between them, omitting the parts of the code that have already been explained.

Following the same structure as in the publisher explanation, the first step is the includes of the C++ header files.
In these, the files that include the publisher class are replaced by the subscriber class and the data writer class by
the data reader class.

* [`Subscriber`](../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE).
  It is the object responsible for the creation and configuration of DataReaders.
* [`DataReader`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE).
  It is the object responsible for the actual reception of the data.
  It registers in the application the topic (TopicDescription) that identifies the data to be read and
  accesses the data received by the subscriber.
* [`DataReaderListener`](../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListenerE).
  This is the listener assigned to the data reader.
* [`DataReaderQoS`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE).
  Structure that defines the QoS of the DataReader.
* [`SampleInfo`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE).
  It is the information that accompanies each sample that is ‘read’ or ‘taken’.

```C++
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
```

The next line defines the `HelloWorldSubscriber` class that implements a subscriber.

```C++
class HelloWorldSubscriber
```

Starting with the private data members of the class, it is worth mentioning the implementation of the data reader
listener.
The private data members of the class will be the participant, the subscriber, the topic, the data reader, and the
data type.
As it was the case with the data writer, the listener implements the callbacks to be executed in case an event
occurs.
The first overridden callback of the SubListener is the [`on_subscription_matched()`](../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener23on_subscription_matchedEP10DataReaderRKN7fastdds3dds25SubscriptionMatchedStatusE), which is the
analog of the [`on_publication_matched()`](../../api_reference/dds_pim/publisher/datawriterlistener.md#_CPPv4N8eprosima7fastdds3dds18DataWriterListener22on_publication_matchedEP10DataWriterRK24PublicationMatchedStatus) callback of the DataWriter.

```C++
void on_subscription_matched(
        DataReader*,
        const SubscriptionMatchedStatus& info) override
{
    if (info.current_count_change == 1)
    {
        std::cout << "Subscriber matched." << std::endl;
    }
    else if (info.current_count_change == -1)
    {
        std::cout << "Subscriber unmatched." << std::endl;
    }
    else
    {
        std::cout << info.current_count_change
                  << " is not a valid value for SubscriptionMatchedStatus current count change" << std::endl;
    }
}
```

The second overridden callback is [`on_data_available()`](../../api_reference/dds_pim/subscriber/datareaderlistener.md#_CPPv4N8eprosima7fastdds3dds18DataReaderListener17on_data_availableEP10DataReader).
In this, the next received sample that the data reader can access is taken and processed to display its content.
It is here that the object of the [`SampleInfo`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfoE) class is defined, which determines whether a sample has already
been read or taken.
Each time a sample is read, the counter of samples received is increased.

```C++
void on_data_available(
        DataReader* reader) override
{
    SampleInfo info;
    if (reader->take_next_sample(&hello_, &info) == eprosima::fastdds::dds::RETCODE_OK)
    {
        if (info.valid_data)
        {
            samples_++;
            std::cout << "Message: " << hello_.message() << " with index: " << hello_.index()
                      << " RECEIVED." << std::endl;
        }
    }
}
```

The public constructor and destructor of the class is defined below.

```C++

HelloWorldSubscriber()
    : participant_(nullptr)
    , subscriber_(nullptr)
    , topic_(nullptr)
    , reader_(nullptr)
    , type_(new HelloWorldPubSubType())
{
}

virtual ~HelloWorldSubscriber()
{
    if (reader_ != nullptr)
    {
        subscriber_->delete_datareader(reader_);
    }
    if (topic_ != nullptr)
    {
        participant_->delete_topic(topic_);
    }
    if (subscriber_ != nullptr)
    {
        participant_->delete_subscriber(subscriber_);
    }
    DomainParticipantFactory::get_instance()->delete_participant(participant_);
```

Next comes the subscriber initialization public member function.
This is the same as the initialization public member function defined for the `HelloWorldPublisher`.
The QoS configuration for all entities, except for the participant’s name, is the default QoS
([`PARTICIPANT_QOS_DEFAULT`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds23PARTICIPANT_QOS_DEFAULTE), [`SUBSCRIBER_QOS_DEFAULT`](../../api_reference/dds_pim/subscriber/subscriberqos.md#_CPPv4N8eprosima7fastdds3dds22SUBSCRIBER_QOS_DEFAULTE), [`TOPIC_QOS_DEFAULT`](../../api_reference/dds_pim/topic/topicqos.md#_CPPv4N8eprosima7fastdds3dds17TOPIC_QOS_DEFAULTE), [`DATAREADER_QOS_DEFAULT`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds22DATAREADER_QOS_DEFAULTE)).
The default value of the QoS of each DDS Entity can be checked in the
[DDS standard](https://www.omg.org/spec/DDS/About-DDS/).

```C++

//!Initialize the subscriber
bool init()
{
    DomainParticipantQos participantQos;
    participantQos.name("Participant_subscriber");
    participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

    if (participant_ == nullptr)
    {
        return false;
    }

    // Register the Type
    type_.register_type(participant_);

    // Create the subscriptions Topic
    topic_ = participant_->create_topic("HelloWorldTopic", "HelloWorld", TOPIC_QOS_DEFAULT);

    if (topic_ == nullptr)
    {
        return false;
    }

    // Create the Subscriber
    subscriber_ = participant_->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

    if (subscriber_ == nullptr)
    {
        return false;
    }

    // Create the DataReader
    reader_ = subscriber_->create_datareader(topic_, DATAREADER_QOS_DEFAULT, &listener_);

    if (reader_ == nullptr)
    {
        return false;
    }

    return true;
```

The public member function `run()` ensures that the subscriber runs until all the samples have been received.
This member function implements an active wait of the subscriber, with a 100ms sleep interval to ease the CPU.

```C++

//!Run the Subscriber
void run(
        uint32_t samples)
{
    while (listener_.samples_ < samples)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
```

Finally, the participant that implements a subscriber is initialized and run in main.

```C++
};

int main(
        int argc,
        char** argv)
{
    std::cout << "Starting subscriber." << std::endl;
    uint32_t samples = 10;

    HelloWorldSubscriber* mysub = new HelloWorldSubscriber();
    if (mysub->init())
    {
        mysub->run(samples);
    }

    delete mysub;
```

<a id="id3"></a>

### CMakeLists.txt

Include at the end of the CMakeList.txt file you created earlier the following code snippet.
This adds all the source
files needed to build the executable, and links the executable and the library together.

```bash
add_executable(DDSHelloWorldSubscriber src/HelloWorldSubscriber.cpp ${DDS_HELLOWORLD_SOURCES_CXX})
target_link_libraries(DDSHelloWorldSubscriber fastdds fastcdr)
```

At this point the project is ready for building, compiling and running the subscriber application.
From the build directory in the workspace, run the following commands.

```bash
cmake ..
cmake --build .
./DDSHelloWorldSubscriber
```

<a id="putting-all-together"></a>

## Putting all together

Finally, from the build directory, run the publisher and subscriber applications from two terminals.

```bash
./DDSHelloWorldPublisher
./DDSHelloWorldSubscriber
```

<video width="100%" autoplay loop>
    <source src="../../../_static/simple_pubsub.mp4">
    Your browser does not support the video tag.
</video>

<a id="summary"></a>

## Summary

In this tutorial you have built a publisher and a subscriber DDS application.
You have also learned how to build the CMake file for source code compilation, and how to include and use the Fast DDS
and Fast CDR libraries in your project.

<a id="next-steps"></a>

## Next steps

In the *eProsima Fast DDS* Github repository you will find more complex examples that implement DDS communication for
a multitude of use cases and scenarios. You can find them
[here](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp).
