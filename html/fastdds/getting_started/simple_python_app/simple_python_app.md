<a id="writing-pubsub-python-app"></a>

<a id="writing-a-simple-python-publisher-and-subscriber-application"></a>

# Writing a simple Python publisher and subscriber application

This section details how to create a simple Fast DDS application with a publisher and a subscriber
using Python API step by step.

<a id="background"></a>

## Background

DDS is a data-centric communications middleware that implements the DCPS model.
This model is based on the development of a publisher, a data generating element; and a subscriber, a
data consuming element.
These entities communicate by means of the topic, an element that binds both DDS entities.
Publishers generate information under a topic and subscribers subscribe to this same topic to receive information.

<a id="prerequisites"></a>

## Prerequisites

First of all, please follow the steps outlined in the [Installation Manual](../../../installation/sources/sources_linux.md#fastdds-python-sl)
section for the installation of *eProsima Fast DDS Python* and all its dependencies.

#### NOTE
All the commands provided in this tutorial are outlined for a Linux environment.

<a id="create-the-application-workspace"></a>

## Create the application workspace

The application workspace will have the following structure at the end of the project.
Files `HelloWorldPublisher.py` and `HelloWorldSubscriber.py` are the Publisher application and
Subscriber application respectively.

```shell-session
.
├── CMakeCache.txt
├── CMakeFiles
├── CMakeLists.txt
├── HelloWorld.hpp
├── HelloWorld.i
├── HelloWorld.idl
├── HelloWorld.py
├── HelloWorldCdrAux.hpp
├── HelloWorldCdrAux.ipp
├── HelloWorldPubSubTypes.cxx
├── HelloWorldPubSubTypes.hpp
├── HelloWorldPubSubTypes.i
├── HelloWorldPublisher.py
├── HelloWorldSubscriber.py
├── HelloWorldTypeObjectSupport.cxx
├── HelloWorldTypeObjectSupport.hpp
├── Makefile
├── _HelloWorldWrapper.so
├── cmake_install.cmake
└── libHelloWorld.so
```

Let’s create the directory tree first.

```bash
mkdir workspace_HelloWorld && cd workspace_HelloWorld
```

<a id="import-linked-libraries-and-its-dependencies"></a>

## Import linked libraries and its dependencies

The DDS application requires the Fast DDS, Fast CDR and Fast DDS Python bindings libraries.
Depending on the installation procedure followed, the process of making these libraries available for our DDS
application will be slightly different.

<!-- TODO Add when already python bindings are inside Fast-DDS
Installation from binaries and manual installation
""""""""""""""""""""""""""""""""""""""""""""""""""

If we have followed the installation from binaries or the manual installation, these libraries are already
accessible from the workspace.
On Linux, the header files can be found in directories `/usr/include/fastdds/` and
`/usr/include/fastcdr/` for Fast DDS and Fast CDR respectively. The compiled libraries of both can be found in
the directory `/usr/lib/`. -->

<a id="colcon-installation"></a>

### Colcon installation

From a Colcon installation there are several ways to import the libraries.
If the libraries need to be available just for the current session, run the following command.

```bash
source <path/to/Fast-DDS-python/workspace>/install/setup.bash
```

They can be made accessible from any session by adding the Fast DDS installation directory to your `$PATH`
variable in the shell configuration files for the current user running the following command.

```bash
echo 'source <path/to/Fast-DDS-python/workspace>/install/setup.bash' >> ~/.bashrc
```

This will set up the environment after each of this user’s logins.

<a id="writing-pubsub-python-datatype"></a>

<a id="build-the-topic-data-type"></a>

## Build the topic data type

*eProsima Fast DDS-Gen* is a Java application that generates source code using the data types defined in an
Interface Description Language (IDL) file.
This application can do two different things:

1. Generate C++ definitions for your custom topic.
2. Generate [SWIG](http://www.swig.org/) interface files to generate the Python bindings for your custom topic.

For this project, we will use the Fast DDS-Gen application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

```bash
touch HelloWorld.idl
```

This creates the HelloWorld.idl file.
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
All that remains is to generate the source code that implements this data type in C++11 and the
[SWIG](http://www.swig.org/) interface files for the Python bindings.
To do this, run the following command.

```bash
<path/to/Fast DDS-Gen>/scripts/fastddsgen -python HelloWorld.idl
```

This must have generated the following files:

* HelloWorld.hpp: HelloWorld C++ type definition.
* HelloWorld.i: [SWIG](http://www.swig.org/) interface file for HelloWorld C++ type definition.
* HelloWorldPubSubTypes.cxx: C++ interface used by Fast DDS to support HelloWorld type.
* HelloWorldPubSubTypes.hpp: C++ header file for HelloWorldPubSubTypes.cxx.
* HelloWorldPubSubTypes.i: [SWIG](http://www.swig.org/) interface file for C++ Serialization and Deserialization code.
* HelloWorldCdrAux.ipp: C++ serialization and deserialization code for the HelloWorld type.
* HelloWorldCdrAux.hpp: C++ header file for HelloWorldCdrAux.ipp.
* HelloWorldTypeObjectSupport.cxx: [TypeObject representation](../../xtypes/discovery_matching.md#xtypes-discovery-matching) registration code.
* HelloWorldTypeObjectSupport.hpp: Header file for HelloWorldTypeObjectSupport.cxx.
* CMakeLists.txt: CMake file to generate C++ source code and Python module from the [SWIG](http://www.swig.org/) interface files,
  compile and generate C++ libraries.

After that, the python bindings can be generated by running the following command.

```bash
cmake .
make
```

This must have generated the python bindings `HelloWorld.py` along with the corresponding the C++ libraries,
`_HelloWorldWrapper.so` and `libHelloWorld.so`

#### NOTE
If the user wants to use the bindings from any other directory, it is recommended to add the current path
to `LD_LIBRARY_PATH`

```bash
export LD_LIBRARY_PATH=$(pwd):$LD_LIBRARY_PATH
```

<a id="write-the-fast-dds-publisher"></a>

## Write the Fast DDS publisher

From the workspace, run the following command to download the HelloWorldPublisher.py file.

```bash
wget -O HelloWorldPublisher.py \
    https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/Python/HelloWorld/HelloWorldPublisher.py
```

This is the Python source code for the publisher application.
It is going to send 10 publications under the topic HelloWorldTopic.

```python
# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
HelloWorld Publisher
"""
from threading import Condition
import time

import fastdds
import HelloWorld

DESCRIPTION = """HelloWorld Publisher example for Fast DDS python bindings"""
USAGE = ('python3 HelloWorldPublisher.py')

class WriterListener (fastdds.DataWriterListener) :
    def __init__(self, writer) :
        self._writer = writer
        super().__init__()


    def on_publication_matched(self, datawriter, info) :
        if (0 < info.current_count_change) :
            print ("Publisher matched subscriber {}".format(info.last_subscription_handle))
            self._writer._cvDiscovery.acquire()
            self._writer._matched_reader += 1
            self._writer._cvDiscovery.notify()
            self._writer._cvDiscovery.release()
        else :
            print ("Publisher unmatched subscriber {}".format(info.last_subscription_handle))
            self._writer._cvDiscovery.acquire()
            self._writer._matched_reader -= 1
            self._writer._cvDiscovery.notify()
            self._writer._cvDiscovery.release()


class Writer:


    def __init__(self):
        self._matched_reader = 0
        self._cvDiscovery = Condition()
        self.index = 0

        factory = fastdds.DomainParticipantFactory.get_instance()
        self.participant_qos = fastdds.DomainParticipantQos()
        factory.get_default_participant_qos(self.participant_qos)
        self.participant = factory.create_participant(0, self.participant_qos)

        self.topic_data_type = HelloWorld.HelloWorldPubSubType()
        self.topic_data_type.set_name("HelloWorld")
        self.type_support = fastdds.TypeSupport(self.topic_data_type)
        self.participant.register_type(self.type_support)

        self.topic_qos = fastdds.TopicQos()
        self.participant.get_default_topic_qos(self.topic_qos)
        self.topic = self.participant.create_topic("HelloWorldTopic", self.topic_data_type.get_name(), self.topic_qos)

        self.publisher_qos = fastdds.PublisherQos()
        self.participant.get_default_publisher_qos(self.publisher_qos)
        self.publisher = self.participant.create_publisher(self.publisher_qos)

        self.listener = WriterListener(self)
        self.writer_qos = fastdds.DataWriterQos()
        self.publisher.get_default_datawriter_qos(self.writer_qos)
        self.writer = self.publisher.create_datawriter(self.topic, self.writer_qos, self.listener)


    def write(self):
        data = HelloWorld.HelloWorld()
        data.message("Hello World")
        data.index(self.index)
        self.writer.write(data)
        print("Sending {message} : {index}".format(message=data.message(), index=data.index()))
        self.index = self.index + 1


    def wait_discovery(self) :
        self._cvDiscovery.acquire()
        print ("Writer is waiting discovery...")
        self._cvDiscovery.wait_for(lambda : self._matched_reader != 0)
        self._cvDiscovery.release()
        print("Writer discovery finished...")


    def run(self):
        self.wait_discovery()
        for x in range(10) :
            time.sleep(1)
            self.write()
        self.delete()


    def delete(self):
        factory = fastdds.DomainParticipantFactory.get_instance()
        self.participant.delete_contained_entities()
        factory.delete_participant(self.participant)


if __name__ == '__main__':
    print('Starting publisher.')
    writer = Writer()
    writer.run()
    exit()
```

<a id="examining-the-code"></a>

### Examining the code

At the beginning of the file we import the Fast DDS Python bindings.

```python
import fastdds
```

and also the Python module generated by Fast-DDS-Gen as described in [Build the topic data type](#writing-pubsub-python-datatype) section.

```python
import HelloWorld
```

Then, the `WriterListener` class is defined by inheriting from the [`DataWriterListener`](../../python_api_reference/dds_pim/publisher/datawriterlistener.md#fastdds.DataWriterListener) class.
This class overrides the default DataWriter listener callbacks, which allows the execution of routines in case of an
event.
The overridden callback [`on_publication_matched()`](../../python_api_reference/dds_pim/publisher/datawriterlistener.md#fastdds.DataWriterListener.on_publication_matched)
allows the definition of a series of actions when a new DataReader
is detected listening to the topic under which the DataWriter is publishing.
The `info.current_count_change()` detects these changes of DataReaders that are matched to the
DataWriter.
This is a member in the [`MatchedStatus`](../../python_api_reference/dds_pim/core/status/matchedstatus.md#fastdds.MatchedStatus) structure that allows tracking changes in the status of
subscriptions.

```python
class WriterListener (fastdds.DataWriterListener) :
    def __init__(self, writer) :
        self._writer = writer
        super().__init__()


    def on_publication_matched(self, datawriter, info) :
        if (0 < info.current_count_change) :
            print ("Publisher matched subscriber {}".format(info.last_subscription_handle))
            self._writer._cvDiscovery.acquire()
            self._writer._matched_reader += 1
            self._writer._cvDiscovery.notify()
            self._writer._cvDiscovery.release()
        else :
            print ("Publisher unmatched subscriber {}".format(info.last_subscription_handle))
            self._writer._cvDiscovery.acquire()
            self._writer._matched_reader -= 1
            self._writer._cvDiscovery.notify()
            self._writer._cvDiscovery.release()
```

The next block creates the `Writer` class that implements a publisher.

```python
class Writer:
```

The publisher’s initialization member function of the `Writer` class are defined below.
This function performs several actions:

1. Uses the [`DomainParticipantFactory`](../../python_api_reference/dds_pim/domain/domainparticipantfactory.md#fastdds.DomainParticipantFactory) to create the participant.
2. Registers the data type defined in the IDL.
3. Creates the topic for the publications.
4. Creates the publisher.
5. Creates the DataWriter with the listener previously created.

```python
def __init__(self):
    self._matched_reader = 0
    self._cvDiscovery = Condition()
    self.index = 0

    factory = fastdds.DomainParticipantFactory.get_instance()
    self.participant_qos = fastdds.DomainParticipantQos()
    factory.get_default_participant_qos(self.participant_qos)
    self.participant = factory.create_participant(0, self.participant_qos)

    self.topic_data_type = HelloWorld.HelloWorldPubSubType()
    self.topic_data_type.set_name("HelloWorld")
    self.type_support = fastdds.TypeSupport(self.topic_data_type)
    self.participant.register_type(self.type_support)

    self.topic_qos = fastdds.TopicQos()
    self.participant.get_default_topic_qos(self.topic_qos)
    self.topic = self.participant.create_topic("HelloWorldTopic", self.topic_data_type.get_name(), self.topic_qos)

    self.publisher_qos = fastdds.PublisherQos()
    self.participant.get_default_publisher_qos(self.publisher_qos)
    self.publisher = self.participant.create_publisher(self.publisher_qos)

    self.listener = WriterListener(self)
    self.writer_qos = fastdds.DataWriterQos()
    self.publisher.get_default_datawriter_qos(self.writer_qos)
    self.writer = self.publisher.create_datawriter(self.topic, self.writer_qos, self.listener)
```

To make the publication, the public member function `write()` is implemented.
This is simply the writing of a change by the DataWriter object.

```python
    def write(self):
        data = HelloWorld.HelloWorld()
        data.message("Hello World")
        data.index(self.index)
        self.writer.write(data)
        print("Sending {message} : {index}".format(message=data.message(), index=data.index()))
        self.index = self.index + 1
```

To detect when a DataReader has matched, the public member function `wait_discovery()` is implemented.
In the DataWriter’s listener callback which states that the DataWriter has matched with a DataReader
that listens to the publication topic, the data member `_matched_reader` is updated.
It contains the number of DataReaders discovered.
Therefore, when the first DataReader has been discovered, the application starts to publish.

```python
def wait_discovery(self) :
    self._cvDiscovery.acquire()
    print ("Writer is waiting discovery...")
    self._cvDiscovery.wait_for(lambda : self._matched_reader != 0)
    self._cvDiscovery.release()
    print("Writer discovery finished...")
```

The public *run* function waits until a DataReader is discovered and executes the action of publishing 10 samples.

```python
def run(self):
    self.wait_discovery()
    for x in range(10) :
        time.sleep(1)
        self.write()
    self.delete()
```

Finally, the Writer is initialized and run in main.

```python
if __name__ == '__main__':
    print('Starting publisher.')
    writer = Writer()
    writer.run()
    exit()
```

<a id="write-the-fast-dds-subscriber"></a>

## Write the Fast DDS subscriber

From the workspace, run the following command to download the HelloWorldSubscriber.py file.

```bash
wget -O HelloWorldSubscriber.py \
    https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/Python/HelloWorld/HelloWorldSubscriber.py
```

This is the Python source code for the subscriber application.
The application runs a subscriber until the user press *Ctrl+C* receiving samples under the topic HelloWorldTopic.

```python
# Copyright 2022 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
HelloWorld Subscriber
"""
import signal

import fastdds
import HelloWorld

DESCRIPTION = """HelloWorld Subscriber example for Fast DDS python bindings"""
USAGE = ('python3 HelloWorldSubscriber.py')

# To capture ctrl+C
def signal_handler(sig, frame):
    print('Interrupted!')

class ReaderListener(fastdds.DataReaderListener):


    def __init__(self):
        super().__init__()


    def on_subscription_matched(self, datareader, info) :
        if (0 < info.current_count_change) :
            print ("Subscriber matched publisher {}".format(info.last_publication_handle))
        else :
            print ("Subscriber unmatched publisher {}".format(info.last_publication_handle))


    def on_data_available(self, reader):
        info = fastdds.SampleInfo()
        data = HelloWorld.HelloWorld()
        reader.take_next_sample(data, info)

        if info.valid_data:
            print("Received {message} : {index}".format(message=data.message(), index=data.index()))


class Reader:


    def __init__(self):
        factory = fastdds.DomainParticipantFactory.get_instance()
        self.participant_qos = fastdds.DomainParticipantQos()
        factory.get_default_participant_qos(self.participant_qos)
        self.participant = factory.create_participant(0, self.participant_qos)

        self.topic_data_type = HelloWorld.HelloWorldPubSubType()
        self.topic_data_type.set_name("HelloWorld")
        self.type_support = fastdds.TypeSupport(self.topic_data_type)
        self.participant.register_type(self.type_support)

        self.topic_qos = fastdds.TopicQos()
        self.participant.get_default_topic_qos(self.topic_qos)
        self.topic = self.participant.create_topic("HelloWorldTopic", self.topic_data_type.get_name(), self.topic_qos)

        self.subscriber_qos = fastdds.SubscriberQos()
        self.participant.get_default_subscriber_qos(self.subscriber_qos)
        self.subscriber = self.participant.create_subscriber(self.subscriber_qos)

        self.listener = ReaderListener()
        self.reader_qos = fastdds.DataReaderQos()
        self.subscriber.get_default_datareader_qos(self.reader_qos)
        self.reader = self.subscriber.create_datareader(self.topic, self.reader_qos, self.listener)


    def delete(self):
        factory = fastdds.DomainParticipantFactory.get_instance()
        self.participant.delete_contained_entities()
        factory.delete_participant(self.participant)


    def run(self):
        signal.signal(signal.SIGINT, signal_handler)
        print('Press Ctrl+C to stop')
        signal.pause()
        self.delete()


if __name__ == '__main__':
    print('Creating subscriber.')
    reader = Reader()
    reader.run()
    exit()
```

<a id="id1"></a>

### Examining the code

Since the source code of both the publisher and subscriber applications is mostly identical, this document will focus on
the main differences between them, omitting the parts of the code that have already been explained.

Following the same structure as in the publisher explanation, the first step is the implementation of the data reader
listener.
The first overridden callback of the ReaderListener is the [`on_subscription_matched()`](../../python_api_reference/dds_pim/subscriber/datareaderlistener.md#fastdds.DataReaderListener.on_subscription_matched),
which is the analog of the [`on_publication_matched()`](../../python_api_reference/dds_pim/publisher/datawriterlistener.md#fastdds.DataWriterListener.on_publication_matched) callback of the DataWriter.

```python
def on_subscription_matched(self, datareader, info) :
    if (0 < info.current_count_change) :
        print ("Subscriber matched publisher {}".format(info.last_publication_handle))
    else :
        print ("Subscriber unmatched publisher {}".format(info.last_publication_handle))
```

The second overridden callback is [`on_data_available()`](../../python_api_reference/dds_pim/subscriber/datareaderlistener.md#fastdds.DataReaderListener.on_data_available).
In this, the next received sample that the data reader can access is taken and processed to display its content.
It is here that the object of the [`SampleInfo`](../../python_api_reference/dds_pim/subscriber/sampleinfo.md#fastdds.SampleInfo) class is defined, which determines whether a sample has
already been read or taken.

```python
def on_data_available(self, reader):
    info = fastdds.SampleInfo()
    data = HelloWorld.HelloWorld()
    reader.take_next_sample(data, info)
```

The next line defines the `Reader` class that implements a subscriber.

```python
class Reader:
```

Next comes the subscriber initialization public member function.
This is the same as the initialization public member function defined for the `Writer`.

```python
def __init__(self):
    factory = fastdds.DomainParticipantFactory.get_instance()
    self.participant_qos = fastdds.DomainParticipantQos()
    factory.get_default_participant_qos(self.participant_qos)
    self.participant = factory.create_participant(0, self.participant_qos)

    self.topic_data_type = HelloWorld.HelloWorldPubSubType()
    self.topic_data_type.set_name("HelloWorld")
    self.type_support = fastdds.TypeSupport(self.topic_data_type)
    self.participant.register_type(self.type_support)

    self.topic_qos = fastdds.TopicQos()
    self.participant.get_default_topic_qos(self.topic_qos)
    self.topic = self.participant.create_topic("HelloWorldTopic", self.topic_data_type.get_name(), self.topic_qos)

    self.subscriber_qos = fastdds.SubscriberQos()
    self.participant.get_default_subscriber_qos(self.subscriber_qos)
    self.subscriber = self.participant.create_subscriber(self.subscriber_qos)

    self.listener = ReaderListener()
    self.reader_qos = fastdds.DataReaderQos()
    self.subscriber.get_default_datareader_qos(self.reader_qos)
    self.reader = self.subscriber.create_datareader(self.topic, self.reader_qos, self.listener)
```

The public member function `run()` ensures that the subscriber runs until the user press *Ctrl+C*.

```python
def run(self):
    signal.signal(signal.SIGINT, signal_handler)
    print('Press Ctrl+C to stop')
    signal.pause()
    self.delete()
```

Finally, the participant that implements a subscriber is initialized and run in main.

```python
if __name__ == '__main__':
    print('Creating subscriber.')
    reader = Reader()
    reader.run()
    exit()
```

<a id="putting-all-together"></a>

## Putting all together

Finally, from the build directory, run the publisher and subscriber applications from two terminals.

```bash
python3 HelloWorldPublisher.py
python3 HelloWorldSubscriber.py
```

<video width="100%" autoplay loop>
    <source src="../../../_static/simple_python_pubsub.mp4">
    Your browser does not support the video tag.
</video>

<a id="summary"></a>

## Summary

In this tutorial you have built a Python publisher and a subscriber DDS application.
You have also learned how to generate from an IDL file the specific Python module for your Topic data type.

<a id="next-steps"></a>

## Next steps

In the *eProsima Fast DDS* Github repository you will find more complex examples that implement DDS communication for
a multitude of use cases and scenarios. You can find them
[here](https://github.com/eProsima/Fast-DDS-python/tree/master/fastdds_python_examples).
