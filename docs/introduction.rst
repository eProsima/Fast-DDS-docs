Getting Started
================


A brief introduction to the RTPS protocol
-----------------------------------------

At the top of RTPS, we find the Domain, which defines a separate plane of communication.
Several domains can coexist at the same time independently.
A domain contains any number of Participants, elements capable of sending and receiving data.
To do this, the participants use their Endpoints:

* Reader: Endpoint able to receive data.
* Writer: Endpoint able to send data.

A Participant can have any number of writer and reader endpoints.

.. image:: RTPS-structure.png

Communication revolves around Topics, which define the data being exchanged.
Topics don’t belong to any participant in particular; instead, all interested participants keep track of changes to the
topic data and make sure to keep each other up to date.
The unit of communication is called a Change, which represents an update to a topic.
Endpoints register these changes on their History, a data structure that serves as a cache for recent changes.
When you publish a change through a writer endpoint, the following steps happen behind the scenes:

* The change is added to the writer’s history cache.
* The writer informs any readers it knows about.
* Any interested (subscribed) readers request the change.
* After receiving data, readers update their history cache with the new change.

By choosing Quality of Service policies, you can affect how these history caches are managed in several ways, but the
communication loop remains the same. You can read more information in :ref:`configuration`.

Building your first application
-------------------------------

To build a minimal application, you must first define the topic. To define the data type of the topic Fast-RTPS offers
two different approaches, dynamically through :ref:`dynamic-types` and statically through
Interface Definition Language (IDL). In this example, we will define the data type statically with IDL,
you have more information about IDL in :ref:`fastrtpsgen-intro`.

Write an IDL file containing the specification you want. In this case, a single string is sufficient.

.. code-block:: idl

    // HelloWorld.idl
    struct HelloWorld
    {
        string msg;
    };

Now we need to translate this file to something Fast RTPS understands. For this we have a code generation tool called
fastrtpsgen (see :ref:`fastrtpsgen-intro`), which can do two different things:

* Generate C++ definitions for your custom topic.
* Optionally, generate a working example that uses your topic data.

You may want to check out the fastrtpsgen user manual, which comes with the distribution of the library.
But for now, the following commands will do:

On Linux: ::

    fastrtpsgen -example CMake HelloWorld.idl

On Windows: ::

    fastrtpsgen.bat -example CMake HelloWorld.idl

The `-example` option creates an example application, and the files needed to build it.

On Linux: ::

    mkdir build && cd build
    cmake ..
    make

On Windows: ::

    mkdir build && cd build
    cmake -G "Visual Studio 15 2017 Win64" ..
    cmake --build .

The application build can be used to spawn any number of publishers and subscribers associated with your topic.

On Linux: ::

    ./HelloWorldPublisherSubscriber publisher
    ./HelloWorldPublisherSubscriber subscriber

On Windows: ::

    HelloWorldPublisherSubscriber.exe publisher
    HelloWorldPublisherSubscriber.exe subscriber

You may need to set up a special rule in your Firewall for *eprosima Fast RTPS* to work correctly on Windows.

Each time you press <Enter\> on the Publisher, a new datagram is generated, sent over the network
and receiver by Subscribers currently online. If more than one subscriber is available, it can be seen that the
message is equally received on all listening nodes.

You can modify any values on your custom, IDL-generated data type before sending.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //PUBSUB_API_WRITE_SAMPLE
   :end-before: //!

Take a look at the `examples/` folder for ideas on how to improve this basic application through different configuration
options, and for examples of advanced Fast RTPS features.
