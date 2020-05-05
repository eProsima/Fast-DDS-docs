.. _writing_pubsub_app:

Writing a simple publisher and subscriber application
------------------------------------------------------

This section explains in detail how to create an simple Fast DDS application with a publisher and a subscriber
step by step.

.. contents::
    :local:
    :backlinks: none
    :depth: 2

Background
^^^^^^^^^^

DDS is a data-centric communications middleware that implements the DCPS model.
This model is based on the development of a publisher, a data generating element, and a subscriber, a
data consuming element.
These entities communicate by means of the topic, an element that binds both DDS entities.
Publishers generate information under a topic and subscribers subscribe to this same topic to receive information.

Prerequisites
^^^^^^^^^^^^^

First of all you need to follow the steps outlined in :ref:`installation_requirements`
for the installation of required packages, and in :ref:`installation-from-sources` for the installation of
*eprosima Fast DDS* and all its dependencies.
You also need to have completed the steps outlined in :ref:`compile-fastrtpsgen` for the installation of the
*eProsima FASTRTPSGEN* tool.

Tasks
^^^^^

Create the application workspace
""""""""""""""""""""""""""""""""

The application workspace will have the following structure at the end of the project.
Files ``build/DDSHelloWorldPublisher`` and ``build/DDSHelloWorldSubscriber`` are the Publisher application and
Subscriber application respectively.

.. code-block:: shell-session

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
            ├── HelloWorld.cxx
            ├── HelloWorld.h
            ├── HelloWorld.idl
            ├── HelloWorldPublisher.cpp
            ├── HelloWorldPubSubTypes.cxx
            ├── HelloWorldPubSubTypes.h
            └── HelloWorldSubscriber.cpp

Let's create the directory tree first.

.. code-block:: bash

    mkdir workspace_DDSHelloWorld && cd workspace_DDSHelloWorld
    mkdir src build

Import linked libraries and its dependencies
""""""""""""""""""""""""""""""""""""""""""""

The DDS application requires the Fast-RTPS and Fast-CDR libraries.
The way we will make these accessible from the
workspace depends on the installation procedure we have followed in :ref:`installation-from-sources`.

Manual installation
*******************

If we have followed a manual installation, these libraries are already accessible from the workspace.
On Linux, the header files can be found in directories `/usr/local/include/fastrtps/` and
`/usr/local/include/fastcdr/` for Fast RTPS and Fast CDR respectively. The compiled libraries of both can be found in
the directory `/usr/local/lib/`.

Colcon installation
*******************

If you have followed the Colcon installation there are several ways to import the libraries.
If you want these to be accessible only from the current shell session, run one of the following two commands.

.. code-block:: bash

    export PATH="<path/to/Fast-RTPS/workspace>/install:$PATH"

or,

.. code-block:: bash

    source <path/to/Fast-RTPS/workspace>/install/setup.bash

If you want these to be accessible from any session, you can add the Fast-RTPS installation directory to your ``$PATH``
variable in the shell configuration files running the following command.

.. code-block:: bash

    echo 'export PATH="<path/to/Fast-RTPS/workspace>/install:$PATH"' >> ~/.bashrc

Configure the CMake project
"""""""""""""""""""""""""""""

We will use the CMake tool to manage the building of the project.
With your preferred text editor create a new file called CMakeLists.txt and copy and paste the following code snippet.
Save this file in the root directory of your workspace. If you have followed these steps, it should
be `workspace_DDSHelloWorld`.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 15-42

In each section we will complete this file to include the specific generated files.

Build the topic data type
"""""""""""""""""""""""""

Briefly, *eProsima FASTRTPSGEN* is a Java application that generates source code using the data types defined in an
IDL file.
See :ref:`fastrtpsgen_intro` for further details.

For this project, we will use the FASTRTPSGEN application to define the data type of the messages that will be sent
by the publishers and received by the subscribers.

In the workspace directory, execute the following commands:

.. code-block:: bash

    cd src && touch HelloWorld.idl

This creates the HelloWorld.idl file in the ``src`` directory.
Open the file in your favorite text editor and copy and paste the following snippet of code.

.. code-block:: idl

    struct HelloWorld
    {
        unsigned long index;
        string message;
    };

With this we have defined the ``HelloWorld`` data type.
This has two elements, an *index* of type ``uint32_t`` and a *message* of type ``std::string``.

All that remains is to generate the source code that implements this data type in C++11. To do this, run the following
command from the ``src`` directory.

.. code-block:: bash

    <path/to/Fast-RTPS-Gen>/scripts/fastrtpsgen HelloWorld.idl

This must have generated the following files:

    * HelloWorld.cxx: HelloWorld type definition.
    * HelloWorld.h: Header file for HelloWorld.cxx.
    * HelloWorldPubSubTypes.cxx: Serialization and Deserialization code for the HelloWorld type.
    * HelloWorldPubSubTypes.h: Header file for HelloWorldPubSubTypes.cxx.

CMakeLists.txt
**************

Include at the end of the CMakeList.txt file you created earlier the following code snippet.
This includes the files we have just created.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 44-45

Write the Fast DDS publisher
""""""""""""""""""""""""""""

From the ``src`` directory in the workspace, run the following command to download the HelloWorldPublisher.cpp file.

.. code-block:: bash

    wget -O HelloWorldPublisher.cpp \
        https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp

Now you have the publisher's source code. The publisher is going to send 10 publications under the topic HelloWorld.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
  :language: C++
  :linenos:

Examine the code
****************

At the beginning of the file we have a Doxygen style comment block with the ``@file`` field that tells us the name of
the file.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 15-18

Below are the includes of the C++ headers.
The first one includes the HelloWorldPubSubTypes.h file with the serialization and deserialization functions of the
data type that we have defined in the previous section.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 20

The next block includes the C++ header files that allow the use of the Fast DDS API.

*   DomainParticipantFactory. Allow the creation and destruction of DomainParticipant objects.
*   DomainParticipant. Acts as a container for all other Entity objects and as a factory for the Publisher, Subscriber,
    and Topic objects.
*   TypeSupport. Provide the participant with the methods to serialize, deserialize and get the key of a
    specific data type.
*   Publisher. Is the object responsible for the actual dissemination of publications.
*   DataWriter. Allows the application to set the value of the data to be published under a given Topic.
*   DataWriterListener. It allows the redefinition of the functions of the DataWriterListener.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 22-27

Next, we define the namespace that contains the eProsima Fast DDS classes and functions that we are going to use in
our application.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 29

The next line creates the HelloWorldPublisher class that implements a publisher.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 31

The public constructor and destructor of the class is defined below. The fields of the class will be the participant,
the publisher, the topic, the data writer, and the data type.
The constructor initializes the private fields of the class to ``nullptr`` except the TypeSupport object, that
initializes an object of the HelloWorldPubSubType class.
The class destructor removes these fields and thus cleans the system memory.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 35-59

We will now explain the private fields of the class that will be necessary to understand in order to discuss the
remaining public methods of the class.
First, the ``hello_`` field is defined as an object of the HelloWorld class that defines the data type we created
with the IDL file.
Then the fields participant, publisher, topic and data writer are defined.
Finally the ``type_`` object of the TypeSupport class is defined.
This object will be used to register the topic data type in the DomainParticipant.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 134-146

Finally, it is defined the PubListener class by inheriting from the DataWriterListener class, to override the data
writer listener callbacks.
These allow us to execute routines in case of an event.
The overridden callback ``on_publication_matched`` allows you to define a series of actions when a new subscriber
is detected listening to the topic the publisher is using.
The ``info.current_count_change`` field allows us to detect these changes of subscribers that are matched to the
publisher.
This is a field in the MachedStatus structure that allows you to track changes in the status of subscriptions.
Finally, the ``listener_`` object of the class is defined.


.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 148-184

Back to the public methods of the class, the next line defines the public publisher's initialization function.
This method performs several actions:

# Initializes the content of the HelloWorld type ``hello_`` structure fields.
# Assigns a name to the participant through the QoS of the DomainParticipant.
# Use the DomainParticipantFactory to create the participant.
# Register the data type defined in the IDL.
# Create the topic for the publications.
# Create the publisher.
# Creates the DataWriter with the listener previously created.

As you can see, the QoS value for all entities, except for the participant's name, is the default value
(``PARTICIPANT_QOS_DEFAULT``, ``PUBLISHER_QOS_DEFAULT``, ``SUBSCRIBER_QOS_DEFAULT``, ``TOPIC_QOS_DEFAULT``,
``DATAWRITER_QOS_DEFAULT``, ``DATAREADER_QOS_DEFAULT``).
The default value of the QoS of each DDS Entity can be checked in the
`DDS standard <https://www.omg.org/spec/DDS/About-DDS/>`_.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 61-103

To make the publication, the publish method is implemented. To make the publication, the publish method is implemented.
This is actually a ``write`` of a change of the DataWriter object.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 105-115

The public run method executes the action of publishing a given number of times.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 117-132

Finally, the participant that implements a publisher is initialized and run in main.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 187-202

CMakeLists.txt
**************

Include at the end of the CMakeList.txt file you created earlier the following code snippet. This adds all the source
files needed to build the executable and links the executable and the library together.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 47, 49

At this point you can build, compile and run the publisher application. From the build directory in the workspace, run
the following commands.

.. code-block:: bash

    cmake ..
    make
    ./DDSHelloWorldPublisher

Write the Fast DDS subscriber
"""""""""""""""""""""""""""""

From the `src` directory in the workspace, execute the following command to download the HelloWorldPublisher.cpp file.

.. code-block:: bash

    wget -O HelloWorldSubscriber.cpp \
        https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp

Now you have the subscriber's source code. The application runs a subscriber until it receives 10 samples under the
topic HelloWorldTopic. At this point the subscriber stops.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :linenos:

Examine the code
****************

As you have noticed, the source code to implement the subscriber is practically identical to the source code implemented
by the publisher.
Therefore, we will focus on the main differences between them, without explaining all the code again.

Following the same structure as in the publisher explanation, we start with the includes of the C++ header files.
In these, the files that include the publisher class are replaced by the subscriber class and the data writer class by
the data reader class.

*   Subscriber. Is the object responsible for the actual reception of the data.
*   DataReader. It registers in the application the topic (TopicDescription) that identifies the data to be read and
    accesses the data received by the subscriber.
*   DataReaderListener. This is the listener assigned to the data reader.
*   DataReaderQoS. Structure that defines the QoS of the DataReader.
*   SampleInfo. Is the information that accompanies each sample that is ‘read’ or ‘taken.’

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 22,29

The next line defines the HelloWorldSubscriber class that implements a subscriber.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 33

The public constructor and destructor of the class is defined below. The fields of the class will be the participant,
the subscriber, the topic, the data reader, and the data type.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 37-61


As for the private fields of the class, it is worth mentioning the implementation of the DataReader listener.
As it was the case with the DataWriter, the listener implements the callbacks to be executed in case an event
occurs.
The first overriden callback of the SubListener is the ``on_subscrition_matched``, which is the analog of the
``on_publication_matched`` callback of the DataWriter.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 140-157

The second overriden callback is ``on_data_available``.
In this, the next received sample that the DataReader can access is taken and processed to display its content.
It is here that the object of the SampleInfo class is defined, which determines whether a sample has already been read
or taken.
Each time a sample is read, the counter of samples received is increased.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 159-172

Back to the public methods of the HelloWorldSubscriber class, we have the subscriber initialization method.
This is the same as the initialization method defined for the publisher.
However, in this one the DataReader QoS values are modified.
First the reliability of the DataReader is modified to be Reliable, namely RELIABLE_RELIABILITY_QOS.
This allows the reader to request the samples that have been lost in the transmission.
Also, we modify the durability of the DataReader which is labeled as Transient-Local, namely
TRANSIENT_LOCAL_DURABILITY_QOS.
The DataReader will request all the samples that the publisher has sent before the subscriber has been matched
with the publisher.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 94-98

Finally, the public method ``run()`` ensures that the subscriber runs until all the samples have been received.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 108-113

Finally, the participant that implements a subscriber is initialized and run in main.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 181-196

CMakeLists.txt
**************

Include at the end of the CMakeList.txt file you created earlier the following code snippet.
This adds all the source
files needed to build the executable and links the executable and the library together.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 51, 53

At this point you can build, compile and run the subscriber application.
From the build directory in the workspace, run the following commands.

.. code-block:: bash

    cmake ..
    make clean && make
    ./DDSHelloWorldSubscriber

Putting all together
""""""""""""""""""""

Finally, from the build directory, run the publisher and subscriber applications from various terminals.

.. code-block:: bash

    ./DDSHelloWorldPublisher
    ./DDSHelloWorldSubscriber

.. raw:: html

    <video width="1000" height="563" autoplay loop>
        <source src="../../../_static/simple_pubsub.mp4">
        Your browser does not support the video tag.
    </video>

Summary
^^^^^^^

In this tutorial you have built a publisher and a subscriber DDS application.
You have also learned how to build the CMake file for source code compilation, and how to include and use the Fast-RTPS
and Fast-CDR libraries in your project.

Next steps
^^^^^^^^^^

In the *eProsima Fast RTPS* Github repository you will find more complex examples that implement DDS communication for
a multitude of use cases and scenarios. You can find them
`here <https://github.com/eProsima/Fast-RTPS/tree/master/examples/C%2B%2B/DDS>`_.
