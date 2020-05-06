Write the Fast DDS publisher
""""""""""""""""""""""""""""

From the `src` directory in the workspace, run the following command to download the HelloWorldPublisher.cpp file.

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
    :lines: 47-48

At this point you can build, compile and run the publisher application. From the build directory in the workspace, run
the following commands.

.. code-block:: bash

    cmake ..
    make
    ./DDSHelloWorldPublisher
