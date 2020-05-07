Write the Fast DDS publisher
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From the `src` directory in the workspace, run the following command to download the HelloWorldPublisher.cpp file.

.. code-block:: bash

    wget -O HelloWorldPublisher.cpp \
        https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp

Now you have the publisher's source code. The publisher is going to send 10 publications under the topic HelloWorld.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
  :language: C++
  :linenos:

Examine the code
"""""""""""""""""

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

*   DomainParticipantFactory.
    Allows for the creation and destruction of DomainParticipant objects.
*   DomainParticipant.
    Acts as a container for all other Entity objects and as a factory for the Publisher, Subscriber,
    and Topic objects.
*   TypeSupport.
    Provides the participant with the functions to serialize, deserialize and get the key of a
    specific data type.
*   Publisher.
    Is the object responsible for the creation of DataReaders.
*   DataWriter.
    Allows the application to set the value of the data to be published under a given Topic.
*   DataWriterListener.
    Allows the redefinition of the functions of the DataWriterListener.

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

It is now explained the private data members of the class that will be necessary to understand in order to discuss the
public member functions of the class.
First, the ``hello_`` data member is defined as an object of the HelloWorld class that defines the data type we created
with the IDL file.
Next, the private data members corresponding to the participant, publisher, topic, data writer and data type are
defined.
The ``type_`` object of the TypeSupport class is the object that will be used to register the topic data type in the
DomainParticipant.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 33-45

Then, it is defined the PubListener class by inheriting from the DataWriterListener class. This class override the
default data writer listener callbacks.
These allow us to execute routines in case of an event.
The overridden callback ``on_publication_matched`` allows you to define a series of actions when a new data writer
is detected listening to the topic under the data writer is publishing.
The ``info.current_count_change`` detects these changes of data readers that are matched to the
data writer.
This is a member in the MachedStatus structure that allows you to track changes in the status of subscriptions.
Finally, the ``listener_`` object of the class is defined.


.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 47-83

The public constructor and destructor of the class is defined below.
The constructor initializes the private data members of the class to ``nullptr`` except the TypeSupport object, that
initializes an object of the HelloWorldPubSubType class.
The class destructor removes these data members and thus cleans the system memory.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 87-111

Continuing with the public member functions of the class, the next snippet of code defines the public publisher's
initialization member function.
This function performs several actions:

1.  Initializes the content of the HelloWorld type ``hello_`` structure fields.
2.  Assigns a name to the participant through the QoS of the DomainParticipant.
3.  Uses the DomainParticipantFactory to create the participant.
4.  Registers the data type defined in the IDL.
5.  Creates the topic for the publications.
6.  Creates the publisher.
7.  Creates the DataWriter with the listener previously created.

As you can see, the QoS configuration for all entities, except for the participant's name, is the default configuration
(``PARTICIPANT_QOS_DEFAULT``, ``PUBLISHER_QOS_DEFAULT``, ``TOPIC_QOS_DEFAULT``, ``DATAWRITER_QOS_DEFAULT``).
The default value of the QoS of each DDS Entity can be checked in the
`DDS standard <https://www.omg.org/spec/DDS/About-DDS/>`_.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 113-155

To make the publication, the public member function ``publish()`` is implemented.
In the data writer's listener callback which states that the data writer has matched with a data reader
that listens to the publication topic, the data member ``matched_`` is updated. It contains the number of data readers
discovered.
Therefore, when the first data reader has been discovered, it starts to publish.
This is actually a ``write`` of a change of the DataWriter object.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 157-167

The public run function executes the action of publishing a given number of times.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 169-184

Finally, the HelloWorldPublisher is initialized and run in main.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 187-202

CMakeLists.txt
"""""""""""""""

Include at the end of the CMakeList.txt file you created earlier the following code snippet. This adds all the source
files needed to build the executable, and links the executable and the library together.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 47-48

At this point you can build, compile and run the publisher application. From the build directory in the workspace, run
the following commands.

.. code-block:: bash

    cmake ..
    make
    ./DDSHelloWorldPublisher
