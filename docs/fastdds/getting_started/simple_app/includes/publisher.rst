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

Examining the code
""""""""""""""""""

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

*   |DomainParticipantFactory-api|.
    Allows for the creation and destruction of DomainParticipant objects.
*   |DomainParticipant-api|.
    Acts as a container for all other Entity objects and as a factory for the Publisher, Subscriber,
    and Topic objects.
*   |TypeSupport-api|.
    Provides the participant with the functions to serialize, deserialize and get the key of a
    specific data type.
*   |Publisher-api|.
    It is the object responsible for the creation of DataWriters.
*   |DataWriter-api|.
    Allows the application to set the value of the data to be published under a given Topic.
*   |DataWriterListener-api|.
    Allows the redefinition of the functions of the DataWriterListener.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 22-27

Next, we define the namespace that contains the eProsima Fast DDS classes and functions that we are going to use in
our application.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 29

The next line creates the :class:`HelloWorldPublisher` class that implements a publisher.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 31

Continuing with the private data members of the class, the ``hello_`` data member is defined as an object of the
:class:`HelloWorld` class that defines the data type
we created with the IDL file.
Next, the private data members corresponding to the participant, publisher, topic, DataWriter and data type are
defined.
The ``type_`` object of the |TypeSupport-api| class is the object that will be used to register the topic data type
in the DomainParticipant.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 33-45

Then, the :class:`PubListener` class is defined by inheriting from the |DataWriterListener-api| class.
This class overrides the default DataWriter listener callbacks, which allow us to execute routines in case of an event.
The overridden callback |DataWriterListener::on_publication_matched-api|
allows you to define a series of actions when a new DataReader
is detected listening to the topic under which the DataWriter is publishing.
The :func:`info.current_count_change` detects these changes of DataReaders that are matched to the
DataWriter.
This is a member in the |MatchedStatus-api| structure that allows you to track changes in the status of subscriptions.
Finally, the ``listener_`` object of the class is defined as an instance of :class:`PubListener`.


.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 47-83
    :dedent: 4

The public constructor and destructor of the :class:`HelloWorldPublisher` class are defined below.
The constructor initializes the private data members of the class to ``nullptr``, with the exception of the TypeSupport
object, that is initialized as an instance of the :class:`HelloWorldPubSubType` class.
The class destructor removes these data members and thus cleans the system memory.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 87-111
    :dedent: 4

Continuing with the public member functions of the :class:`HelloWorldPublisher` class, the next snippet of code defines
the public publisher's initialization member function.
This function performs several actions:

1.  Initializes the content of the HelloWorld type ``hello_`` structure members.
2.  Assigns a name to the participant through the QoS of the DomainParticipant.
3.  Uses the |DomainParticipantFactory-api| to create the participant.
4.  Registers the data type defined in the IDL.
5.  Creates the topic for the publications.
6.  Creates the publisher.
7.  Creates the DataWriter with the listener previously created.

As you can see, the QoS configuration for all entities, except for the participant's name, is the default configuration
(|PARTICIPANT_QOS_DEFAULT-api|, |PUBLISHER_QOS_DEFAULT-api|, |TOPIC_QOS_DEFAULT-api|, |DATAWRITER_QOS_DEFAULT-api|).
The default value of the QoS of each DDS Entity can be checked in the
`DDS standard <https://www.omg.org/spec/DDS/About-DDS/>`_.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 113-155
    :dedent: 4

To make the publication, the public member function ``publish()`` is implemented.
In the DataWriter's listener callback which states that the DataWriter has matched with a DataReader
that listens to the publication topic, the data member ``matched_`` is updated. It contains the number of DataReaders
discovered.
Therefore, when the first DataReader has been discovered, the application starts to publish.
This is simply the `writing` of a change by the DataWriter object.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 157-167
    :dedent: 4

The public run function executes the action of publishing a given number of times, waiting for 1 second between
publications.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldPublisher.cpp
    :language: C++
    :lines: 169-184
    :dedent: 4

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
