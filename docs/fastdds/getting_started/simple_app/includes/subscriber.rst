Write the Fast DDS subscriber
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From the `src` directory in the workspace, execute the following command to download the HelloWorldSubscriber.cpp file.

.. code-block:: bash

    wget -O HelloWorldSubscriber.cpp \
        https://raw.githubusercontent.com/eProsima/Fast-RTPS-docs/master/code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp

Now you have the subscriber's source code. The application runs a subscriber until it receives 10 samples under the
topic HelloWorldTopic. At this point the subscriber stops.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :linenos:

Examining the code
""""""""""""""""""

As you have noticed, the source code to implement the subscriber is practically identical to the source code implemented
by the publisher.
Therefore, we will focus on the main differences between them, without explaining all the code again.

Following the same structure as in the publisher explanation, we start with the includes of the C++ header files.
In these, the files that include the publisher class are replaced by the subscriber class and the data writer class by
the data reader class.

*   |Subscriber-api|.
    It is the object responsible for the creation and configuration of DataReaders.
*   |DataReader-api|.
    It is the object responsible for the actual reception of the data.
    It registers in the application the topic (TopicDescription) that identifies the data to be read and
    accesses the data received by the subscriber.
*   |DataReaderListener-api|.
    This is the listener assigned to the data reader.
*   |DataReaderQoS-api|.
    Structure that defines the QoS of the DataReader.
*   |SampleInfo-api|.
    It is the information that accompanies each sample that is ‘read’ or ‘taken.’

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 22,29

The next line defines the :class:`HelloWorldSubscriber` class that implements a subscriber.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 33

Starting with the private data members of the class, it is worth mentioning the implementation of the data reader
listener.
The private data members of the class will be the participant, the subscriber, the topic, the data reader, and the
data type.
As it was the case with the data writer, the listener implements the callbacks to be executed in case an event
occurs.
The first overridden callback of the SubListener is the |DataReaderListener::on_subscription_matched-api|, which is the
analog of the |DataWriterListener::on_publication_matched-api| callback of the DataWriter.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 60-77
    :dedent: 8

The second overridden callback is |DataReaderListener::on_data_available-api|.
In this, the next received sample that the data reader can access is taken and processed to display its content.
It is here that the object of the |SampleInfo-api| class is defined, which determines whether a sample has already
been read or taken.
Each time a sample is read, the counter of samples received is increased.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 79-92
    :dedent: 8

The public constructor and destructor of the class is defined below.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 102-126
    :dedent: 4

Then we have the subscriber initialization public member function.
This is the same as the initialization public member function defined for the :class:`HelloWorldPublisher`.
The QoS configuration for all entities, except for the participant's name, is the default QoS
(|PARTICIPANT_QOS_DEFAULT-api|, |SUBSCRIBER_QOS_DEFAULT-api|, |TOPIC_QOS_DEFAULT-api|, |DATAREADER_QOS_DEFAULT-api|).
The default value of the QoS of each DDS Entity can be checked in the
`DDS standard <https://www.omg.org/spec/DDS/About-DDS/>`_.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 128-168
    :dedent: 4

The public member function :func:`run` ensures that the subscriber runs until all the samples have been received.
This member function implements an active wait of the subscriber, with a 100ms sleep interval to ease the CPU.


.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 170-178
    :dedent: 4

Finally, the participant that implements a subscriber is initialized and run in main.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 181-196

CMakeLists.txt
"""""""""""""""

Include at the end of the CMakeList.txt file you created earlier the following code snippet.
This adds all the source
files needed to build the executable, and links the executable and the library together.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/CMakeLists.txt
    :language: bash
    :lines: 50-51

At this point you can build, compile and run the subscriber application.
From the build directory in the workspace, run the following commands.

.. code-block:: bash

    cmake ..
    make clean && make
    ./DDSHelloWorldSubscriber
