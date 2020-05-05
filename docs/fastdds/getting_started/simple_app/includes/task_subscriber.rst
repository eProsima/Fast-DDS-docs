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
The first overridden callback of the SubListener is the ``on_subscrition_matched``, which is the analog of the
``on_publication_matched`` callback of the DataWriter.

.. literalinclude:: /../code/Examples/C++/DDSHelloWorld/src/HelloWorldSubscriber.cpp
    :language: C++
    :lines: 140-157

The second overridden callback is ``on_data_available``.
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
