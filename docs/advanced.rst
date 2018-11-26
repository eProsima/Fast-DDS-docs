Advanced Functionalities
========================


This section covers slightly more advanced, but useful features that enrich your implementation.


Topics and Keys
---------------

The RTPS standard contemplates the use of keys to define multiple data sources/sinks within a single topic.

There are three ways of implementing keys into your topic:

* Defining a `@Key` field in the IDL file when using FastRTPSGen (see the examples that come with the distribution).
* Manually implementing and using a :func:`getKey()` method.
* Adding the attribute `Key` to the member and its parents when using dynamic types (see :ref:`dynamic-types`).

Publishers and Subscribers using topics with keys must be configured to use them, otherwise, they will have no effect:

+--------------------------------------------+
| **C++**                                    |
+--------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp |
|    :language: c++                          |
|    :start-after: //CONF-QOS-KEY            |
|    :end-before: //!--                      |
+--------------------------------------------+
| **XML**                                    |
+--------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml  |
|    :language: xml                          |
|    :start-after: <!-->CONF-QOS-KEY         |
|    :end-before: <!--><-->                  |
+--------------------------------------------+

The RTPS Layer requires you to call the :func:`getKey()` method manually within your callbacks.

You can tweak the History to accommodate data from multiples keys based on your current configuration. This consist of defining a maximum number of data sinks and a maximum size for each sink:

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp             |
|    :language: c++                                      |
|    :start-after: //CONF-QOS-RESOURCELIMIT-INSTANCES    |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml              |
|    :language: xml                                      |
|    :start-after: <!-->CONF-QOS-RESOURCELIMIT-INSTANCES |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

Note that your History must be big enough to accommodate the maximum number of samples for each key. eProsima Fast RTPS will notify you if your History is too small.

.. _flow-controllers:

Flow Controllers
----------------

*eProsima Fast RTPS* supports user configurable flow controllers on a Publisher and Participant level. These
controllers can be used to limit the amount of data to be sent under certain conditions depending on the
kind of controller implemented.

The current release implement throughput controllers, which can be used to limit the total message throughput to be sent
over the network per time measurement unit. In order to use them, a descriptor must be passed into the Participant
or Publisher Attributes.

+-----------------------------------------------+
| **C++**                                       |
+-----------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp    |
|    :language: c++                             |
|    :start-after: //CONF-QOS-FLOWCONTROLLER    |
|    :end-before: //!--                         |
+-----------------------------------------------+
| **XML**                                       |
+-----------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml     |
|    :language: xml                             |
|    :start-after: <!-->CONF-QOS-FLOWCONTROLLER |
|    :end-before: <!--><-->                     |
+-----------------------------------------------+


In the Writer-Reader layer, the throughput controller is built-in and the descriptor defaults to infinite throughput.
To change the values:

.. code-block:: c++

    WriterAttributes WParams;
    WParams.throughputController.size = 300000; //300kb
    WParams.throughputController.timeMS = 1000; //1000ms

Note that specifying a throughput controller with a size smaller than the socket size can cause messages to never become sent.

Sending large data
------------------

The default message size *eProsima Fast RTPS* uses is a conservative value of 65kb.
If your topic data is bigger, it must be fragmented.

Fragmented messages are sent over multiple packets, as understood by the particular transport layer.
To make this possible, you must configure the Publisher to work in asynchronous mode.

+--------------------------------------------+
| **C++**                                    |
+--------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp |
|    :language: c++                          |
|    :start-after: //CONF-QOS-PUBLISHMODE    |
|    :end-before: //!--                      |
+--------------------------------------------+
| **XML**                                    |
+--------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml  |
|    :language: xml                          |
|    :start-after: <!-->CONF-QOS-PUBLISHMODE |
|    :end-before: <!--><-->                  |
+--------------------------------------------+

In the Writer-Subscriber layer, you have to configure the Writer:

.. code-block:: c++

    WriterAttributes Wparam;
    Wparam.mode = ASYNCHRONOUS_WRITER;    // Allows fragmentation

Note that in best-effort mode messages can be lost if you send big data too fast and the buffer is filled at a faster
rate than what the client can process messages.
On the other hand, in reliable mode, the existence of a lot of data fragments could decrease the frequency at which
messages are received.
If this happens, it can be resolved by increasing socket buffers size, as described in :ref:`tuning-socket-buffer`.
It can also help to set a lower Heartbeat period in reliable mode, as stated in :ref:`tuning-reliable-mode`.

When you are sending large data, it is convenient to setup a flow controller to avoid a burst of messages in the network
and increase performance.
See :ref:`flow-controllers`

Example: Sending a unique large file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This is a proposed example of how should the user configure its application in order to achieve the best performance. To make this example more tangible, it is going to be supposed that the file has a size of 9.9MB and the network in which the publisher and the subscriber are operating has a bandwidth of 100MB/s

First of all, the asynchronous mode has to be activated in the publisher parameters. Then, a suitable reliability mode has to be selected. In this case, it is important to make sure that all fragments of the message are received. The loss of a fragment means the loss of the entire message, so it would be best to choose the reliable mode.

The default message size of this fragments using the UDPv4 transport has a value of 65kb (which includes the space
reserved for the data and the message header).
This means that the publisher would have to write at least about 1100 fragments.

This amount of fragment could slow down the transmission, so it could be interesting to decrease the heartbeat period in order to increase the reactivity of the publisher.

Another important consideration is the addition of a flow controller. Without a flow controller, the publisher can occupy the entire bandwidth. A reasonable flow controller for this application could be a limit of 5MB/s, which represents only 5% of the total bandwidth. Anyway, these values are highly dependent on the specific application and its desired behaviour.

At last, there is another detail to have in mind: it is critical to check the size of the system UDP buffers. In Linux, buffers can be enlarged with

.. code-block:: bash

    sysctl -w net.ipv4.udp_mem="102400 873800 16777216"
    sysctl -w net.core.netdev_max_backlog="30000"
    sysctl -w net.core.rmem_max="16777216"
    sysctl -w net.core.wmem_max="16777216"


Example: Video streaming
^^^^^^^^^^^^^^^^^^^^^^^^

In this example, the target application transmits video between a publisher and a subscriber. This video will have a resolution of 640x480 and a frequency of 50fps.

As in the previous example, since the application is sending data that requires fragmentation, the asynchronous mode has to be activated in the publisher parameters.

In audio or video transmissions, sometimes is better to have a stable and high datarate feed than a 100% lossless communication. Working with a frequency of 50hz makes insignificant the loss of one or two samples each second. Thus, for a higher performance, it can be appropriate to configure the reliability mode to best-effort.


.. _discovery:

Discovery
---------

Fast RTPS provides a discovery mechanism that allows matching automatically publishers and subscribers. The discovery mechanism is divided into two phases: Participant Discovery Phase and Endpoints Discovery Phase.

* Participant Discovery Phase (PDP)
    Before discovering any entity of a remote participant, both participants have to meet between them. Participant Discovery
    Phase provides this step and is responsible for sending periodic information about itself. To know how to configure where to send this periodic information, see :ref:`initial-peers`. When both participants are met, is the turn of Endpoints
    Discovery Phase.

* Endpoints Discovery Phase (EDP)
    This phase is responsible for sending entities information to the remote participant. Also, it has to process the
    entities information of the remote participant and check which entities can match between them.

By default, the discovery mechanism is enabled, but you can disable it through participant attributes.

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp       |
|    :language: c++                                |
|    :start-after: //CONF-QOS-DISABLE-DISCOVERY    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml        |
|    :language: xml                                |
|    :start-after: <!-->CONF-QOS-DISABLE-DISCOVERY |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

Static Endpoints Discovery
^^^^^^^^^^^^^^^^^^^^^^^^^^

Endpoints Discovery Phase can be replaced by a static version that doesn't send any information. It is useful when
you have a limited network bandwidth and a well-known schema of publishers and subscribers. Instead of receiving entities
information for matching, this information is loaded from an XML file.

First of all, you have to disable the Endpoints Discovery Phase and enable the Static Endpoints Discovery. This can be done
from the participant attributes.

.. code-block:: c++

    ParticipantAttributes participant_attr;
    participant_attr.rtps.builtin.use_SIMPLE_EndpointDiscoveryProtocol = false;
    participant_attr.rtps.builtin.use_STATIC_EndpointDiscoveryProtocol = true;

Then, you will need to load the XML file containing the configuration of the remote participant. So, for example, if there
is a remote participant with a subscriber which is waiting to receive samples from your publisher, you will need to load
the configuration of this remote participant.

.. code-block:: c++

    participant_attr.rtps.builtin.setStaticEndpointXMLFilename("ParticipantWithASubscriber.xml");

A basic XML configuration file for this remote participant would contain information like the name of the remote participant, the topic name and
data type of the subscriber, and its entity and user-defined ID. All these values have to exactly match the parameter
values used to configure the remote participant (through the class :class:`ParticipantAttributes`) and its subscriber (through
the class :class:`SubscriberAttributes`). Missing elements will acquire default values. For example:

.. code-block:: xml

    <staticdiscovery>
        <participant>
            <name>HelloWorldSubscriber</name>
            <reader>
                <userId>3</userId>
                <entityId>4</userId>
                <topicName>HelloWorldTopic</topicName>
                <topicDataType>HelloWorld</topicDataType>
            </reader>
        </participant>
    </staticdiscovery>

The XML that configures the participant on the other side (in this case, a subscriber) could look like this:

.. code-block:: xml

    <staticdiscovery>
        <participant>
            <name>HelloWorldPublisher</name>
            <writer>
                <userId>1</userId>
                <entityId>2</userId>
                <topicName>HelloWorldTopic</topicName>
                <topicDataType>HelloWorld</topicDataType>
            </writer>
        </participant>
    </staticdiscovery>

You can find an example that uses `Static Endpoint Discovery <https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/StaticHelloWorldExample>`_.

The full list of fields for readers and writers includes the following parameters:

* **userId**: numeric value.
* **entityID**: numeric value.
* **expectsInlineQos**: *true* or *false*. **(only valid for readers)**
* **topicName**: text value.
* **topicDataType**: text value.
* **topicKind**: *NO_KEY* or *WITH_KEY*.
* **reliabilityQos**: *BEST_EFFORT_RELIABILITY_QOS* or *RELIABLE_RELIABILITY_QOS*.
* **unicastLocator**
    - address: text value.
    - port: numeric value.
* **multicastLocator**
    - address: text value.
    - port: numeric value.
* **topic**
    - name: text value.
    - data type: text value.
    - kind: text value.
* **durabilityQos**: *VOLATILE_DURABILITY_QOS*, *TRANSIENT_LOCAL_DURABILITY_QOS* or *TRANSIENT_DURABILITY_QOS*.
* **ownershipQos**
    - kind: *SHARED_OWNERSHIP_QOS* or *EXCLUSIVE_OWNERSHIP_QOS*.
* **partitionQos**: text value.
* **livelinessQos**
    - kind: *AUTOMATIC_LIVELINESS_QOS*, *MANUAL_BY_PARTICIPANT_LIVELINESS_QOS* or *MANUAL_BY_TOPIC_LIVELINESS_QOS*.
    - leaseDuration_ms: numeric value.


Subscribing to Discovery Topics
-------------------------------

As specified in the :ref:`discovery` section, the Participant or RTPS Participant has a series of meta-data endpoints
for use during the discovery process.
The participant listener interface includes methods which are called each time a Publisher or a Subscriber is discovered.
This allows you to create your own network analysis tools.

+--------------------------------------------------+
| **Implementation of custom listener**            |
+--------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp       |
|    :language: c++                                |
|    :start-after: //API-DISCOVERY-TOPICS-LISTENER |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **Setting the custom listener**                  |
+--------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp       |
|    :language: c++                                |
|    :start-after: //API-DISCOVERY-TOPICS-SET      |
|    :end-before: //!--                            |
+--------------------------------------------------+

The callbacks defined in the ReaderListener you attach to the EDP will execute for each data message after
the built-in protocols have processed it.

Tuning
-------

Taking advantage of multicast
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For topics with several subscribers, it is recommendable to configure them to use multicast instead of unicast.
By doing so, only one network package will be sent for each sample.
This will improve both CPU and network usage. Multicast configuration is explained in :ref:`multicast-locators`.

.. _tuning-socket-buffer:

Increasing socket buffers size
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In high rate scenarios or large data scenarios, the bottleneck could be the size of the socket buffers.
Network packages could be dropped because there is no space in the socket buffer.
Using Reliable :ref:`reliability` *Fast RTPS* will try to recover lost samples, but with the penalty of retransmission.
Using Best-Effort :ref:`reliability` samples will be definitely lost.

By default *eProsima Fast RTPS* creates socket buffers with the system default size, but you can modify it.
``sendSocketBufferSize`` attribute helps to increase the socket buffer used to send data.
``listenSocketBufferSize`` attribute helps to increase the socket buffer used to read data.

   +-------------------------------------------------------+
   | **C++**                                               |
   +-------------------------------------------------------+
   | .. literalinclude:: ../code/CodeTester.cpp            |
   |    :language: c++                                     |
   |    :start-after: //CONF-QOS-INCREASE-SOCKETBUFFERS    |
   |    :lines: 1-2                                        |
   +-------------------------------------------------------+
   | **XML**                                               |
   +-------------------------------------------------------+
   | .. literalinclude:: ../code/XMLTester.xml             |
   |    :language: xml                                     |
   |    :start-after: <!-->CONF-QOS-INCREASE-SOCKETBUFFERS |
   |    :lines: 1-6                                        |
   +-------------------------------------------------------+

Finding out system maximum values
"""""""""""""""""""""""""""""""""

Linux operating system sets a maximum value for socket buffer sizes.
When you set in *Fast RTPS* a socket buffer size, your value cannot exceed the maximum value of the system.

To get these values you can use the command ``sysctl``.
Maximum buffer size value of socket buffers used to send data could be retrieved using this command:

.. code-block:: bash

   $> sudo sysctl -a | grep net.core.wmem_max
   net.core.wmem_max = 1048576

For socket buffers used to receive data the command is:

.. code-block:: bash

   $> sudo sysctl -a | grep net.core.rmem_max
   net.core.rmem_max = 4194304

If these default maximum values are not enough for you, you can also increase them.

.. code-block:: bash

    $> echo 'net.core.wmem_max=12582912' >> /etc/sysctl.conf
    $> echo 'net.core.rmem_max=12582912' >> /etc/sysctl.conf

.. _tuning-reliable-mode:

Tuning Reliable mode
^^^^^^^^^^^^^^^^^^^^^

RTPS protocol can maintain reliable communication using special messages (Heartbeat and Ack/Nack messages).
RTPS protocol can detect which samples are lost and re-sent them again.

You can modify the frequency these special submessages are exchanged by specifying a custom heartbeat period.
The heartbeat period in the Publisher-Subscriber level is configured as part of the :class:`ParticipantAttributes`:

.. code-block:: c++

    PublisherAttributes pubAttr;
    pubAttr.times.heartbeatPeriod.seconds = 0;
    pubAttr.times.heartbeatPeriod.fraction = 4294967 * 500; //500 ms

In the Writer-Reader layer, this belongs to the :class:`WriterAttributes`:

.. code-block:: c++

    WriterAttributes Wattr;
    Wattr.times.heartbeatPeriod.seconds = 0;
    Wattr.times.heartbeatPeriod.fraction = 4294967 * 500; //500 ms

A smaller heartbeat period increases the number of overhead messages in the network,
but speeds up the system response when a piece of data is lost.

Non-strict reliability
""""""""""""""""""""""

Using a strict reliability, configuring :ref:`history-qos` kind as ``KEEP_ALL``, determinates all samples have to be
received by all subscribers.
This implicates a performance decrease in case a lot of samples are dropped.
If you don't need this strictness, use a non-strict reliability, i.e. configure :ref:`history-qos` kind as ``KEEP_LAST``.

Slow down sample rate
^^^^^^^^^^^^^^^^^^^^^

Sometimes publishers could send data in a too high rate for subscribers.
This can end dropping samples.
To avoid this you can slow down the rate using :ref:`flow-controllers`.

Whitelist Interfaces
-------------------------------------

There could be situations where you want to block some network interfaces to avoid connections or sending data through them.
This can be managed using the field *interface whitelist* in the transport descriptors,
and with them, you can set the interfaces you want to use to send or receive packets.
The values on this list should match the IPs of your machine in that networks.
For example:

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: ../code/CodeTester.cpp       |
|    :language: c++                                |
|    :start-after: //CONF-TRANSPORT-DESCRIPTORS    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: ../code/XMLTester.xml        |
|    :language: xml                                |
|    :start-after: <!-->CONF-TRANSPORT-DESCRIPTORS |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

Additional Quality of Service options
-------------------------------------

As a user, you can implement your own quality of service (QoS) restrictions in your application. *eProsima Fast RTPS*
comes bundled with a set of examples of how to implement common client-wise QoS settings:

* Deadline: Rise an alarm when the frequency of message arrival for a topic falls below a certain threshold.
* Ownership Strength: When multiple data sources come online, filter duplicates by focusing on the higher priority sources.
* Filtering: Filter incoming messages based on content, time, or both.

These examples come with their own `Readme.txt` that explains how the implementations work.


This marks the end of this document. We recommend you to take a look at the Doxygen API reference and
the embedded examples that come with the distribution. If you need more help, send us an email to `support@eprosima.com`.
