Advanced Functionalities
########################


This section covers slightly more advanced, but useful features that enrich your implementation.


.. _topics-and-keys:

Topics and Keys
***************

The RTPS standard contemplates the use of keys to define multiple data sources/sinks within a single topic.

There are three ways of implementing keys into your topic:

* Defining a `@Key` field in the IDL file when using FastRTPSGen (see the examples that come with the distribution).
* Manually implementing and using a :func:`getKey()` method.
* Adding the attribute `Key` to the member and its parents when using dynamic types (see :ref:`dynamic-types`).

Publishers and Subscribers using topics with keys must be configured to use them, otherwise, they will have no effect:

+-------------------------------------------------+
| **C++**                                         |
+-------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp     |
|    :language: c++                               |
|    :start-after: //CONF-QOS-KEY                 |
|    :end-before: //!--                           |
+-------------------------------------------------+
| **XML**                                         |
+-------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml      |
|    :language: xml                               |
|    :start-after: <!-->CONF-QOS-KEY              |
|    :end-before: <!--><-->                       |
+-------------------------------------------------+

The RTPS Layer requires you to call the :func:`getKey()` method manually within your callbacks.

You can tweak the History to accommodate data from multiple keys based on your current configuration.
This consist of defining a maximum number of data sinks and a maximum size for each sink:

+--------------------------------------------------------+
| **C++**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp            |
|    :language: c++                                      |
|    :start-after: //CONF-QOS-RESOURCELIMIT-INSTANCES    |
|    :end-before: //!--                                  |
+--------------------------------------------------------+
| **XML**                                                |
+--------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml             |
|    :language: xml                                      |
|    :start-after: <!-->CONF-QOS-RESOURCELIMIT-INSTANCES |
|    :end-before: <!--><-->                              |
+--------------------------------------------------------+

Note that your History must be big enough to accommodate the maximum number of samples for each key.
eProsima Fast RTPS will notify you if your History is too small.

.. _flow-controllers:

Flow Controllers
****************

*eProsima Fast RTPS* supports user configurable flow controllers on a Publisher and Participant level. These
controllers can be used to limit the amount of data to be sent under certain conditions depending on the
kind of controller implemented.

The current release implement throughput controllers, which can be used to limit the total message throughput to be sent
over the network per time measurement unit. In order to use them, a descriptor must be passed into the Participant
or Publisher Attributes.

+-----------------------------------------------+
| **C++**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp   |
|    :language: c++                             |
|    :start-after: //CONF-QOS-FLOWCONTROLLER    |
|    :end-before: //!--                         |
+-----------------------------------------------+
| **XML**                                       |
+-----------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml    |
|    :language: xml                             |
|    :start-after: <!-->CONF-QOS-FLOWCONTROLLER |
|    :end-before: <!--><-->                     |
+-----------------------------------------------+


In the Writer-Reader layer, the throughput controller is built-in and the descriptor defaults to infinite throughput.
To change the values:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_RTPS_FLOWCONTROLLER
   :end-before: //!

Note that specifying a throughput controller with a size smaller than the socket size can cause messages to never become
sent.

Sending large data
******************

The default message size *eProsima Fast RTPS* uses is a conservative value of 65Kb.
If your topic data is bigger, it must be fragmented.

Fragmented messages are sent over multiple packets, as understood by the particular transport layer.
To make this possible, you must configure the Publisher to work in asynchronous mode.

+-------------------------------------------------------+
| **C++**                                               |
+-------------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp           |
|    :language: c++                                     |
|    :start-after: //CONF-QOS-PUBLISHMODE               |
|    :end-before: //!--                                 |
+-------------------------------------------------------+
| **XML**                                               |
+-------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml            |
|    :language: xml                                     |
|    :start-after: <!-->CONF-QOS-PUBLISHMODE            |
|    :end-before: <!--><-->                             |
+-------------------------------------------------------+

In the Writer-Subscriber layer, you have to configure the Writer:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_RTPS_PUBLISHMODE
   :end-before: //!

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
====================================

This is a proposed example of how should the user configure its application in order to achieve the best performance.
To make this example more tangible, it is going to be supposed that the file has a size of 9.9MB and the network in
which the publisher and the subscriber are operating has a bandwidth of 100MB/s

First of all, the asynchronous mode has to be activated in the publisher parameters.
Then, a suitable reliability mode has to be selected.
In this case, it is important to make sure that all fragments of the message are received.
The loss of a fragment means the loss of the entire message, so it would be best to choose the reliable mode.

The default message size of this fragments using the UDPv4 transport has a value of 65Kb (which includes the space
reserved for the data and the message header).
This means that the publisher would have to write at least about 1100 fragments.

This amount of fragment could slow down the transmission, so it could be interesting to decrease the heartbeat period
in order to increase the reactivity of the publisher.

Another important consideration is the addition of a flow controller.
Without a flow controller, the publisher can occupy the entire bandwidth.
A reasonable flow controller for this application could be a limit of 5MB/s, which represents only 5% of the total
bandwidth.
Anyway, these values are highly dependent on the specific application and its desired behavior.

At last, there is another detail to have in mind: it is critical to check the size of the system UDP buffers.
In Linux, buffers can be enlarged with

.. code-block:: bash

    sysctl -w net.ipv4.udp_mem="102400 873800 16777216"
    sysctl -w net.core.netdev_max_backlog="30000"
    sysctl -w net.core.rmem_max="16777216"
    sysctl -w net.core.wmem_max="16777216"


Example: Video streaming
========================

In this example, the target application transmits video between a publisher and a subscriber.
This video will have a resolution of 640x480 and a frequency of 50fps.

As in the previous example, since the application is sending data that requires fragmentation, the asynchronous mode
has to be activated in the publisher parameters.

In audio or video transmissions, sometimes is better to have a stable and high datarate feed than a 100% lossless
communication.
Working with a frequency of 50Hz makes insignificant the loss of one or two samples each second.
Thus, for a higher performance, it can be appropriate to configure the reliability mode to best-effort.


Subscribing to Discovery Topics
*******************************

As specified in the :ref:`discovery` section, the Participant or RTPS Participant has a series of meta-data endpoints
for use during the discovery process.
The participant listener interface includes methods which are called each time a Publisher or a Subscriber is
discovered.
This allows you to create your own network analysis tools.

+--------------------------------------------------+
| **Implementation of custom listener**            |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //API-DISCOVERY-TOPICS-LISTENER |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **Setting the custom listener**                  |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //API-DISCOVERY-TOPICS-SET      |
|    :end-before: //!--                            |
+--------------------------------------------------+

The callbacks defined in the ReaderListener you attach to the EDP will execute for each data message after
the built-in protocols have processed it.

.. _partitions:

Partitions
**********

Partitions introduce a logical entity isolation level concept inside the physical isolation induced by a Domain.
They represent another level to separate Publishers and Subscribers beyond Domain and Topic.
For a Publisher to communicate with a Subscriber, they have to belong at least to a common partition.
In this sense, partitions represent a light mechanism to provide data separation among Endpoints:

 * Unlike Domain and Topic, Partitions can be changed dynamically during the life cycle of the
   Endpoint with little cost.
   Specifically, no new threads are launched, no new memory is allocated, and the change history is not affected.
   Beware that modifying the Partition membership of endpoints will trigger the announcement
   of the new QoS configuration, and as a result, new Endpoint matching may occur,
   depending on the new Partition configuration.
   Changes on the memory allocation and running threads may occur due to the matching of remote Endpoints.

 * Unlike Domain and Topic, an Endpoint can belong to several Partitions at the same time.
   For certain data to be shared over different Topics, there must be a different Publisher for each Topic,
   each of them sharing its own history of changes.
   On the other hand, a single Publisher can share the same data over different Partitions using a single topic change,
   thus reducing network overload.

The Partition membership of an Endpoint can be configured on the :class:`qos.m_partitions` attribute of
the :class:`PublisherAttributes` or :class:`SubscriberAttributes` objects.
This attribute holds a list of Partition name strings.
If no Partition is defined for an Entity, it will be automatically included in the default nameless Partition.
Therefore, a Publisher and a Subscriber that specify no Partition will still be able to communicate through
the default Partition.

.. note::

   Partitions are linked to the Endpoint and not to the changes.
   This means that the Endpoint history is oblivious to modifications in the Partitions.
   For example, if a Publisher switches Partitions and afterwards needs to resend some older change again,
   it will deliver it to the new Partition set, regardless of which Partitions were defined
   when the change was created.
   This means that a late joiner Subscriber may receive changes that were created when another
   set of Partitions was active.

Wildcards in Partitions
=======================

Partition name entries can have wildcards following the naming conventions defined by the
POSIX ``fnmatch`` API (1003.2-1992 section B.6).
Entries with wildcards can match several names, allowing an Endpoint to easily be included in several Partitions.
Two Partition names with wildcards will match if either of them matches the other one according to ``fnmatch``.
That is, the matching is checked both ways.
For example, consider the following configuration:

 - A publisher with Partition ``part*``
 - A subscriber with Partition ``partition*``

Even though ``partition*`` does not match ``part*``, these publisher and subscriber will communicate
between them because ``part*`` matches ``partition*``.

Note that a Partition with name ``*`` will match any other partition **except the default Partition**.

Full example
============

Given a system with the following Partition configuration:

+----------------+---------+--------------------------------+
| Participant_1  | Pub_11  | {"Partition_1", "Partition_2"} |
+                +---------+--------------------------------+
|                | Pub_12  | {"*"}                          |
+----------------+---------+--------------------------------+
| Participant_2  | Pub_21  | {}                             |
+                +---------+--------------------------------+
|                | Pub_22  | {"Partition*"}                 |
+----------------+---------+--------------------------------+
| Participant_3  | Subs_31 | {"Partition_1"}                |
+                +---------+--------------------------------+
|                | Subs_32 | {"Partition_2"}                |
+                +---------+--------------------------------+
|                | Subs_33 | {"Partition_3"}                |
+                +---------+--------------------------------+
|                | Subs_34 | {}                             |
+----------------+---------+--------------------------------+

The endpoints will finally match the Partitions depicted on the following table.
Note that ``Pub_12`` does not match the default Partition.

+--------------+-------------------+-------------------+---------------------------------------+
|              | Participant_1     | Participant_2     | Participant_3                         |
|              +---------+---------+---------+---------+---------+---------+---------+---------+
|              | Pub_11  | Pub_12  | Pub_21  | Pub_22  | Subs_31 | Subs_32 | Subs_33 | Subs_34 |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| Partition_1  |    ✓    |    ✓    |    ✕    |    ✓    |    ✓    |    ✕    |    ✕    |    ✕    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| Partition_2  |    ✓    |    ✓    |    ✕    |    ✓    |    ✕    |    ✓    |    ✕    |    ✕    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| Partition_3  |    ✕    |    ✓    |    ✕    |    ✓    |    ✕    |    ✕    |    ✓    |    ✕    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+
| {default}    |    ✕    |    ✕    |    ✓    |    ✕    |    ✕    |    ✕    |    ✕    |    ✓    |
+--------------+---------+---------+---------+---------+---------+---------+---------+---------+

The following table provides the communication matrix for the given example:

+--------------------------+-------------------+-------------------+
|                          | Participant_1     | Participant_2     |
|                          +---------+---------+---------+---------+
|                          | Pub_11  | Pub_12  | Pub_21  | Pub_22  |
+----------------+---------+---------+---------+---------+---------+
| Participant_3  | Subs_31 |    ✓    |    ✓    |    ✕    |    ✓    |
+                +---------+---------+---------+---------+---------+
|                | Subs_32 |    ✓    |    ✓    |    ✕    |    ✓    |
+                +---------+---------+---------+---------+---------+
|                | Subs_33 |    ✕    |    ✓    |    ✕    |    ✓    |
+                +---------+---------+---------+---------+---------+
|                | Subs_34 |    ✕    |    ✕    |    ✓    |    ✕    |
+----------------+---------+---------+---------+---------+---------+

The following piece of code shows the set of parameters needed for the use case depicted in this example.


+-----------------------------------------------------+
| **C++**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp         |
|    :language: c++                                   |
|    :start-after: //CONF-QOS-PARTITIONS              |
|    :end-before: //!--                               |
+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-QOS-PARTITIONS           |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+


.. _intraprocess-delivery:

Intra-process delivery
**********************

*eProsima Fast RTPS* allows to speed up communications between entities within the same process by avoiding any of the
copy or send operations involved in the transport layer (either UDP or TCP).
This feature is enabled by default, and can be configured using :ref:`xml_profiles`.
Currently the following options are available:

* **INTRAPROCESS_OFF**: The feature is disabled.
* **INTRAPROCESS_USER_DATA_ONLY**: Discovery metadata keeps using ordinary transport.
* **INTRAPROCESS_FULL**: Default value. Both user data and discovery metadata using Intra-process delivery.

+-----------------------------------------------------+
| **XML**                                             |
+-----------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml          |
|    :language: xml                                   |
|    :start-after: <!-->CONF-LIBRARY-SETTINGS         |
|    :end-before: <!--><-->                           |
+-----------------------------------------------------+

Tuning
******

Taking advantage of multicast
=============================

For topics with several subscribers, it is recommendable to configure them to use multicast instead of unicast.
By doing so, only one network package will be sent for each sample.
This will improve both CPU and network usage. Multicast configuration is explained in :ref:`rtpsendpointqos`.

.. _tuning-socket-buffer:

Increasing socket buffers size
==============================

In high rate scenarios or large data scenarios, the bottleneck could be the size of the socket buffers.
Network packages could be dropped because there is no space in the socket buffer.
Using Reliable :ref:`reliabilityqospolicy` *Fast RTPS* will try to recover lost samples, but with the penalty of
retransmission.
Using Best-Effort :ref:`reliabilityqospolicy` samples will be definitely lost.

By default *eProsima Fast RTPS* creates socket buffers with the system default size, but you can modify it.
``sendSocketBufferSize`` attribute helps to increase the socket buffer used to send data.
``listenSocketBufferSize`` attribute helps to increase the socket buffer used to read data.

   +-------------------------------------------------------+
   | **C++**                                               |
   +-------------------------------------------------------+
   | .. literalinclude:: /../code/CodeTester.cpp           |
   |    :language: c++                                     |
   |    :start-after: //CONF-QOS-INCREASE-SOCKETBUFFERS    |
   |    :lines: 1-2                                        |
   +-------------------------------------------------------+
   | **XML**                                               |
   +-------------------------------------------------------+
   | .. literalinclude:: /../code/XMLTester.xml            |
   |    :language: xml                                     |
   |    :start-after: <!-->CONF-QOS-INCREASE-SOCKETBUFFERS |
   |    :lines: 1-6                                        |
   +-------------------------------------------------------+

Finding out system maximum values
---------------------------------

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
====================

RTPS protocol can maintain reliable communication using special messages (Heartbeat and Ack/Nack messages).
RTPS protocol can detect which samples are lost and re-sent them again.

You can modify the frequency these special submessages are exchanged by specifying a custom heartbeat period.
The heartbeat period in the Publisher-Subscriber level is configured as part of the :class:`ParticipantAttributes`:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_TUNING_RELIABLE_PUBLISHER
   :end-before: //!--

In the Writer-Reader layer, this belongs to the :class:`WriterAttributes`:

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_TUNING_RELIABLE_WRITER
   :end-before: //!--

A smaller heartbeat period increases the number of overhead messages in the network,
but speeds up the system response when a piece of data is lost.

Non-strict reliability
----------------------

Using a strict reliability, configuring :ref:`historyqospolicykind` kind as ``KEEP_ALL``, determines all samples have to
be received by all subscribers.
This implicates a performance decrease in case a lot of samples are dropped.
If you don't need this strictness, use a non-strict reliability, i.e. configure :ref:`historyqospolicykind` kind as
``KEEP_LAST``.

Slow down sample rate
=====================

Sometimes publishers could send data in a too high rate for subscribers.
This can end dropping samples.
To avoid this you can slow down the rate using :ref:`flow-controllers`.

.. _additionalQos:

Additional Quality of Service options
*************************************

As a user, you can implement your own quality of service (QoS) restrictions in your application. *eProsima Fast RTPS*
comes bundled with a set of examples of how to implement common client-wise QoS settings:

* Ownership Strength: When multiple data sources come online, filter duplicates by focusing on the higher priority
  sources.
* Filtering: Filter incoming messages based on content, time, or both.

These examples come with their own `Readme.txt` that explains how the implementations work.
