Advanced Functionalities
########################


This section covers slightly more advanced, but useful features that enrich your implementation.


.. _topics-and-keys:

Topics and Keys
***************

The RTPS standard contemplates the use of keys to define multiple data sources/sinks within a single topic.

There are three ways of implementing keys into your topic:

* Defining a `@Key` field in the IDL file when using Fast DDS-Gen (see the examples that come with the distribution).
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
eProsima Fast DDS will notify you if your History is too small.

.. _flow-controllers:

Flow Controllers
****************

*eProsima Fast DDS* supports user configurable flow controllers on a Publisher and Participant level. These
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
Using Reliable :ref:`reliabilityqospolicy` *Fast DDS* will try to recover lost samples, but with the penalty of
retransmission.
Using Best-Effort :ref:`reliabilityqospolicy` samples will be definitely lost.

By default *eProsima Fast DDS* creates socket buffers with the system default size, but you can modify it.
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
When you set in *Fast DDS* a socket buffer size, your value cannot exceed the maximum value of the system.

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

As a user, you can implement your own quality of service (QoS) restrictions in your application. *eProsima Fast DDS*
comes bundled with a set of examples of how to implement common client-wise QoS settings:

* Ownership Strength: When multiple data sources come online, filter duplicates by focusing on the higher priority
  sources.
* Filtering: Filter incoming messages based on content, time, or both.

These examples come with their own `Readme.txt` that explains how the implementations work.
