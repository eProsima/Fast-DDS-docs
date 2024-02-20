.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-largeData:

Large Data Rates
================

When the amount of data exchanged between a :ref:`dds_layer_publisher` and a :ref:`dds_layer_subscriber`
is large, some extra configuration may be required to compensate for side effects on the network and CPU load.
This large amount of data can be a result of the data types being large, a high message rate, or
a combination of both.

In this scenario, several approaches can be considered depending on the problem:

* For the cases in which the data samples are large (in the order of MB) such as transmitting raw video frames,
  point clouds, images, etc. between different hosts, TCP based communications may yield better reception rates
  with lower message loss, specially in the cases where a best effort transport layer is more susceptible to
  data loss, such as WiFi.
  To tackle these cases, :ref:`use-case-tcp` documents several ways to configure Fast DDS to communicate over TCP.

* Network packages could be dropped because the transmitted amount of data fills the socket buffer
  before it can be processed.
  The solution is to :ref:`increase the buffers size<tuning-socket-buffer>`.

* It is also possible to limit the rate at which the Publisher sends data using
  :ref:`flow-controllers`, in order to limit the effect of message bursts, and avoid to flood
  the Subscribers faster than they can process the messages.

* On |RELIABLE_RELIABILITY_QOS-api| mode,
  the overall message rate can be affected due to the retransmission of lost packets.
  Selecting the Heartbeat period allows to tune between increased meta traffic or faster response to lost packets.
  See :ref:`tuning-heartbeat-period`.

* Also on |RELIABLE_RELIABILITY_QOS-api| mode,
  with high message rates, the history of the :ref:`dds_layer_publisher_dataWriter`
  can be filled up, blocking the publication of new messages.
  A :ref:`non-strict reliable mode<tuning-nonstrict-reliability>` can be configured to avoid this blocking,
  at the cost of potentially losing some messages on some of the Subscribers.


.. warning::

   *eProsima Fast DDS* defines a conservative default message size of 64kB,
   which roughly corresponds to TCP and UDP payload sizes.
   If the topic data is bigger, it will automatically be fragmented into several transport packets.

.. warning::

   The loss of a fragment means the loss of the entire message.
   This has the most impact on |BEST_EFFORT_RELIABILITY_QOS-api| mode, where the message loss
   probability increases with the number of fragments


.. _tuning-socket-buffer:

Increasing socket buffers size
------------------------------

In high rate scenarios or large data scenarios, network packages can be dropped because
the transmitted amount of data fills the socket buffer before it can be processed.
Using |RELIABLE_RELIABILITY_QOS-api| mode,
*Fast DDS* will try to recover lost samples, but with the penalty of
retransmission.
With |BEST_EFFORT_RELIABILITY_QOS-api| mode,
samples will be definitely lost.

By default *eProsima Fast DDS* creates socket buffers with the system default size.
However, these sizes can be modified using the :ref:`dds_layer_domainParticipantQos`,
as shown in the example below.

+-------------------------------------------------------+
| **C++**                                               |
+-------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp        |
|    :language: c++                                     |
|    :start-after: //CONF-QOS-INCREASE-SOCKETBUFFERS    |
|    :end-before: //!--                                 |
|    :dedent: 8                                         |
+-------------------------------------------------------+
| **XML**                                               |
+-------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml            |
|    :language: xml                                     |
|    :start-after: <!-->CONF-QOS-INCREASE-SOCKETBUFFERS |
|    :end-before: <!--><-->                             |
|    :lines: 2-3,5-                                     |
|    :append: </profiles>                               |
+-------------------------------------------------------+

.. _finding-out-maximum-socket-values:

Finding out system maximum values
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Operating systems set a maximum value for socket buffer sizes.
If the buffer sizes are tuned with DomainParticipantQos, the values set
cannot exceed the maximum value of the system.

Linux
.....

The maximum buffer size values can be retrieved with the command ``sysctl``.
For socket buffers used to send data, use the following command:

.. code-block:: bash

   $> sudo sysctl -a | grep net.core.wmem_max
   net.core.wmem_max = 1048576

For socket buffers used to receive data the command is:

.. code-block:: bash

   $> sudo sysctl -a | grep net.core.rmem_max
   net.core.rmem_max = 4194304

However, these maximum values are also configurable and can be increased if needed.
The following command increases the maximum buffer size of sending sockets:

.. code-block:: bash

    $> sudo sysctl -w net.core.wmem_max=12582912

For receiving sockets, the command is:

.. code-block:: bash

    $> sudo sysctl -w net.core.rmem_max=12582912

Windows
.......

The following command changes the maximum buffer size of sending sockets:

.. code-block::

    C:\> reg add HKLM\SYSTEM\CurrentControlSet\services\AFD\Parameters /v DefaultSendWindow /t REG_DWORD /d 12582912

For receiving sockets, the command is:

.. code-block::

    C:\> reg add HKLM\SYSTEM\CurrentControlSet\services\AFD\Parameters /v DefaultReceiveWindow /t REG_DWORD /d 12582912


Increasing the Transmit Queue Length of an interface (Linux only)
-----------------------------------------------------------------

The Transmit Queue Length (``txqueuelen``) is a TCP/UDP/IP stack network interface value.
This value sets the number of packets allowed per kernel transmit queue of a network interface device.
By default, the ``txqueuelen`` value for Ethernet interfaces is set to ``1000`` in Linux.
This value is adequate for most Gigabit network devices.
However, in some specific cases, the ``txqueuelen`` setting should be increased to avoid overflows that drop packets.
Similarly, choosing a value that is too large can cause added overhead resulting in higher network latencies.

Note that this information only applies to the `sending` side, and not the `receiving` side.
Also increasing the ``txqueuelen`` should go together with increasing the buffer sizes of the UDP and/or TCP buffers.
(this must be applied for both the `sending` and `receiving` sides).

The settings for a specific network adapter can be viewed using the one of the following commands:

.. tabs::

  .. tab:: ``ip``

    .. code-block:: bash

      ip link show ${interface}

  .. tab:: ``ifconfig``

    .. code-block:: bash

        ifconfig ${interface}

This will display the configuration of the adapter, and among the parameters the ``txqueuelen``.
This parameter can be a value between 1000 and 20000.

.. important::

  If the ``ip`` command is used, the Transmit Queue Length parameter is called ``qlen``.

The ``txqueuelen`` can be modified for the current session using either the ``ifconfig`` or ``ip`` commands.
However, take into account that after rebooting the default values will be configured again.

.. tabs::

  .. tab:: ``ip``

    .. code-block:: bash

      ip link set txqueuelen ${value} dev ${interface}

  .. tab:: ``ifconfig``

    .. code-block:: bash

      ifconfig ${interface} txqueuelen ${size}

.. _flow-controllers:

Flow Controllers
----------------

*eProsima Fast DDS* provides a mechanism to limit the rate at which the data is sent by a DataWriter.
These controllers should be registered on the creation of the DomainParticipant using |FlowControllersQos|, and
then referenced on the creation of the DataWriter using |PublishModeQosPolicy|.

A new thread is spawned the first time a flow controller is referenced by an asynchronous DataWriter.
This thread will be responsible for arbitrating the network output of the samples being transmitted by all the
DataWriters referencing the same flow controller.

Flow controllers should be given a name so they can later on be referenced by the DataWriters.
A default, unlimited, |FIFO_SCHED_POLICY-api| flow controller is always available with name
|FASTDDS_FLOW_CONTROLLER_DEFAULT-api|.

Scheduling policy
^^^^^^^^^^^^^^^^^

There are different kinds of flow controllers, depending on the scheduling policy used.
All of them will limit the number of bytes sent to the network to no more than
|FlowControllerDescriptor::max_bytes_per_period-api| bytes during |FlowControllerDescriptor::period_ms-api|
milliseconds.
They only differ in the way they decide the order in which the samples are sent.

* |FIFO_SCHED_POLICY-api| will output samples on a first come, first served order.

* |ROUND_ROBIN_SCHED_POLICY-api| will output one sample from each DataWriter in circular order.

* |HIGH_PRIORITY_SCHED_POLICY-api| will output samples from DataWriters with the highest priority first.
  The priority of a DataWriter is configured using property ``fastdds.sfc.priority``.
  Allowed values are from -10 (highest priority) to 10 (lowest priority).
  If the property is not present, it will be set to the lowest priority.
  Samples for DataWriters with the same priority are handled with FIFO order.

* |PRIORITY_WITH_RESERVATION_SCHED_POLICY-api| works as the previous one, but allows the DataWriters to reserve
  part of the output bandwidth.
  This is done with the property ``fastdds.sfc.bandwidth_reservation``.
  Allowed values are from 0 to 100, and express a percentage of the total flow controller limit.
  If the property is not present, it will be set to 0 (no bandwidth is reserved for the DataWriter).
  After the reserved bandwidth has been consumed, the rest of the samples will be handled with the rules of
  |HIGH_PRIORITY_SCHED_POLICY-api|.


Example configuration
^^^^^^^^^^^^^^^^^^^^^

+------------------------------------------------+
| **C++**                                        |
+------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp |
|    :language: c++                              |
|    :start-after: //CONF-QOS-FLOWCONTROLLER     |
|    :end-before: //!--                          |
|    :dedent: 8                                  |
+------------------------------------------------+
| **XML**                                        |
+------------------------------------------------+
| There is currently no way of configuring flow  |
| controllers with XML.                          |
| This will be added in future releases of the   |
| product                                        |
+------------------------------------------------+

.. Warning::

   Specifying a flow controller with a size smaller than the transport buffer size
   can cause the messages to never be sent.


.. _tuning-heartbeat-period:

Tuning Heartbeat Period
-----------------------

On |RELIABLE_RELIABILITY_QOS-api| (:ref:`reliabilityqospolicy`), RTPS protocol can detect which messages have been lost
and retransmit them.
This mechanism is based on meta-traffic information exchanged between
DataWriters and DataReaders,
namely, Heartbeat and Ack/Nack messages.

A smaller Heartbeat period increases the CPU and network overhead, but speeds up the system response when
a piece of data is lost.
Therefore, users can customize the Heartbeat period to match their needs.
This can be done with the DataWriterQos.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_TUNING_RELIABLE_WRITER
   :end-before: //!--
   :dedent: 8

.. _tuning-nonstrict-reliability:

Using Non-strict Reliability
----------------------------

When :ref:`historyqospolicykind` is set as |KEEP_ALL_HISTORY_QOS-api|, all samples have to be received
(and acknowledged) by all subscribers before they can be overridden by the DataWriter.
If the message rate is high and the network is not reliable (i.e., lots of packets get lost), the history of the
DataWriter can be filled up, blocking the publication of new messages until any
of the old messages is acknowledged by all subscribers.

If this strictness is not needed, :ref:`historyqospolicykind` can be set as |KEEP_LAST_HISTORY_QOS-api|.
In this case, when the history of the DataWriter is full, the oldest message that has not
been fully acknowledged yet is overridden with the new one.
If any subscriber did not receive the discarded message, the publisher
will send a GAP message to inform the subscriber that the message is lost forever.


Practical Examples
------------------

.. _use-case-largeData-exampleFile:

Example: Sending a large file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Consider the following scenario:

* A Publisher needs to send a file with a size of 9.9 MB.
* The Publisher and Subscriber are connected through a network
  with a bandwidth of 100 MB/s

With a fragment size of 64 kB, the Publisher has to send about 1100 fragments to send the whole file.
A possible configuration for this scenario could be:

* Using |RELIABLE_RELIABILITY_QOS-api|,
  since a losing a single fragment would mean the loss of the complete file.
* Decreasing the heartbeat period, in order to increase the reactivity of the Publisher.
* Limiting the data rate using a :ref:`Flow Controller<flow-controllers>`,
  to avoid this transmission cannibalizing the whole bandwidth.
  A reasonable rate for this application could be 5 MB/s, which represents only 5% of the total bandwidth.

.. note::

   Using :ref:`transport_sharedMemory_sharedMemory` the only limit to the fragment size is the available memory.
   Therefore, all fragmentation can be avoided in SHM by increasing the size of the shared buffers.

.. _use-case-largeData-exampleStreaming:

Example: Video streaming
^^^^^^^^^^^^^^^^^^^^^^^^

In this scenario, the application transmits a video stream between a Publisher
and a Subscriber, at 50 fps. In real-time audio or video transmissions,
it is usually preferred to have a high stable datarate feed, even at the cost of losing some
samples.
Losing one or two samples per second at 50 fps is more acceptable than freezing the video waiting for the retransmission
of lost samples.
Therefore, in this case |BEST_EFFORT_RELIABILITY_QOS-api| can be appropriate.
