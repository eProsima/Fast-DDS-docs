.. _use-case-largeData:

Large Data Rates
================

When the amount of data exchanged between a :ref:`dds_layer_publisher` and a :ref:`dds_layer_subscriber`
is large, some tuning may be required to compensate for side effects on the network and CPU load.
This large amount of data can be a result of the data types being large, a high message rate, or
a combination of both.

In this scenario, several limitations have to be taken into account:

* Network packages could be dropped because the transmitted amount of data fills the socket buffer
  before it can be processed.
  The solution is to :ref:`increase the buffers size<tuning-socket-buffer>`).

* It is also possible to limit the rate at which the :ref:`dds_layer_publisher` sends data using
  :ref:`flow-controllers`, in order to limit the effect of message bursts, and avoid to flood
  the :ref:`Subscribers<dds_layer_subscriber>` faster than they can process the messages.

* On *reliable* mode, the overall message rate can be affected due to the retransmission of lost packets.
  Selecting the Heartbeat period allows to tune between increased meta traffic or faster response to lost packets.
  See :ref:`tuning-heartbeat-period`.

* Also on *reliable* mode, with high message rates, the history of the :ref:`dds_layer_publisher_dataWriter`
  can be filled up, blocking the publication of new messages.
  A :ref:`non-strict reliable mode<tuning-nonstrict-reliability>` can be configured to avoid this blocking,
  at the cost of potentially losing some messages on some of the :ref:`Subscribers<dds_layer_subscriber>`.


.. warning::

   eProsima Fast DDS defines a conservative default message size of 64kB,
   which roughly corresponds to TCP and UDP payload sizes.
   If the topic data is bigger, it will automatically be be fragmented into several transport packets.

   The loss of a fragment means the loss of the entire message.
   This has most impact on *best-effort* mode, where the message loss
   probability increases with the number of fragments


.. _tuning-socket-buffer:

Increasing socket buffers size
------------------------------

In high rate scenarios or large data scenarios, network packages could be dropped because
the transmitted amount of data fills the socket buffer before it can be processed.
Using *reliable* :ref:`reliabilityqospolicy`, Fast DDS will try to recover lost samples, but with the penalty of
retransmission.
Using Best-Effort :ref:`reliabilityqospolicy` samples will be definitely lost.

By default eProsima Fast DDS creates socket buffers with the system default size.
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
   |    :lines: 1-6                                        |
   +-------------------------------------------------------+

Finding out system maximum values
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Linux operating system sets a maximum value for socket buffer sizes.
If the buffer sizes are tuned with :ref:`dds_layer_domainParticipantQos`, the values set
cannot exceed the maximum value of the system.

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

    $> echo 'net.core.wmem_max=12582912' >> /etc/sysctl.conf

For receiving sockets, the command is:

.. code-block:: bash

    $> echo 'net.core.rmem_max=12582912' >> /etc/sysctl.conf


.. _flow-controllers:

Flow Controllers
----------------

eProsima Fast DDS provides a mechanism to limit the total message rate at which the data is sent by a
:ref:`dds_layer_publisher_dataWriter`.
These controllers can be configured at :ref:`dds_layer_publisher_dataWriter` or :ref:`dds_layer_domainParticipant`
level.
On the :ref:`dds_layer_domainParticipant` the throughput controller is configured on the ``wire_protocol()`` field,
while the :ref:`dds_layer_publisher_dataWriterQos` uses the ``throughput_controller()`` field.

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
| .. literalinclude:: /../code/XMLTester.xml     |
|    :language: xml                              |
|    :start-after: <!-->CONF-QOS-FLOWCONTROLLER  |
|    :end-before: <!--><-->                      |
+------------------------------------------------+

.. Warning::

   Specifying a throughput controller with a size smaller than the socket size
   can cause the messages to never be sent.


.. _tuning-heartbeat-period:

Tuning Heartbeat Period
-----------------------

On *reliable* :ref:`reliabilityqospolicy`, RTPS protocol can detect which messages have been lost
and retransmit them again.
This mechanism is based on meta-traffic information exchanged between
:ref:`DataWriters<dds_layer_publisher_dataWriter>` and :ref:`DataReaders<dds_layer_subscriber_dataReader>`,
namely, Heartbeat and Ack/Nack messages.

A smaller Heartbeat period increases the CPU and network overhead, but speeds up the system response when
a piece of data is lost.
Therefore, users can customize the Heartbeat period to match their needs.
This can be done with the :ref:`dds_layer_publisher_dataWriterQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //CONF_QOS_TUNING_RELIABLE_WRITER
   :end-before: //!--
   :dedent: 8

.. _tuning-nonstrict-reliability:

Using Non-strict Reliability
----------------------------

When :ref:`historyqospolicykind` is set as ``KEEP_ALL``, all samples have to be received (and acknowledged)
by all subscribers before they can be discarded by the :ref:`dds_layer_publisher_dataWriter`.
If the message rate is high and the network is not reliable (i.e., lots of packets get lost), the history of the
:ref:`dds_layer_publisher_dataWriter` can be filled up, blocking the publication of new messages until any
of the old messages is acknowledged by all subscribers.

If this strictness is not needed, :ref:`historyqospolicykind` can be set as ``KEEP_LAST``.
In this case, when the history of the :ref:`dds_layer_publisher_dataWriter` is full, the oldest message that has not
been fully acknowledged yet is discarded to give room to the new one.
If any subscriber did not receive the discarded message, it will not be sent again and the subscriber
will be informed that the message is lost forever.


Practical Examples
------------------

.. _use-case-largeData-exampleFile:

Example: Sending a large file
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Consider the following scenario:

* A :ref:`dds_layer_publisher` needs to send a file with a size of 9.9MB.
* The network that connects :ref:`dds_layer_publisher` and :ref:`dds_layer_subscriber`
  has a bandwidth of 100MB/s

With a fragment size of 64 kB, the :ref:`dds_layer_publisher` has to send about 1100 fragments to send the whole file.
A possible configuration for this scenario could be:

* Using *reliable* reliability, since a losing a single fragment would mean the loss of the complete file.
* Decreasing the heartbeat period, in order to increase the reactivity of the :ref:`dds_layer_publisher`.
* Limiting the data rate using a :ref:`Flow Controller<flow-controllers>`,
  to avoid this transmission cannibalizing the whole bandwidth.
  A reasonable rate for this application could be 5MB/s, which represents only 5% of the total bandwidth.

.. _use-case-largeData-exampleStreaming:

Example: Video streaming
^^^^^^^^^^^^^^^^^^^^^^^^

In this scenario, the application transmits a video stream between a :ref:`dds_layer_publisher`
and a :ref:`dds_layer_subscriber`, at 50fps. In real-time audio or video transmissions,
it is usually preferred to have a high stable datarate feed, even at the cost of losing some
samples.
Losing one or two samples per second at 50fps is more acceptable than freezing the video waiting for the retransmission
of lost samples.
Therefore, in this case *best-effort* reliability can be appropriate.

