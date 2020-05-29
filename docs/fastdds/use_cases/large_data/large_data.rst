.. _use-case-largeData:

Sending large data
==================

eProsima Fast DDS defines a conservative default message size of 64kB,
which roughly corresponds to TCP and UDP payload sizes.
If the topic data is bigger, it will be be fragmented into several transport packets.

.. warning::

   The loss of a fragment means the loss of the entire message.
   This has most impact on *best-effort* mode, where the message loss
   probability increases with the number of fragments

Note that in *best-effort* mode, messages can be lost if big data is sent too fast, due to the receiver buffer
being filled faster than the client can process messages.
In *reliable* mode no messages will be lost, but the overall message rate can be reduced due to the
re-sending of lost packets.

This problem can be alleviated increasing the size of the socket buffers,
as described in :ref:`tuning-socket-buffer`.
On *reliable* mode additional improvement can be obtained by setting a lower Heartbeat period,
as described in :ref:`tuning-reliable-mode`.

It may be also convenient to setup a flow controller when sending large data, to avoid message bursts in the network.
Refer to :ref:`flow-controllers`.

Example: Sending a large file
-----------------------------

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


Example: Video streaming
------------------------

In this scenario, the application transmits a video stream between a :ref:`dds_layer_publisher`
and a :ref:`dds_layer_subscriber`, at 50fps. In real-time audio or video transmissions,
it is usually preferred to have a high stable datarate feed, even at the cost of losing some
samples.
Losing one or two samples per second at 50fps is more acceptable than freezing the video waiting for the retransmission
of lost samples.
Therefore, in this case *best-effort* reliability can be appropriate.

