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

.. include:: ./includes/tune_buffers.rst
.. include:: ./includes/flow_controllers.rst
.. include:: ./includes/congestion_control.rst
.. include:: ./includes/tune_heartbeat.rst
.. include:: ./includes/non_strict_reliability.rst
.. include:: ./includes/practical_examples.rst
