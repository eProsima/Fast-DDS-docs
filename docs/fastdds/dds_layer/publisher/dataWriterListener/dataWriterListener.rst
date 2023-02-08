.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_dataWriterListener:

DataWriterListener
==================

|DataWriterListener-api| is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_publisher_dataWriter`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

|DataWriterListener-api| defines the following callbacks:

* |DataWriterListener::on_publication_matched-api|: The DataWriter has found a
  :ref:`dds_layer_subscriber_dataReader` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  DataReader that was previously considered to be matched.

* |DataWriterListener::on_offered_deadline_missed-api|: The DataWriter failed to provide
  data within the deadline period configured on its :ref:`dds_layer_publisher_dataWriterQos`.
  It will be called for each deadline period and data instance for which the
  DataWriter failed to provide data.

* |DataWriterListener::on_offered_incompatible_qos-api|: The DataWriter has found a
  DataReader that matches the Topic and has
  a common partition, but with a requested QoS that is incompatible with the one defined on the
  DataWriter.

* |DataWriterListener::on_liveliness_lost-api|: The DataWriter did not respect the
  liveliness configuration on its DataWriterQos, and therefore,
  DataReader entities will consider the DataWriter
  as no longer *active*.

* |DataWriterListener::on_unacknowledged_sample_removed-api|: The Datawriter has removed a sample that has not been
  acknowledged by every matched DataReader.

.. _dds_layer_publisher_dataWriterListener_on_unack_sample_removed:

on_unacknowledged_sample_removed callback
-----------------------------------------

|DataWriterListener::on_unacknowledged_sample_removed-api| non-standard callback notifies the user when a sample has
been removed without being sent/received by the matched DataReaders.
This could happen in constrained networks or if the publication throughput is too demanding.
This callback can be used to detect these situations so the publishing application can apply some solution to ease this
issue like reducing the publication rate.

The criteria to consider that a sample has been removed without being acknowledged depends on the
:ref:`reliabilityqospolicy`:

* |BEST_EFFORT_RELIABILITY_QOS-api| DataWriters will notify that a sample has been removed while unacknowledged if the
  sample has not been sent through the transport.
* |RELIABLE_RELIABILITY_QOS-api| DataWriters consider samples to have been removed unacknowledged if not every matched
  DataReader has confirmed its reception by sending the corresponding meta-traffic ``ACK`` message.
  Consequently, a sample that is notified as removed unacknowledged might be received by one or more DataReaders, but
  not by every matched one, or at least, the ``ACK`` message has not been received at the moment of sample removal.
  A race condition is inevitable in this case, because when the sample is removed, the ``ACK`` from some matched
  DataReader is missing, but that means that it might have been lost in the transmission or that the message is still
  coming through and it will be received after the sample removal.
  Thus, this criteria may include false positives, but from the user's point of view, it is more meaningful to know when
  the sample has not been acknowledged by every matched DataReader even if some samples are erroneously notified.

A specific case must be considered for reliable DataWriters with :ref:`disablepositiveacksqospolicy` enabled.
This policy disables the sending of positive ``ACK`` messages, unless the sample has been lost in which case the matched
DataReader notifies the loss with a negative ``NACK`` message.
If no ``NACK`` has been received in the time defined in the QoS policy, the sample is considered to be received.
Again, this is prone to race conditions because the ``NACK`` message might be on its way or have been lost in the
network.
For this specific case, where ``ACK`` messages are not going to be received, the reliable DataWriter uses the same
criteria as the best effort DataWriter.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_LISTENER_SPECIALIZATION
   :end-before: //!

