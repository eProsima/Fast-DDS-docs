.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_dataReaderListener:

DataReaderListener
==================

|DataReaderListener-api| is an abstract class defining the callbacks
that will be triggered in response to state changes on the :ref:`dds_layer_subscriber_dataReader`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

DataReaderListener defines the following callbacks:

* |DataReaderListener::on_data_available-api|:
  There is new data available for the application on the DataReader.
  There is no queuing of invocations to this callback, meaning that if several new data changes are received
  at once, only one callback invocation may be issued for all of them, instead of one per change.
  If the application is retrieving the received data on this callback, it must keep
  :ref:`reading data<dds_layer_subscriber_accessreceived>` until no new changes are left.

* |DataReaderListener::on_subscription_matched-api|:
  The DataReader has found a
  :ref:`dds_layer_publisher_dataWriter` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  DataWriter that was previously considered to be matched.
  It is also triggered when a matched DataWriter has changed its
  :ref:`dds_layer_publisher_dataWriterQos`.

* |DataReaderListener::on_requested_deadline_missed-api|:
  The DataReader did not receive
  data within the deadline period configured on its :ref:`dds_layer_subscriber_dataReaderQos`.
  It will be called for each deadline period and data instance for which the
  DataReader missed data.

.. warning::
   Currently
   |DataReaderListener::on_requested_deadline_missed-api|
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.

* |DataReaderListener::on_requested_incompatible_qos-api|:
  The DataReader has found a
  DataWriter that matches the Topic and has
  a common partition, but with a QoS that is incompatible with the one defined on the
  DataReader.

* |DataReaderListener::on_liveliness_changed-api|:
  The liveliness status of a matched DataWriter has changed.
  Either a DataWriter that was *inactive* has become *active* or the other
  way around.

* |DataReaderListener::on_sample_rejected-api|:
  A received data sample was rejected.

.. warning::
   Currently
   |DataReaderListener::on_sample_rejected-api|
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.

* |DataReaderListener::on_sample_lost-api|:
  A data sample was lost and will never be received.

.. warning::
   Currently
   |DataReaderListener::on_sample_lost-api|
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.

.. important::

   For more information about callbacks and its hierarchy, please refer to
   :ref:`dds_layer_core_entity_commonchars_listener`.

.. literalinclude:: /../code/DDSCodeTester.cpp
  :language: c++
  :start-after: //DDS_DATAREADER_LISTENER_SPECIALIZATION
  :end-before: //!
