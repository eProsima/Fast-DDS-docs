.. _dds_layer_subscriber_dataReaderListener:

DataReaderListener
==================

:ref:`api_pim_datareaderlistener` is an abstract class defining the callbacks
that will be triggered in response to state changes on the :ref:`dds_layer_subscriber_dataReader`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

:ref:`api_pim_datareaderlistener` defines the following callbacks:

* :cpp:func:`on_data_available<eprosima::fastdds::dds::DataReaderListener::on_data_available>`:
  There is new data available for the application on the :ref:`dds_layer_subscriber_dataReader`.
  There is no queuing of invocations to this callback, meaning that if several new data changes are received
  at once, only one callback invocation may be issued for all of them, instead of one per change.
  If the application is retrieving the received data on this callback, it must keep
  :ref:`reading data<dds_layer_subscriber_accessreceived>` until no new changes are left.

* :cpp:func:`on_subscription_matched<eprosima::fastdds::dds::DataReaderListener::on_subscription_matched>`:
  The :ref:`dds_layer_subscriber_dataReader` has found a
  :ref:`dds_layer_publisher_dataWriter` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  :ref:`dds_layer_publisher_dataWriter` that was previously considered to be matched.
  It is also triggered when a matched :ref:`dds_layer_publisher_dataWriter` has changed its
  :ref:`dds_layer_publisher_dataWriterQos`.

* :cpp:func:`on_requested_deadline_missed<eprosima::fastdds::dds::DataReaderListener::on_requested_deadline_missed>`:
  The :ref:`dds_layer_subscriber_dataReader` did not receive
  data within the deadline period configured on its :ref:`dds_layer_subscriber_dataReaderQos`.
  It will be called for each deadline period and data instance for which the
  :ref:`dds_layer_subscriber_dataReader` missed data.

.. note::
   Currently
   :cpp:func:`on_requested_deadline_missed<eprosima::fastdds::dds::DataReaderListener::on_requested_deadline_missed>`
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.

* :cpp:func:`on_requested_incompatible_qos<eprosima::fastdds::dds::DataReaderListener::on_requested_incompatible_qos>`:
  The :ref:`dds_layer_subscriber_dataReader` has found a
  :ref:`dds_layer_publisher_dataWriter` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition, but with a QoS that is incompatible with the one defined on the
  :ref:`dds_layer_subscriber_dataReader`.

.. note::
   Currently
   :cpp:func:`on_requested_incompatible_qos<eprosima::fastdds::dds::DataReaderListener::on_requested_incompatible_qos>`
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.

* :cpp:func:`on_liveliness_changed<eprosima::fastdds::dds::DataReaderListener::on_liveliness_changed>`:
  The liveliness status of a matched :ref:`dds_layer_publisher_dataWriter` has changed.
  Either a :ref:`dds_layer_publisher_dataWriter` that was *inactive* has become *active* or the other
  way around.

* :cpp:func:`on_sample_rejected<eprosima::fastdds::dds::DataReaderListener::on_sample_rejected>`:
  A received data sample was rejected.

.. note::
   Currently
   :cpp:func:`on_sample_rejected<eprosima::fastdds::dds::DataReaderListener::on_sample_rejected>`
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.

* :cpp:func:`on_sample_lost<eprosima::fastdds::dds::DataReaderListener::on_sample_lost>`:
  A data sample was lost and will never be received.

.. note::
   Currently
   :cpp:func:`on_sample_lost<eprosima::fastdds::dds::DataReaderListener::on_sample_lost>`
   is not implemented (it will never be called), and will be implemented on a future release of Fast DDS.


.. literalinclude:: /../code/DDSCodeTester.cpp
  :language: c++
  :start-after: //DDS_DATAREADER_LISTENER_SPECIALIZATION
  :end-before: //!
