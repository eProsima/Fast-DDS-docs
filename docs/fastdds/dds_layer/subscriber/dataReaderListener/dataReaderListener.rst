.. _dds_layer_subscriber_dataReaderListener:

DataReaderListener
==================

:class:`DataReaderListener` is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_subscriber_dataReader`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

:class:`DataReaderListener` defines the following callbacks:

* **on_data_available**: There is new data available for the application on the :ref:`dds_layer_subscriber_dataReader`.

* **on_subscription_matched**: The :ref:`dds_layer_subscriber_dataReader` has found a
  :ref:`dds_layer_publisher_dataWriter` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  :ref:`dds_layer_publisher_dataWriter` that was previously considered to be matched.

* **on_requested_deadline_missed**: The :ref:`dds_layer_subscriber_dataReader` didn't receive
  data within the deadline period configured on its :ref:`dds_layer_subscriber_dataReaderQos`.
  It will be called for each deadline period and data instance for which the
  :ref:`dds_layer_subscriber_dataReader` missed data.

* **on_requested_incompatible_qos**: The :ref:`dds_layer_subscriber_dataReader` has found a
  :ref:`dds_layer_publisher_dataWriter` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition, but with a QoS that is incompatible with the one defined on the
  :ref:`dds_layer_subscriber_dataReader`.

.. note::
   Currently this callback is not implemented (it will never be called), and will be implemented
   on a future release of Fast DDS.

* **on_liveliness_changed**: The liveliness status of a matched :ref:`dds_layer_publisher_dataWriter`
  has changed.
  Either a :ref:`dds_layer_publisher_dataWriter` that was *inactive* has become *active* or the other
  way around.

* **on_sample_rejected**: A received data sample was rejected.

* **on_sample_lost**: A data sample was lost and will never be received.

.. literalinclude:: /../code/DDSCodeTester.cpp
  :language: c++
  :start-after: //DDS_DATAREADER_LISTENER_SPECIALIZATION
  :end-before: //!
