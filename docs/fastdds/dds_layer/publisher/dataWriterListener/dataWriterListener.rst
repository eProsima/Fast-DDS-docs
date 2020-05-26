.. _dds_layer_publisher_dataWriterListener:

DataWriterListener
==================

:class:`DataWriterListener` is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_publisher_dataWriter`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

:class:`DataWriterListener` defines the following callbacks:

* **on_publication_matched**: The :ref:`dds_layer_publisher_dataWriter` has found a
  :ref:`dds_layer_subscriber_dataReader` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition and a compatible QoS, or has ceased to be matched with a
  :ref:`dds_layer_subscriber_dataReader` that was previously considered to be matched.

* **on_offered_deadline_missed**: The :ref:`dds_layer_publisher_dataWriter` failed to provide
  data within the deadline period configured on its :ref:`dds_layer_publisher_dataWriterQos`.
  It will be called for each deadline period and data instance for which the
  :ref:`dds_layer_publisher_dataWriter` failed to provide data.

.. warning::
   Currently *on_offered_deadline_missed* is not implemented (it will never be called), and will be implemented
   on a future release of Fast DDS.

* **on_offered_incompatible_qos**: The :ref:`dds_layer_publisher_dataWriter` has found a
  :ref:`dds_layer_subscriber_dataReader` that matches the :ref:`dds_layer_topic_topic` and has
  a common partition, but with a requested QoS that is incompatible with the one defined on the
  :ref:`dds_layer_publisher_dataWriter`.

.. warning::
   Currently *on_offered_incompatible_qos* is not implemented (it will never be called), and will be implemented
   on a future release of Fast DDS.

* **on_liveliness_lost**: The :ref:`dds_layer_publisher_dataWriter` did not respect the
  liveliness configuration on its :ref:`dds_layer_publisher_dataWriterQos`, and therefore,
  :ref:`dds_layer_subscriber_dataReader` entities will consider the :ref:`dds_layer_publisher_dataWriter`
  as no longer *active*.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_LISTENER_SPECIALIZATION
   :end-before: //!

