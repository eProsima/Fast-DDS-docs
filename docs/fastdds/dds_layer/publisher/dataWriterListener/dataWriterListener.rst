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

.. warning::
   Currently *on_offered_deadline_missed* is not implemented (it will never be called), and will be implemented
   on a future release of Fast DDS.

* |DataWriterListener::on_offered_incompatible_qos-api|: The DataWriter has found a
  DataReader that matches the Topic and has
  a common partition, but with a requested QoS that is incompatible with the one defined on the
  DataWriter.

* |DataWriterListener::on_liveliness_lost-api|: The DataWriter did not respect the
  liveliness configuration on its DataWriterQos, and therefore,
  DataReader entities will consider the DataWriter
  as no longer *active*.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DATAWRITER_LISTENER_SPECIALIZATION
   :end-before: //!

