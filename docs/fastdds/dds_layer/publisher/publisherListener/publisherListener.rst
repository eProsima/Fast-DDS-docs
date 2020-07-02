.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_publisherListener:

PublisherListener
=================

|PublisherListener-api| is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_publisher_publisher`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

|PublisherListener-api| inherits from :ref:`dds_layer_publisher_dataWriterListener`.
Therefore, it has the ability to react to all events that are reported to the
:ref:`dds_layer_publisher_dataWriter`.
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that |PublisherListener-api| inherits from DataWriterListener
will only be called if the triggering DataWriter has
no Listener attached,
or if the callback is disabled by the |StatusMask-api| on the DataWriter.

|PublisherListener-api| does not add any new callback.
Please, refer to the :ref:`dds_layer_publisher_dataWriterListener` for the list of inherited callbacks
and override examples.

