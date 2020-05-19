.. _dds_layer_subscriber_subscriberListener:

SubscriberListener
==================

:ref:`api_pim_subscriberlistener` is an abstract class defining the
callbacks that will be triggered in response to state changes on the
:ref:`dds_layer_subscriber_subscriber`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

:ref:`api_pim_subscriberlistener` inherits from :ref:`dds_layer_subscriber_dataReaderListener`.
Therefore, it has the ability to react to all events that are reported to the
:ref:`dds_layer_subscriber_dataReader`.
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that :ref:`api_pim_subscriberlistener` inherits from
:ref:`dds_layer_subscriber_dataReaderListener`
will only be called if the triggering :ref:`dds_layer_subscriber_dataReader` has
no Listener attached.

Additionally, :ref:`api_pim_subscriberlistener` adds the following callback:

 * :cpp:func:`on_data_on_readers<eprosima::fastdds::dds::SubscriberListener::on_data_on_readers>`:
   A new data value is available on any :ref:`dds_layer_subscriber_dataReader`
   belonging to this :ref:`dds_layer_subscriber_subscriber`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_SUBSCRIBER_LISTENER_SPECIALIZATION
   :end-before: //!

