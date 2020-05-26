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
   New data is available on any :ref:`dds_layer_subscriber_dataReader`
   belonging to this :ref:`dds_layer_subscriber_subscriber`.
   There is no queuing of invocations to this callback, meaning that if several new data changes are received
   at once, only one callback invocation may be issued for all of them, instead of one per change.
   If the application is retrieving the received data on this callback, it must keep
   :ref:`reading data<dds_layer_subscriber_accessreceived>` until no new changes are left.


.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_SUBSCRIBER_LISTENER_SPECIALIZATION
   :end-before: //!

