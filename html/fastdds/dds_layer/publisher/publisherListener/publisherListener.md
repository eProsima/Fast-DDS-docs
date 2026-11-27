<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-publisherlistener"></a>

<a id="publisherlistener"></a>

# PublisherListener

[`PublisherListener`](../../../api_reference/dds_pim/publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) is an abstract class defining the callbacks that will be triggered
in response to state changes on the [Publisher](../publisher/publisher.md#dds-layer-publisher-publisher).
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

[`PublisherListener`](../../../api_reference/dds_pim/publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) inherits from [DataWriterListener](../dataWriterListener/dataWriterListener.md#dds-layer-publisher-datawriterlistener).
Therefore, it has the ability to react to all events that are reported to the
[DataWriter](../dataWriter/dataWriter.md#dds-layer-publisher-datawriter).
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that [`PublisherListener`](../../../api_reference/dds_pim/publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) inherits from DataWriterListener
will only be called if the triggering DataWriter has
no Listener attached,
or if the callback is disabled by the [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) on the DataWriter.

[`PublisherListener`](../../../api_reference/dds_pim/publisher/publisherlistener.md#_CPPv4N8eprosima7fastdds3dds17PublisherListenerE) does not add any new callback.
Please, refer to the [DataWriterListener](../dataWriterListener/dataWriterListener.md#dds-layer-publisher-datawriterlistener) for the list of inherited callbacks
and override examples.
