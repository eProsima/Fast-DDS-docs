<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-publisher-datawriter-creation"></a>

<a id="creating-a-datawriter"></a>

# Creating a DataWriter

A [DataWriter](dataWriter.md#dds-layer-publisher-datawriter) always belongs to a [Publisher](../publisher/publisher.md#dds-layer-publisher-publisher).
Creation of a DataWriter is done with the [`create_datawriter()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher17create_datawriterEP5TopicRK13DataWriterQosP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) member function on the
Publisher instance, that acts as a factory for the DataWriter.

Mandatory arguments are:

* A [Topic](../../topic/topic/topic.md#dds-layer-topic-topic) bound to the data type that will be transmitted.
* The [DataWriterQos](dataWriter.md#dds-layer-publisher-datawriterqos) describing the behavior of the DataWriter.
  If the provided value is `DATAWRITER_QOS_DEFAULT`,
  the value of the [Default DataWriterQos](dataWriter.md#dds-layer-defaultdatawriterqos) is used.
  If the provided value is `DATAWRITER_QOS_USE_TOPIC_QOS`,
  the values of the default QoS and the provided TopicQoS are used, whereby any policy
  that is set on the TopicQoS overrides the corresponding policy on the default QoS.

Optional arguments are:

* A Listener derived from [DataWriterListener](../dataWriterListener/dataWriterListener.md#dds-layer-publisher-datawriterlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the DataWriter.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  DataWriterListener.
  By default all events are enabled.

[`create_datawriter()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher17create_datawriterEP5TopicRK13DataWriterQosP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

```c++
// Create a DataWriter with default DataWriterQos and no Listener
// The value DATAWRITER_QOS_DEFAULT is used to denote the default QoS.
DataWriter* data_writer_with_default_qos =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer_with_default_qos)
{
    // Error
    return;
}

// A custom DataWriterQos can be provided to the creation method
DataWriterQos custom_qos;

// Modify QoS attributes
// (...)

DataWriter* data_writer_with_custom_qos =
        publisher->create_datawriter(topic, custom_qos);
if (nullptr == data_writer_with_custom_qos)
{
    // Error
    return;
}

// Create a DataWriter with default QoS and a custom Listener.
// CustomDataWriterListener inherits from DataWriterListener.
// The value DATAWRITER_QOS_DEFAULT is used to denote the default QoS.
CustomDataWriterListener custom_listener;
DataWriter* data_writer_with_default_qos_and_custom_listener =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT, &custom_listener);
if (nullptr == data_writer_with_default_qos_and_custom_listener)
{
    // Error
    return;
}

// Create a DataWriter with default QoS and a custom TopicQos.
// The value DATAWRITER_QOS_USE_TOPIC_QOS is used to denote the default QoS
// and to override the TopicQos.
Topic* topic =
        participant->create_topic("", "", TOPIC_QOS_DEFAULT);
DataWriter* data_writer_with_default_qos_and_custom_topic_qos =
        publisher->create_datawriter(topic, DATAWRITER_QOS_USE_TOPIC_QOS);
if (nullptr == data_writer_with_default_qos_and_custom_topic_qos)
{
    // Error
    return;
}
```

<a id="dds-layer-publisher-datawriter-creation-profile"></a>

<a id="profile-based-creation-of-a-datawriter"></a>

## Profile based creation of a DataWriter

Instead of using a DataWriterQos, the name of a profile
can be used to create a DataWriter with the [`create_datawriter_with_profile()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher30create_datawriter_with_profileEP5TopicRKNSt6stringEP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE)
member function on the Publisher instance.

Mandatory arguments are:

* A Topic bound to the data type that will be transmitted.
* A string with the name that identifies the DataWriter.

Optional arguments are:

* A Listener derived from DataWriterListener, implementing the callbacks
  that will be triggered in response to events and state changes on the DataWriter.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  DataWriterListener.
  By default all events are enabled.

[`create_datawriter_with_profile()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher30create_datawriter_with_profileEP5TopicRKNSt6stringEP18DataWriterListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) will return a null pointer if there was an error during the
operation, e.g. if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

#### NOTE
XML profiles must have been loaded previously. See [Loading profiles from an XML file](../../domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory-load-profiles).

```c++
// First load the XML with the profiles
DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

// Create a DataWriter using a profile and no Listener
DataWriter* data_writer_with_profile =
        publisher->create_datawriter_with_profile(topic, "data_writer_profile");
if (nullptr == data_writer_with_profile)
{
    // Error
    return;
}

// Create a DataWriter using a profile and a custom Listener.
// CustomDataWriterListener inherits from DataWriterListener.
CustomDataWriterListener custom_listener;
DataWriter* data_writer_with_profile_and_custom_listener =
        publisher->create_datawriter_with_profile(topic, "data_writer_profile", &custom_listener);
if (nullptr == data_writer_with_profile_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-publisher-datawriter-with-payload-pool-creation"></a>

<a id="creating-a-datawriter-with-a-custom-payloadpool"></a>

## Creating a DataWriter with a custom PayloadPool

A custom [PayloadPool](../../../rtps_layer/rtps_layer.md#rtps-layer-custom-payload-pool) can be passed as an argument during the creation of a
[DataWriter](dataWriter.md#dds-layer-publisher-datawriter).
This allows for customizing the management of the information exchanged between DataWriters and DataReaders.
The same configuration can be set in the
[opposite endpoint](../../subscriber/dataReader/createDataReader.md#dds-layer-subscriber-datareader-with-payload-pool-creation).

```c++
// A DataWriterQos must be provided to the creation method
DataWriterQos qos;

// Create PayloadPool
std::shared_ptr<eprosima::fastdds::rtps::IPayloadPool> payload_pool =
        std::dynamic_pointer_cast<eprosima::fastdds::rtps::IPayloadPool>(std::make_shared<CustomPayloadPool>());

DataWriter* data_writer = publisher->create_datawriter(topic, qos, nullptr, StatusMask::all(), payload_pool);
if (nullptr == data_writer)
{
    // Error
    return;
}
```

This configuration can be performed also in the [RTPS layer](../../../rtps_layer/rtps_layer.md#rtps-layer-custom-payload-pool).
The [customization example](../../../rtps_layer/rtps_layer.md#rtps-layer-payload-pool-example) applies both layers.

<a id="dds-layer-publisher-datawriter-deletion"></a>

<a id="deleting-a-datawriter"></a>

## Deleting a DataWriter

A DataWriter can be deleted with the [`delete_datawriter()`](../../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9Publisher17delete_datawriterEPK10DataWriter) member function on the
[Publisher](../publisher/publisher.md#dds-layer-publisher-publisher) instance where the DataWriter was created.

```c++
// Create a DataWriter
DataWriter* data_writer =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == data_writer)
{
    // Error
    return;
}

// Use the DataWriter to communicate
// (...)

// Delete the DataWriter
if (publisher->delete_datawriter(data_writer) != RETCODE_OK)
{
    // Error
    return;
}
```
