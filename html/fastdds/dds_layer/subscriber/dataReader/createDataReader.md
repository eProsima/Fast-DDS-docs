<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-subscriber-datareader-creation"></a>

<a id="creating-a-datareader"></a>

# Creating a DataReader

A [DataReader](dataReader.md#dds-layer-subscriber-datareader) always belongs to a [Subscriber](../subscriber/subscriber.md#dds-layer-subscriber-subscriber).
Creation of a DataReader is done with the [`create_datareader()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber17create_datareaderEP16TopicDescriptionRK13DataReaderQosP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) member function on the
Subscriber instance, that acts as a factory for the DataReader.

Mandatory arguments are:

* A [Topic](../../topic/topic/topic.md#dds-layer-topic-topic) bound to the data type that will be transmitted.
* The [DataReaderQos](dataReader.md#dds-layer-subscriber-datareaderqos) describing the behavior of the DataReader.
  If the provided value is `DATAREADER_QOS_DEFAULT`,
  the value of the [Default DataReaderQos](dataReader.md#dds-layer-defaultdatareaderqos) is used.
  If the provided value is `DATAREADER_QOS_USE_TOPIC_QOS`,
  the values of the default QoS and the provided TopicQoS are used, whereby any policy
  that is set on the TopicQoS overrides the corresponding policy on the default QoS.

Optional arguments are:

* A Listener derived from [DataReaderListener](../dataReaderListener/dataReaderListener.md#dds-layer-subscriber-datareaderlistener), implementing the callbacks
  that will be triggered in response to events and state changes on the DataReader.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  DataReaderListener.
  By default all events are enabled.

[`create_datareader()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber17create_datareaderEP16TopicDescriptionRK13DataReaderQosP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

```c++
// Create a DataReader with default DataReaderQos and no Listener
// The value DATAREADER_QOS_DEFAULT is used to denote the default QoS.
DataReader* data_reader_with_default_qos =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader_with_default_qos)
{
    // Error
    return;
}

// A custom DataReaderQos can be provided to the creation method
DataReaderQos custom_qos;

// Modify QoS attributes
// (...)

DataReader* data_reader_with_custom_qos =
        subscriber->create_datareader(topic, custom_qos);
if (nullptr == data_reader_with_custom_qos)
{
    // Error
    return;
}

// Create a DataReader with default QoS and a custom Listener.
// CustomDataReaderListener inherits from DataReaderListener.
// The value DATAREADER_QOS_DEFAULT is used to denote the default QoS.
CustomDataReaderListener custom_listener;
DataReader* data_reader_with_default_qos_and_custom_listener =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT, &custom_listener);
if (nullptr == data_reader_with_default_qos_and_custom_listener)
{
    // Error
    return;
}

// Create a DataReader with default QoS and a custom TopicQos.
// The value DATAREADER_QOS_USE_TOPIC_QOS is used to denote the default QoS
// and to override the TopicQos.
Topic* topic =
        participant->create_topic("", "", TOPIC_QOS_DEFAULT);
DataReader* data_reader_with_default_qos_and_custom_topic_qos =
        subscriber->create_datareader(topic, DATAREADER_QOS_USE_TOPIC_QOS);
if (nullptr == data_reader_with_default_qos_and_custom_topic_qos)
{
    // Error
    return;
}
```

<a id="dds-layer-subscriber-datareader-creation-profile"></a>

<a id="profile-based-creation-of-a-datareader"></a>

## Profile based creation of a DataReader

Instead of using a DataReaderQos, the name of a profile
can be used to create a DataReader with the [`create_datareader_with_profile()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber30create_datareader_with_profileEP16TopicDescriptionRKNSt6stringEP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE)
member function on the Subscriber instance.

Mandatory arguments are:

* A Topic bound to the data type that will be transmitted.
* A string with the name that identifies the DataReader.

Optional arguments are:

* A Listener derived from DataReaderListener, implementing the callbacks
  that will be triggered in response to events and state changes on the DataReader.
  By default empty callbacks are used.
* A [`StatusMask`](../../../api_reference/dds_pim/core/status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) that activates or deactivates triggering of individual callbacks on the
  DataReaderListener.
  By default all events are enabled.

[`create_datareader_with_profile()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber30create_datareader_with_profileEP16TopicDescriptionRKNSt6stringEP18DataReaderListenerRK10StatusMaskNSt10shared_ptrIN7fastdds4rtps12IPayloadPoolEEE) will return a null pointer if there was an error during the operation,
e.g. if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

#### NOTE
XML profiles must have been loaded previously. See [Loading profiles from an XML file](../../domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory-load-profiles).

```c++
// First load the XML with the profiles
DomainParticipantFactory::get_instance()->load_XML_profiles_file("profiles.xml");

// Create a DataReader using a profile and no Listener
DataReader* data_reader_with_profile =
        subscriber->create_datareader_with_profile(topic, "data_reader_profile");
if (nullptr == data_reader_with_profile)
{
    // Error
    return;
}

// Create a DataReader using a profile and a custom Listener.
// CustomDataReaderListener inherits from DataReaderListener.
CustomDataReaderListener custom_listener;
DataReader* data_reader_with_profile_and_custom_listener =
        subscriber->create_datareader_with_profile(topic, "data_reader_profile", &custom_listener);
if (nullptr == data_reader_with_profile_and_custom_listener)
{
    // Error
    return;
}
```

<a id="dds-layer-subscriber-datareader-with-payload-pool-creation"></a>

<a id="creating-a-datawriter-with-a-custom-payloadpool"></a>

## Creating a DataWriter with a custom PayloadPool

A custom [PayloadPool](../../../rtps_layer/rtps_layer.md#rtps-layer-custom-payload-pool) can be passed as an argument during the creation of a
[DataReader](dataReader.md#dds-layer-subscriber-datareader).
This allows for customizing the management of the information exchanged between DataWriters and DataReaders.
The same configuration can be set in the
[opposite endpoint](../../publisher/dataWriter/createDataWriter.md#dds-layer-publisher-datawriter-with-payload-pool-creation).

```c++
// A DataReaderQos must be provided to the creation method
DataReaderQos qos;

// Create PayloadPool
std::shared_ptr<CustomPayloadPool> payload_pool = std::make_shared<CustomPayloadPool>();

DataReader* data_reader = subscriber->create_datareader(topic, qos, nullptr, StatusMask::all(), payload_pool);
if (nullptr == data_reader)
{
    // Error
    return;
}
```

This configuration can be performed also in the [RTPS layer](../../../rtps_layer/rtps_layer.md#rtps-layer-custom-payload-pool).
The [customization example](../../../rtps_layer/rtps_layer.md#rtps-layer-payload-pool-example) applies both layers.

<a id="dds-layer-subscriber-datareader-deletion"></a>

<a id="deleting-a-datareader"></a>

## Deleting a DataReader

A DataReader can be deleted with the [`delete_datareader()`](../../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10Subscriber17delete_datareaderEPK10DataReader) member function on the
[Subscriber](../subscriber/subscriber.md#dds-layer-subscriber-subscriber) instance where the DataReader was created.

#### NOTE
A DataReader can only be deleted if all Entities belonging to the DataReader
(QueryConditions) have already been deleted.
Otherwise, the function will issue an error and the DataReader will not be deleted.
This can be performed by using the [`delete_contained_entities()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader25delete_contained_entitiesEv) member function of the
[DataReader](dataReader.md#dds-layer-subscriber-datareader).

```c++
// Create a DataReader
DataReader* data_reader =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == data_reader)
{
    // Error
    return;
}

// Use the DataReader to communicate
// (...)

// Delete the entities the DataReader created
if (data_reader->delete_contained_entities() != RETCODE_OK)
{
    // DataReader failed to delete the entities it created.
    return;
}

// Delete the DataReader
if (subscriber->delete_datareader(data_reader) != RETCODE_OK)
{
    // Error
    return;
}
```
