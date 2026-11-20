<a id="api-pim-domainentity"></a>

<a id="domainentity"></a>

# DomainEntity

### class DomainEntity : public eprosima::fastdds::dds::[Entity](entity.md#_CPPv4N8eprosima7fastdds3dds6EntityE)

The [DomainEntity](#classeprosima_1_1fastdds_1_1dds_1_1DomainEntity) class is a subclass of [Entity](entity.md#classeprosima_1_1fastdds_1_1dds_1_1Entity) created in order to differentiate between DomainParticipants and the rest of Entities. 

Subclassed by [eprosima::fastdds::dds::DataReader](../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader), [eprosima::fastdds::dds::DataWriter](../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter), [eprosima::fastdds::dds::Publisher](../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher), [eprosima::fastdds::dds::Subscriber](../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber), [eprosima::fastdds::dds::Topic](../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic)

### Public Functions

### inline DomainEntity(const [StatusMask](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE) &mask = [StatusMask](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMaskE)::[all](status/statusmask.md#_CPPv4N8eprosima7fastdds3dds10StatusMask3allEv)())

Constructor. 

* **Parameters:**
  **mask** – [StatusMask](status/statusmask.md#classeprosima_1_1fastdds_1_1dds_1_1StatusMask) (default: all)
