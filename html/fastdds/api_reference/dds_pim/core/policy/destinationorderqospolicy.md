<a id="api-pim-destinationorderqospolicy"></a>

<a id="destinationorderqospolicy"></a>

# DestinationOrderQosPolicy

### class DestinationOrderQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Controls the criteria used to determine the logical order among changes made by [Publisher](../../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) entities to the same instance of data (i.e., matching [Topic](../../topic/topic_class.md#classeprosima_1_1fastdds_1_1dds_1_1Topic) and key).

#### NOTE
Immutable Qos Policy 

#### WARNING
This [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) can be defined and is transmitted to the rest of the network but is not implemented in this version.

### Public Functions

### inline DestinationOrderQosPolicy()

Constructor. 

### virtual ~DestinationOrderQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [DestinationOrderQosPolicyKind](destinationorderqospolicykind.md#_CPPv4N8eprosima7fastdds3dds29DestinationOrderQosPolicyKindE) kind

DestinationOrderQosPolicyKind.

By default, BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS.
