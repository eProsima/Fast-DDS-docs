<a id="api-pim-destinationorderqospolicykind"></a>

<a id="destinationorderqospolicykind"></a>

# DestinationOrderQosPolicyKind

### enum eprosima::fastdds::dds::DestinationOrderQosPolicyKind

Enum DestinationOrderQosPolicyKind, different kinds of destination order for [DestinationOrderQosPolicy](destinationorderqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DestinationOrderQosPolicy). 

*Values:*

### enumerator BY_RECEPTION_TIMESTAMP_DESTINATIONORDER_QOS

Indicates that data is ordered based on the reception time at each [Subscriber](../../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber). Since each subscriber may receive the data at different times there is no guaranteed that the changes will be seen in the same order. Consequently, it is possible for each subscriber to end up with a different final value for the data. 

### enumerator BY_SOURCE_TIMESTAMP_DESTINATIONORDER_QOS

Indicates that data is ordered based on a timestamp placed at the source (by the Service or by the application). In any case this guarantees a consistent final value for the data in all subscribers.
