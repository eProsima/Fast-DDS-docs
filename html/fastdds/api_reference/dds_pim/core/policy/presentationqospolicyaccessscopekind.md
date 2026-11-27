<a id="api-pim-presentationqospolicyaccessscopekind"></a>

<a id="presentationqospolicyaccessscopekind"></a>

# PresentationQosPolicyAccessScopeKind

### enum eprosima::fastdds::dds::PresentationQosPolicyAccessScopeKind

Enum PresentationQosPolicyAccessScopeKind, different kinds of Presentation Policy order for [PresentationQosPolicy](presentationqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1PresentationQosPolicy). 

*Values:*

### enumerator INSTANCE_PRESENTATION_QOS

Scope spans only a single instance. Indicates that changes to one instance need not be coherent nor ordered with respect to changes to any other instance. In other words, order and coherent changes apply to each instance separately. 

### enumerator TOPIC_PRESENTATION_QOS

Scope spans to all instances within the same [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)), but not across instances in different [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)). 

### enumerator GROUP_PRESENTATION_QOS

Scope spans to all instances belonging to [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)) entities within the same [Publisher](../../publisher/publisher_class.md#classeprosima_1_1fastdds_1_1dds_1_1Publisher) (or [Subscriber](../../subscriber/subscriber_class.md#classeprosima_1_1fastdds_1_1dds_1_1Subscriber)).
