<a id="api-pim-historyqospolicy"></a>

<a id="historyqospolicy"></a>

# HistoryQosPolicy

### class HistoryQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies the behavior of the Service in the case where the value of a sample changes (one or more times) before it can be successfully communicated to one or more existing subscribers. This QoS policy controls whether the Service should deliver only the most recent value, attempt to deliver all intermediate values, or do something in between. On the publishing side this policy controls the samples that should be maintained by the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) on behalf of existing [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) entities. The behavior with regards to a DataReaderentities discovered after a sample is written is controlled by the DURABILITY QoS policy. On the subscribing side it controls the samples that should be maintained until the application “takes” them from the Service.

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline HistoryQosPolicy()

Constructor. 

### virtual ~HistoryQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### [HistoryQosPolicyKind](historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE) kind

HistoryQosPolicyKind.

By default, KEEP_LAST_HISTORY_QOS.

### std::int32_t depth

History depth. 



 By default, 1. If a value other than 1 is specified, it should be consistent with the settings of the [ResourceLimitsQosPolicy](resourcelimitsqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1ResourceLimitsQosPolicy).

#### WARNING
Only takes effect if the kind is KEEP_LAST_HISTORY_QOS.
