<a id="api-pim-durabilityserviceqospolicy"></a>

<a id="durabilityserviceqospolicy"></a>

# DurabilityServiceQosPolicy

### class DurabilityServiceQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies the configuration of the durability service. That is, the service that implements the [DurabilityQosPolicy](durabilityqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1DurabilityQosPolicy) kind of TRANSIENT and PERSISTENT.

#### NOTE
Immutable Qos Policy 

#### WARNING
This [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) can be defined and is transmitted to the rest of the network but is not implemented in this version.

### Public Functions

### inline DurabilityServiceQosPolicy()

Constructor. 

### virtual ~DurabilityServiceQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### fastdds::dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) service_cleanup_delay

Control when the service is able to remove all information regarding a data-instance.

By default,

[dds::c_TimeZero](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1aee681a69cd352a1a7aa6ec2791a7a484). 

### [HistoryQosPolicyKind](historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE) history_kind

Controls the [HistoryQosPolicy](historyqospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1HistoryQosPolicy) of the fictitious [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

that stores the data within the durability service.

By default, KEEP_LAST_HISTORY_QOS.

### std::int32_t history_depth

Number of most recent values that should be maintained on the History. It only have effect if the history_kind is KEEP_LAST_HISTORY_QOS.

By default, 1.

### std::int32_t max_samples

Control the ResourceLimitsQos of the implied [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that stores the data within the durability service. Specifies the maximum number of data-samples the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)) can manage across all the instances associated with it. Represents the maximum samples the middleware can store for any one [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

). It is inconsistent for this value to be less than max_samples_per_instance.

By default, LENGTH_UNLIMITED.

### std::int32_t max_instances

Control the ResourceLimitsQos of the implied [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) that stores the data within the durability service. Represents the maximum number of instances [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

) can manage.

By default, LENGTH_UNLIMITED.

### std::int32_t max_samples_per_instance

Control the ResourceLimitsQos of the implied [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

that stores the data within the durability service. Represents the maximum number of samples of any one instance a DataWriter(or DataReader) can manage. It is inconsistent for this value to be greater than max_samples.

By default, LENGTH_UNLIMITED.
