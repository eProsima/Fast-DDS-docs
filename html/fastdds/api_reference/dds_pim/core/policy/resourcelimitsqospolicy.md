<a id="api-pim-resourcelimitsqospolicy"></a>

<a id="resourcelimitsqospolicy"></a>

# ResourceLimitsQosPolicy

### class ResourceLimitsQosPolicy : public eprosima::fastdds::dds::Parameter_t, public eprosima::fastdds::dds::[QosPolicy](qospolicy.md#_CPPv4N8eprosima7fastdds3dds9QosPolicyE)

Specifies the resources that the Service can consume in order to meet the requested QoS

#### NOTE
Immutable Qos Policy 

### Public Functions

### inline ResourceLimitsQosPolicy()

Constructor. 

### virtual ~ResourceLimitsQosPolicy() = default

Destructor. 

### inline virtual void clear() override

Clears the [QosPolicy](qospolicy.md#classeprosima_1_1fastdds_1_1dds_1_1QosPolicy) object. 

### Public Members

### std::int32_t max_samples

Specifies the maximum number of data-samples the [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)) can manage across all the instances associated with it. Represents the maximum samples the middleware can store for any one [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

).

Value less or equal to 0 means infinite resources. By default, LENGTH_UNLIMITED.

#### WARNING
It is inconsistent if `max_samples < (max_instances * max_samples_per_instance)`. 

### std::int32_t max_instances

Represents the maximum number of instances [DataWriter](../../publisher/datawriter.md#classeprosima_1_1fastdds_1_1dds_1_1DataWriter) (or [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

) can manage.

Value less or equal to 0 means infinite resources. By default, LENGTH_UNLIMITED.

#### WARNING
It is inconsistent if `(max_instances * max_samples_per_instance) > max_samples`. 

### std::int32_t max_samples_per_instance

Represents the maximum number of samples of any one instance a DataWriter(or DataReader) can manage.

Value less or equal to 0 means infinite resources. By default, LENGTH_UNLIMITED.

#### WARNING
It is inconsistent if `(max_instances * max_samples_per_instance) > max_samples`. 

### std::int32_t allocated_samples

Number of samples initially allocated.

By default, 100.

### std::int32_t extra_samples

Represents the extra number of samples available once the max_samples have been reached in the history. This makes it possible, for example, to loan samples even with a full history. By default, 1.
