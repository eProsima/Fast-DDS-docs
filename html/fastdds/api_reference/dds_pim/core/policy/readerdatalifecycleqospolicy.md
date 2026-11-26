<a id="api-pim-readerdatalifecycleqospolicy"></a>

<a id="readerdatalifecycleqospolicy"></a>

# ReaderDataLifecycleQosPolicy

### class ReaderDataLifecycleQosPolicy

Specifies the behavior of the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader) with regards to the lifecycle of the data-instances it manages. 

#### NOTE
Mutable Qos Policy 

#### WARNING
This Qos Policy will be implemented in future releases. 

### Public Functions

### inline ReaderDataLifecycleQosPolicy()

Constructor. 

### inline virtual ~ReaderDataLifecycleQosPolicy()

Destructor. 

### Public Members

### dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) autopurge_no_writer_samples_delay

Indicates the duration the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

must retain information regarding instances that have the instance_state NOT_ALIVE_NO_WRITERS.

By default,

[dds::c_TimeInfinite](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1a481d27d32a5fbfe02ca0030ecc4ee6ef). 

### dds::[Duration_t](../Time_t/Duration_t.md#_CPPv4N8eprosima7fastdds3dds10Duration_tE) autopurge_disposed_samples_delay

Indicates the duration the [DataReader](../../subscriber/datareader.md#classeprosima_1_1fastdds_1_1dds_1_1DataReader)

must retain information regarding instances that have the instance_state NOT_ALIVE_DISPOSED.

By default,

[dds::c_TimeInfinite](../Time_t/const_values.md#namespaceeprosima_1_1fastdds_1_1dds_1a481d27d32a5fbfe02ca0030ecc4ee6ef).
