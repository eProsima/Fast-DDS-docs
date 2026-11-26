<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-reduce-memory"></a>

<a id="reduce-memory-usage"></a>

# Reduce memory usage

A great number of modern systems have tight constraints on available memory, making the reduction of memory usage to a
minimum critical. Reducing memory consumption of a *Fast DDS* application can be achieved through various approaches,
mainly through architectural restructuring of the application, but also by limiting the resources the
middleware utilizes, and by avoiding static allocations.

<a id="limiting-resources"></a>

## Limiting Resources

The [ResourceLimitsQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#resourcelimitsqospolicy) controls the resources that the service can use in order to meet the requirements
imposed. It limits the amount of allocated memory per [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or
[DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader), as per the following parameters:

* [`max_samples`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE): Configures the maximum number of samples that the DataWriter or DataReader can manage across all the
  instances associated with it, i.e. it represents the maximum samples that the middleware can store for a DataReader or
  DataWriter.
* [`max_instances`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13max_instancesE): Configures the maximum number of instances that the DataWriter or DataReader can manage.
* [`max_samples_per_instance`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE): Controls the maximum number of samples within an instance  that the DataWriter or
  DataReader can manage.
* [`allocated_samples`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy17allocated_samplesE): States the number of samples that will be allocated on initialization.

All these parameters may be lowered as much as needed to reduce memory consumption, limit the resources to the
application’s needs. Below is an example of a configuration for the minimum resource limits possible.

#### WARNING
* The value of [`max_samples`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) must be higher or equal to the value of [`max_samples_per_instance`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE).
* The value established for the [HistoryQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#historyqospolicy) [`depth`](../../api_reference/dds_pim/core/policy/historyqospolicy.md#_CPPv4N8eprosima7fastdds3dds16HistoryQosPolicy5depthE) must be lower or equal to the value stated for
  [`max_samples_per_instance`](../../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy24max_samples_per_instanceE).

### C++

```c++
ResourceLimitsQosPolicy resource_limits;

// The ResourceLimitsQosPolicy is constructed with max_samples = LENGTH_UNLIMITED by default
// Change max_samples to the minimum
resource_limits.max_samples = 1;

// The ResourceLimitsQosPolicy is constructed with max_instances = LENGTH_UNLIMITED by default
// Change max_instances to the minimum
resource_limits.max_instances = 1;

// The ResourceLimitsQosPolicy is constructed with max_samples_per_instance = LENGTH_UNLIMITED by default
// Change max_samples_per_instance to the minimum
resource_limits.max_samples_per_instance = 1;

// The ResourceLimitsQosPolicy is constructed with allocated_samples = 100 by default
// No allocated samples
resource_limits.allocated_samples = 0;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <data_writer profile_name="data_writer_min_samples">
        <topic>
            <historyQos>
                <kind>KEEP_LAST</kind>
                <depth>1</depth>
            </historyQos>
            <resourceLimitsQos>
                <max_samples>1</max_samples>
                <max_instances>1</max_instances>
                <max_samples_per_instance>1</max_samples_per_instance>
                <allocated_samples>0</allocated_samples>
            </resourceLimitsQos>
        </topic>
    </data_writer>

    <data_reader profile_name="data_reader_min_samples">
        <topic>
            <historyQos>
                <kind>KEEP_LAST</kind>
                <depth>1</depth>
            </historyQos>
            <resourceLimitsQos>
                <max_samples>1</max_samples>
                <max_instances>1</max_instances>
                <max_samples_per_instance>1</max_samples_per_instance>
                <allocated_samples>0</allocated_samples>
            </resourceLimitsQos>
        </topic>
    </data_reader>
</profiles>
```

<a id="set-dynamic-allocation"></a>

## Set Dynamic Allocation

By default [MemoryManagementPolicy](../../dds_layer/core/policy/eprosimaExtensions.md#memorymanagementpolicy) is set to [`PREALLOCATED_WITH_REALLOC_MEMORY_MODE`](../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy37PREALLOCATED_WITH_REALLOC_MEMORY_MODEE), meaning that the
amount of memory required by the configured [ResourceLimitsQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#resourcelimitsqospolicy) will be allocated at initialization.
If some more memory has to be allocated at run time, it is reallocated.

Using the dynamic settings of the [RTPSEndpointQos](../../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos) will prevent unnecessary allocations. Lowest footprint is
achieved with [`DYNAMIC_RESERVE_MEMORY_MODE`](../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy27DYNAMIC_RESERVE_MEMORY_MODEE) at the cost of higher allocation counts, in this mode memory is
allocated when needed and freed as soon as it stops being used. For higher determinism at a small memory cost the
[`DYNAMIC_REUSABLE_MEMORY_MODE`](../../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy28DYNAMIC_REUSABLE_MEMORY_MODEE) option is available, this option is similar but once more memory is allocated it is
not freed and is reused for future messages.

### C++

```c++
RTPSEndpointQos endpoint;
endpoint.history_memory_policy = eprosima::fastdds::rtps::DYNAMIC_REUSABLE_MEMORY_MODE;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <data_writer profile_name="data_writer_low_memory">
        <!-- ...  -->
        <historyMemoryPolicy>DYNAMIC_REUSABLE</historyMemoryPolicy>
    </data_writer>

    <data_reader profile_name="data_reader_low_memory">
        <!-- ...  -->
        <historyMemoryPolicy>DYNAMIC_REUSABLE</historyMemoryPolicy>
    </data_reader>
</profiles>
```
