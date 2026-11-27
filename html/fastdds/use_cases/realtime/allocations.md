<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="realtime-allocations"></a>

<a id="tuning-allocations"></a>

# Tuning allocations

Allocating and deallocating memory implies some non-deterministic time consuming operations.
Therefore, most real-time systems need to operate in a way that all dynamic memory is allocated during the
application initialization, avoiding memory management operations in the main loop.

If users provide maximum sizes for the data and collections that *Fast DDS* keeps internally,
memory for these data and collections can be preallocated during entity initialization.
In order to choose the correct size values, users must be aware of the topology of the whole domain.
Specifically, the number of [DomainParticipants](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant),
[DataWriters](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), and [DataReaders](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader)
must be known when setting their configuration.

The following sections describe how to configure allocations to be done during the initialization of the
entities.
Although some examples are provided on each section as reference, there is also a
[complete example use case](#tuning-allocations-full-example).

<a id="parameters-on-the-participant"></a>

## Parameters on the participant

Every DomainParticipant holds an internal collection with information about every local and remote
peer DomainParticipants that has been discovered.
This information includes, among other things:

* A nested collection with information of every DataWriter announced on the
  peer DomainParticipant.
* A nested collection with information of every DataReader announced on the
  peer DomainParticipant.
* Custom data configured by the user on the peer DomainParticipant, namely,
  [UserDataQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#userdataqospolicy), [PartitionQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy),
  and [PropertyPolicyQos](../../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos).

By default, these collections are fully dynamic, meaning that new memory is allocated when a new
DomainParticipant, DataWriter, or DataReader
is discovered.
Likewise, the mentioned custom configuration data parameters have an arbitrary size.
By default, the memory for these parameters is allocated when the peer DomainParticipant announces
their value.

However, [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) has a member function [`allocation()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10allocationEv),
of type [ParticipantResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos), that allows configuring
maximum sizes for these collections and parameters, so that all the required memory can be preallocated during
the initialization of the DomainParticipant.

<a id="limiting-the-number-of-discovered-entities"></a>

### Limiting the number of discovered entities

[ParticipantResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos) provides three data members to configure the allocation behavior of
discovered entities:

* [`participants`](../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes12participantsE) configures the allocation of the collection of discovered
  DomainParticipants.
* [`readers`](../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes7readersE) configures the allocation of the collection of DataWriters
  within each discovered DomainParticipant.
* [`writers`](../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes7writersE) configures the allocation of the collection of DataReaders
  within each discovered DomainParticipant.

By default, a full dynamic behavior is used.
Using these members, however, it is easy to configure the collections to be preallocated during initialization,
setting them to a static maximum expected value, as shown in the example below.
Please, refer to [ResourceLimitedContainerConfig](../../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig) for a complete description of additional configuration
alternatives given by these data members.

### C++

```c++
DomainParticipantQos qos;

// Fix the size of discovered participants to 3
// This will effectively preallocate the memory during initialization
qos.allocation().participants =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);

// Fix the size of discovered DataWriters to 1 per DomainParticipant
// Fix the size of discovered DataReaders to 3 per DomainParticipant
// This will effectively preallocate the memory during initialization
qos.allocation().writers =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
qos.allocation().readers =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_profile_qos_entity_resource_limit">
        <rtps>
            <allocation>
                <!-- Limit to 3 participants -->
                <total_participants>
                    <initial>3</initial>
                    <maximum>3</maximum>
                    <increment>0</increment>
                </total_participants>

                <!-- Limit to 3 readers per participant -->
                <total_readers>
                    <initial>3</initial>
                    <maximum>3</maximum>
                    <increment>0</increment>
                </total_readers>

                <!-- Limit to 1 writer per participant -->
                <total_writers>
                    <initial>1</initial>
                    <maximum>1</maximum>
                    <increment>0</increment>
                </total_writers>
            </allocation>
        </rtps>
    </participant>
</profiles>
```

#### WARNING
Configuring a collection as fixed in size effectively limits the number of peer entities
that can be discovered.
Once the configured limit is reached, any new entity will be ignored.
In the given example, if a fourth peer DomainParticipant appears, it will
not be discovered, as the collection of discovered DomainParticipants
is already full.

<a id="limiting-the-size-of-custom-parameters"></a>

### Limiting the size of custom parameters

[`data_limits`](../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes11data_limitsE) inside [ParticipantResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos) provides three data members
to configure the allocation behavior of custom parameters:

* [`max_user_data`](../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits13max_user_dataE) limits the size of [UserDataQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#userdataqospolicy) to the given number
  of octets.
* [`max_properties`](../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits14max_propertiesE) limits the size of [PartitionQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy) to the given number
  of octets.
* [`max_partitions`](../../api_reference/rtps/attributes/VariableLengthDataLimits.md#_CPPv4N8eprosima7fastdds4rtps24VariableLengthDataLimits14max_partitionsE) limits the size of [PropertyPolicyQos](../../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) to the given number
  of octets.

If these sizes are configured to something different than zero, enough memory will be allocated for them
for each participant and endpoint.
A value of zero implies no size limitation, and memory will be dynamically allocated as needed.
By default, a full dynamic behavior is used.

[`content_filter`](../../api_reference/rtps/attributes/RTPSParticipantAllocationAttributes.md#_CPPv4N8eprosima7fastdds4rtps35RTPSParticipantAllocationAttributes14content_filterE) inside [ParticipantResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#participantresourcelimitsqos) provides members
to configure the allocation behavior of content filter discovery information:

* [`expression_initial_size`](../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfiguration23expression_initial_sizeE) sets the preallocated size of
  the filter expression.
* [`expression_parameters`](../../api_reference/rtps/builtin/data/ContentFilterProperty.md#_CPPv4N8eprosima7fastdds4rtps21ContentFilterProperty23AllocationConfiguration21expression_parametersE) controls the allocation behavior
  for the list of expression parameters.
  Refer to [ResourceLimitedContainerConfig](../../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig) for a complete description of the alternatives.
  Receiving information about a content filter with more parameters than the maximum configured here, will make
  the filtering happen on the reader side.

### C++

```c++
DomainParticipantQos qos;

// Fix the size of the complete user data field to 256 octets
qos.allocation().data_limits.max_user_data = 256u;
// Fix the size of the complete partitions field to 256 octets
qos.allocation().data_limits.max_partitions = 256u;
// Fix the size of the complete properties field to 512 octets
qos.allocation().data_limits.max_properties = 512u;
// Set the preallocated filter expression size to 512 characters
qos.allocation().content_filter.expression_initial_size = 512u;
// Set the maximum number of expression parameters to 4 and its allocation configuration to fixed size
qos.allocation().content_filter.expression_parameters =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(4u);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_profile_qos_parameter_resource_limit">
        <rtps>
            <allocation>
                <max_partitions>256</max_partitions>
                <max_user_data>256</max_user_data>
                <max_properties>512</max_properties>

                <!-- content_filter cannot be configured using XML (yet) -->
            </allocation>
        </rtps>
    </participant>
</profiles>
```

#### WARNING
If the data fields announced by the remote peer do not fit on the preallocated memory,
an error will be triggered during the processing of the announcement message.
This usually means that the discovery messages of a remote peer with too large data fields
will be discarded, i.e., peers with too large data fields will not be discovered.

<a id="parameters-on-the-datawriter"></a>

## Parameters on the DataWriter

Every DataWriter holds internal collections with information about every
DataReader to which it matches.
By default, these collections are fully dynamic, meaning that new memory is allocated when a new
DataReader is matched.
However, [DataWriterQos](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriterqos) has a data member [`writer_resource_limits()`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQos22writer_resource_limitsEv),
of type [WriterResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#writerresourcelimitsqos), that allows configuring
the memory allocation behavior on the DataWriter.

[WriterResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#writerresourcelimitsqos) provides data members [`matched_subscriber_allocation`](../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQos29matched_subscriber_allocationE)
and [`reader_filters_allocation`](../../api_reference/dds_pim/core/policy/writerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23WriterResourceLimitsQos25reader_filters_allocationE)
of type [ResourceLimitedContainerConfig](../../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig) that allow configuring
the maximum expected size of the collection of matched DataReader,
and the collection of writer side content filters,
so they can be preallocated during the initialization of the DataWriter,
as shown in the example below.
Please, refer to [ResourceLimitedContainerConfig](../../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig) for a complete description of additional configuration
alternatives given by these data members.

### C++

```c++
DataWriterQos qos;

// Fix the size of matched DataReaders to 3
// This will effectively preallocate the memory during initialization
qos.writer_resource_limits().matched_subscriber_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
// Fix the size of writer side content filters to 1
// This will effectively preallocate the memory during initialization
qos.writer_resource_limits().reader_filters_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <data_writer profile_name="writer_profile_qos_resource_limit">
        <!-- Limit to 3 matching readers -->
        <matchedSubscribersAllocation>
            <initial>3</initial>
            <maximum>3</maximum>
            <increment>0</increment>
        </matchedSubscribersAllocation>

        <!-- reader_filters_allocation cannot be configured using XML (yet) -->
    </data_writer>
</profiles>
```

#### WARNING
Configuring the collection of matched DataReaders as fixed in size
effectively limits the number of DataReaders to be matched.
Once the configured limit is reached, any new DataReader will be ignored.
In the given example, if a fourth (potentially matching) DataReader
appears, it will not be matched, as the collection is already full.

<a id="parameters-on-the-datareader"></a>

## Parameters on the DataReader

Every DataReader holds an internal collection with information about every
[ReaderResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#readerresourcelimitsqos) to which it matches.
By default, this collection is fully dynamic, meaning that new memory is allocated when a new
DataWriter is matched.
However, [DataReaderQos](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos) has a data member [`reader_resource_limits()`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQos22reader_resource_limitsEv),
of type [ReaderResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#readerresourcelimitsqos), that allows configuring
the memory allocation behavior on the DataReader.

[ReaderResourceLimitsQos](../../dds_layer/core/policy/eprosimaExtensions.md#readerresourcelimitsqos) provides a data member [`matched_publisher_allocation`](../../api_reference/dds_pim/subscriber/readerresourcelimitsqos.md#_CPPv4N8eprosima7fastdds3dds23ReaderResourceLimitsQos28matched_publisher_allocationE)
of type [ResourceLimitedContainerConfig](../../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig) that allows configuring
the maximum expected size of the collection of matched DataWriters,
so that it can be preallocated during the initialization of the DataReader,
as shown in the example below.
Please, refer to [ResourceLimitedContainerConfig](../../dds_layer/core/policy/eprosimaExtensions.md#resourcelimitedcontainerconfig) for a complete description of additional configuration
alternatives given by this data member.

### C++

```c++
DataReaderQos qos;

// Fix the size of matched DataWriters to 1
// This will effectively preallocate the memory during initialization
qos.reader_resource_limits().matched_publisher_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <data_reader profile_name="reader_profile_qos_resource_limit">
        <!-- Limit to 1 matching writer -->
        <matchedPublishersAllocation>
            <initial>1</initial>
            <maximum>1</maximum>
            <increment>0</increment>
        </matchedPublishersAllocation>
    </data_reader>
</profiles>
```

#### WARNING
Configuring the collection of matched DataWriters as fixed in size
effectively limits the number of DataWriters to be matched.
Once the configured limit is reached, any new DataWriter will be ignored.
In the given example, if a fourth (potentially matching) DataWriter
appears, it will not be matched, as the collection is already full.

<a id="tuning-allocations-full-example"></a>

<a id="full-example"></a>

## Full example

Given a system with the following topology:

#### **Allocation tuning example topology**

| Participant P1     | Participant P2     | Participant P3     |
|--------------------|--------------------|--------------------|
| Topic 1 publisher  | Topic 1 subscriber | Topic 2 subscriber |
| Topic 1 subscriber |                    | Topic 2 publisher  |
| Topic 1 subscriber |                    | Topic 2 subscriber |
* The total number of DomainParticipants is 3.
* The maximum number of DataWriters per DomainParticipant is 1
* The maximum number of DataReaders per DomainParticipant is 2.
* The DataWriter for topic 1
  matches with 3 DataReaders.
* The DataWriter for topic 2
  matches with 2 DataReaders.
* All the DataReaders
  match exactly with 1 DataWriter.

We will assume that content filtering is not being used, and will also limit the size of the parameters:

* Maximum [PartitionQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#partitionqospolicy) size: 256
* Maximum [UserDataQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#userdataqospolicy) size: 256
* Maximum [PropertyPolicyQos](../../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) size: 512

The following piece of code shows the set of parameters needed for the use case depicted in this example.

### C++

```c++
// DomainParticipant configuration
//////////////////////////////////
DomainParticipantQos participant_qos;

// We know we have 3 participants on the domain
participant_qos.allocation().participants =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
// We know we have at most 2 readers on each participant
participant_qos.allocation().readers =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
// We know we have at most 1 writer on each participant
participant_qos.allocation().writers =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);

// We know the maximum size of partition data
participant_qos.allocation().data_limits.max_partitions = 256u;
// We know the maximum size of user data
participant_qos.allocation().data_limits.max_user_data = 256u;
// We know the maximum size of properties data
participant_qos.allocation().data_limits.max_properties = 512u;

// Content filtering is not being used
participant_qos.allocation().content_filter.expression_initial_size = 0u;
participant_qos.allocation().content_filter.expression_parameters =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(0u);

// DataWriter configuration for Topic 1
///////////////////////////////////////
DataWriterQos writer1_qos;

// We know we will only have three matching subscribers, and no content filters
writer1_qos.writer_resource_limits().matched_subscriber_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(3u);
writer1_qos.writer_resource_limits().reader_filters_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(0u);

// DataWriter configuration for Topic 2
///////////////////////////////////////
DataWriterQos writer2_qos;

// We know we will only have two matching subscribers
writer2_qos.writer_resource_limits().matched_subscriber_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(2u);
writer2_qos.writer_resource_limits().reader_filters_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(0u);


// DataReader configuration for both Topics
///////////////////////////////////////////
DataReaderQos reader_qos;

// We know we will only have one matching publisher
reader_qos.reader_resource_limits().matched_publisher_allocation =
        eprosima::fastdds::ResourceLimitedContainerConfig::fixed_size_configuration(1u);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_alloc_qos_example">
        <rtps>
            <allocation>
                <!-- We know we have 3 participants on the domain -->
                <total_participants>
                    <initial>3</initial>
                    <maximum>3</maximum>
                    <increment>0</increment>
                </total_participants>
                <!-- We know we have at most 2 readers on each participant -->
                <total_readers>
                    <initial>2</initial>
                    <maximum>2</maximum>
                    <increment>0</increment>
                </total_readers>
                <!-- We know we have at most 1 writer on each participant -->
                <total_writers>
                    <initial>1</initial>
                    <maximum>1</maximum>
                    <increment>0</increment>
                </total_writers>
                <max_partitions>256</max_partitions>
                <max_user_data>256</max_user_data>
                <max_properties>512</max_properties>

                <!-- content_filter cannot be configured using XML (yet) -->
            </allocation>
        </rtps>
    </participant>

    <data_writer profile_name="allocation_qos_example_pub_for_topic_1">
        <!-- we know we will have three matching subscribers and no content filter -->
        <matchedSubscribersAllocation>
            <initial>3</initial>
            <maximum>3</maximum>
            <increment>0</increment>
        </matchedSubscribersAllocation>

        <!-- reader_filters_allocation cannot be configured using XML (yet) -->
    </data_writer>

    <data_writer profile_name="allocation_qos_example_pub_for_topic_2">
        <!-- we know we will have two matching subscribers and no content filter -->
        <matchedSubscribersAllocation>
            <initial>2</initial>
            <maximum>2</maximum>
            <increment>0</increment>
        </matchedSubscribersAllocation>

        <!-- reader_filters_allocation cannot be configured using XML (yet) -->
    </data_writer>

    <data_reader profile_name="allocation_qos_example_sub">
        <!-- we know we will only have one matching publisher -->
        <matchedPublishersAllocation>
            <initial>1</initial>
            <maximum>1</maximum>
            <increment>0</increment>
        </matchedPublishersAllocation>
    </data_reader>
</profiles>
```
