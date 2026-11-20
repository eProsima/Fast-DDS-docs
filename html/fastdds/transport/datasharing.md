<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="datasharing-delivery"></a>

<a id="data-sharing-delivery"></a>

# Data-sharing delivery

*Fast DDS* allows to speed up communications between entities within the same machine
by sharing the history of the [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) with the [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) through shared memory.
This prevents any of the overhead involved in the transport layer,
effectively avoiding any data copy between DataWriter and DataReader.

#### NOTE
Fast DDS utilizes the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)’s [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) to identify peers running in the same host.
Two participants with identical 4 first bytes on the [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) are considered to be running in the same
host.
[`is_on_same_host_as()`](../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4NK8eprosima7fastdds4rtps6GUID_t18is_on_same_host_asERK6GUID_t) API is provided to check this condition.
Please, take also into account the caveats included in [GUID Prefix considerations for intra-process delivery](intraprocess.md#intraprocess-delivery-guids).

Use of Data-sharing delivery does not prevent data copies between the application
and the DataReader and DataWriter.
These can be avoided in some cases using [Zero-Copy communication](../use_cases/zero_copy/zero_copy.md#use-case-zero-copy).

#### NOTE
Although Data-sharing delivery uses shared memory,
it differs from [Shared Memory Transport](shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory)
in that Shared Memory is a full-compliant transport.
That means that with Shared Memory Transport
the data being transmitted must be copied from the DataWriter history to the transport
and from the transport to the DataReader.
With Data-sharing these copies can be avoided.

<a id="overview"></a>

## Overview

When the DataWriter is created, *Fast DDS* will pre-allocate a pool of
[`max_samples`](../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy11max_samplesE) + [`extra_samples`](../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE) samples that reside
in a shared memory mapped file.
When publishing new data, the DataWriter will take a sample from this pool and add it to its history,
and notify the DataReader which sample from the pool has the new data.

The DataReader will have access to the same shared memory mapped file,
and will be able to access the data published by the DataWriter.

<a id="datasharing-delivery-constraints"></a>

<a id="constraints"></a>

## Constraints

This feature is available only if the following requirements are met:

* The [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) have access to the same shared memory.
* The [Topic](../dds_layer/topic/topic.md#dds-layer-topic) has a bounded [TopicDataType](../dds_layer/topic/typeSupport/typeSupport.md#dds-layer-definition-data-types),
  i.e., its [`is_bounded()`](../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4NK8eprosima7fastdds3dds13TopicDataType10is_boundedEv) member function returns true.
* The Topic [is not keyed](../dds_layer/topic/typeSupport/typeSupport.md#dds-layer-topic-keyed-data-types).
* The DataWriter is configured with [`PREALLOCATED_MEMORY_MODE`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy24PREALLOCATED_MEMORY_MODEE) or [`PREALLOCATED_WITH_REALLOC_MEMORY_MODE`](../api_reference/rtps/resources/MemoryManagementPolicy.md#_CPPv4N8eprosima7fastdds4rtps22MemoryManagementPolicy37PREALLOCATED_WITH_REALLOC_MEMORY_MODEE).
* No [security](../security/security.md#security) plugins are used.

There is also a limitation with the DataReader’s HistoryQos.
Using Data-sharing mechanism, the DataWriter’s history is shared with the DataReaders.
This means that the effective HistoryQos depth on the DataReader is, at most,
the Datawriter’s HistoryQos depth.
To avoid confusions, set the DataReaders’ history depth to a value equal or less than the DataWriter’s.

<a id="data-sharing-delivery-configuration"></a>

## Data-sharing delivery configuration

Data-sharing delivery can be configured in the [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and the [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader)
using [DataSharingQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#datasharingqospolicy).
Four attributes can be configured:

* The data-sharing delivery kind
* The shared memory directory
* The data-sharing domain identifiers.
* The maximum number of data-sharing domain identifiers.

<a id="data-sharing-delivery-kind"></a>

### Data-Sharing delivery kind

Can be set to one of three modes:

* **AUTO**: If both a DataWriter and DataReader meet the requirements,
  data-sharing delivery will be used between them.
  This is the default value.
* **ON**: Like **AUTO**, but the creation of the entity will fail if the requirements are not met.
* **OFF**: No data-sharing delivery will be used on this entity.

The following matrix shows when two entities are data-sharing compatible according to their configuration
(given that the entity creation does not fail and that both entities have access to a shared memory):

|            |                                     | **Reader**                          |
|------------|-------------------------------------|-------------------------------------|
| **ON**     | **OFF**                             | **AUTO**                            |
| **Writer** | **ON**                              | Only if they have common domain IDs |
| **OFF**    | No                                  | No                                  |
| **AUTO**   | Only if they have common domain IDs | No                                  |

<a id="data-sharing-domain-identifiers"></a>

### Data-sharing domain identifiers

Each entity defines a set of identifiers that represent a *domain* to which the entity belongs.
Two entities will be able to use data-sharing delivery between them only if both have at least a common domain.

Users can define the domains of a [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) with the [DataSharingQosPolicy](../dds_layer/core/policy/eprosimaExtensions.md#datasharingqospolicy).
If no domain identifier is provided by the user, the system will create one automatically.
This automatic data-sharing domain will be unique for the machine where the entity is running.
That is, all entities running on the same machine, and for which the user has configured no user-specific domains,
will be able to use data-sharing delivery (given that the rest of requirements are met).

During the discovery phase, entities will exchange their domain identifiers and check if they can
use Data-sharing to communicate.

#### NOTE
Even though a data-sharing domain identifier is a 64 bit integer,
user-defined identifiers are restricted to 16 bit integers.

<a id="maximum-number-of-data-sharing-domain-identifiers"></a>

### Maximum number of Data-sharing domain identifiers

The maximum number of domain identifiers that are expected to be received
from a remote entity during discovery.
If the remote entity defines (and sends) more than this number of domain identifiers,
the discovery will fail.

By default there is no limit to the number of identifiers.
The default value can be changed with the [`max_domains()`](../api_reference/dds_pim/core/policy/datasharingqospolicy.md#_CPPv4NK8eprosima7fastdds3dds20DataSharingQosPolicy11max_domainsEv) function.
Defining a finite number allows to preallocate the required memory
to receive the list of identifiers during the entity creation,
avoiding dynamic memory allocations afterwards.
Note that a value of `0` means no limit.

<a id="shared-memory-directory"></a>

### Shared memory directory

If a user-defined directory is given for the shared memory files,
this directory will be used for the memory-mapped files used for data-sharing delivery.
If none is given, the default directory configured for the current system is used.

Configuring a user-defined directory may be useful in some scenarios:

* To select a file system with Huge TLB enabled for the memory-mapped files.
* To allow data-sharing delivery between containers that mount the same container.

<a id="datareader-datawriter-history-coupling"></a>

<a id="datareader-and-datawriter-history-coupling"></a>

## DataReader and DataWriter history coupling

With traditional [Transport Layer](transport.md#comm-transports-configuration) delivery,
the DataReader and DataWriter keep separate and independent histories,
each one with their own copy of the sample.
Once the sample is sent through the transport and received by the DataReader,
the DataWriter is free to remove the sample from its history
without affecting the DataReader.

With data-sharing delivery,
the DataReader directly accesses the data instance created by the DataWriter.
This means that the samples in both the history of the DataReader and the DataWriter
refer to the same object in the shared memory.
Therefore, there is a strong coupling in the behavior of the DataReader and DataWriter histories.

#### IMPORTANT
If the DataWriter reuses the same sample to publish new data,
the DataReader loses access to the old data sample.

#### NOTE
The DataWriter can remove the sample from its history,
and it will still be available on the DataReader,
unless the same sample from the pool is reused to publish a new one.

<a id="data-acknowledgement"></a>

### Data acknowledgement

With data-sharing delivery, sample acknowledgment from the DataReader occurs the first time
a sample is retrieved by the application (using [`DataReader::read_next_sample()`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16read_next_sampleEPvP10SampleInfo),
[`DataReader::take_next_sample()`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader16take_next_sampleEPvP10SampleInfo), or any of their variations).
Once the data has been accessed by the application,
the DataWriter is free to reuse that sample to publish new data.
The DataReader detects when a sample has been reused
and automatically removes it from its history.

This means that subsequent attempts to access the same sample
from the DataReader may return no sample at all.

<a id="blocking-reuse-of-samples-until-acknowledged"></a>

### Blocking reuse of samples until acknowledged

With [`KEEP_LAST_HISTORY_QOS`](../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) or [`BEST_EFFORT_RELIABILITY_QOS`](../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) configurations,
the DataWriter can remove samples from its history to add new ones,
even if they were not acknowledged by the DataReader.
In situations where the publishing rate is consistently faster
than the rate at which the DataReader can process the samples,
this can lead to every sample being reused before the application
has a chance to process it, thus blocking the communication at application level.

In order to avoid this situation,
the samples in the preallocated pool are never reused unless they have been acknowledged,
i.e., they have been processed by the application at least once.
If there is no reusable sample in the pool,
the writing operation in the DataWriter will be blocked until one is available
or until [`max_blocking_time`](../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) is reached.

Note that the DataWriter history is not affected by this behavior,
samples will be removed from the history by standard rules.
Only the reuse of pool samples is affected.
This means that the DataWriter history can be empty and the write operation
be still blocked because all samples in the pool are unacknowledged.

The chance of the DataWriter blocking on a write operation can be reduced
using [`extra_samples`](../api_reference/dds_pim/core/policy/resourcelimitsqospolicy.md#_CPPv4N8eprosima7fastdds3dds23ResourceLimitsQosPolicy13extra_samplesE).
This will make the pool to allocate more samples than the history size,
so that the DataWriter has more chances to get a free sample,
while the DataReader can still access samples that have been removed from the
DataWriter history.
