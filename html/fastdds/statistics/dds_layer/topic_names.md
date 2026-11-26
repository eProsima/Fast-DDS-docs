<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="statistics-topic-names"></a>

<a id="id1"></a>

# Statistics Topic names

Data collected by the *Fast DDS Statistics module* is published in one of the topics listed below.
In order to simplify its use, the API provides aliases for the different statistics topics (see
[Statistics topic names](../../api_reference/statistics/topic_names.md#api-statistics-topic-names)).
The following table shows the correlation between the topic name and the corresponding alias.

| **Topic name**                                | **Alias**                                                                                                                                             |
|-----------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------|
| `_fastdds_statistics_history2history_latency` | [`HISTORY_LATENCY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21HISTORY_LATENCY_TOPICE)                 |
| `_fastdds_statistics_network_latency`         | [`NETWORK_LATENCY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21NETWORK_LATENCY_TOPICE)                 |
| `_fastdds_statistics_publication_throughput`  | [`PUBLICATION_THROUGHPUT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics28PUBLICATION_THROUGHPUT_TOPICE)   |
| `_fastdds_statistics_subscription_throughput` | [`SUBSCRIPTION_THROUGHPUT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics29SUBSCRIPTION_THROUGHPUT_TOPICE) |
| `_fastdds_statistics_rtps_sent`               | [`RTPS_SENT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15RTPS_SENT_TOPICE)                             |
| `_fastdds_statistics_rtps_lost`               | [`RTPS_LOST_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15RTPS_LOST_TOPICE)                             |
| `_fastdds_statistics_heartbeat_count`         | [`HEARTBEAT_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21HEARTBEAT_COUNT_TOPICE)                 |
| `_fastdds_statistics_acknack_count`           | [`ACKNACK_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics19ACKNACK_COUNT_TOPICE)                     |
| `_fastdds_statistics_nackfrag_count`          | [`NACKFRAG_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics20NACKFRAG_COUNT_TOPICE)                   |
| `_fastdds_statistics_gap_count`               | [`GAP_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15GAP_COUNT_TOPICE)                             |
| `_fastdds_statistics_data_count`              | [`DATA_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics16DATA_COUNT_TOPICE)                           |
| `_fastdds_statistics_resent_datas`            | [`RESENT_DATAS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics18RESENT_DATAS_TOPICE)                       |
| `_fastdds_statistics_sample_datas`            | [`SAMPLE_DATAS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics18SAMPLE_DATAS_TOPICE)                       |
| `_fastdds_statistics_pdp_packets`             | [`PDP_PACKETS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics17PDP_PACKETS_TOPICE)                         |
| `_fastdds_statistics_edp_packets`             | [`EDP_PACKETS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics17EDP_PACKETS_TOPICE)                         |
| `_fastdds_statistics_discovered_entity`       | [`DISCOVERY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15DISCOVERY_TOPICE)                             |
| `_fastdds_statistics_physical_data`           | `PHYSICAL_DATA_TOPIC`                                                                                                                                 |

<a id="history-latency-topic"></a>

## [`HISTORY_LATENCY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21HISTORY_LATENCY_TOPICE)

The `_fastdds_statistics_history2history_latency` statistics topic collects data related with the latency between any
two matched endpoints.
This measurement provides information about the DDS overall latency independent of the user’s application overhead.
Specifically, the measured latency corresponds to the time spent between the instant when the sample is written to the
DataWriter’s history and the time when the sample is added to the DataReader’s history and the notification is issued to
the corresponding user’s callback.

<a id="network-latency-topic"></a>

## [`NETWORK_LATENCY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21NETWORK_LATENCY_TOPICE)

#### NOTE
**[Pro]** This topic is not available in *Fast DDS* **[Pro]**, see [Statistics topics not available in Fast DDS](#statistics-topic-names-removed) .

The `_fastdds_statistics_network_latency` statistics topic collects data related with the network latency (expressed
in *ns*) between any two communicating locators.
This measurement provides information about the transport layer latency.
The measured latency corresponds to the time spent between the message being written until the message being received
in the `MessageReceiver`.

#### IMPORTANT
In the case of [TCP Transport](../../transport/tcp/tcp.md#transport-tcp-tcp), the reported latency also includes the time spent on the datagram’s CRC
related operations.
Mind that is possible to disable CRC operations when defining the [TCPTransportDescriptor](../../transport/tcp/tcp.md#transport-tcp-transportdescriptor).

<a id="publication-throughput-topic"></a>

## [`PUBLICATION_THROUGHPUT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics28PUBLICATION_THROUGHPUT_TOPICE)

The `_fastdds_statistics_publication_throughput` statistics topic collects the amount of data (expressed in *B/s*)
that is being sent by each DataWriter.
This measurement provides information about the publication’s throughput.

<a id="subscription-throughput-topic"></a>

## [`SUBSCRIPTION_THROUGHPUT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics29SUBSCRIPTION_THROUGHPUT_TOPICE)

The `_fastdds_statistics_subscription_throughput` statistics topic collects the amount of data (expressed in *B/s*)
that is being received by each DataReader.
This measurement provides information about the subscription’s throughput.

<a id="rtps-sent-topic"></a>

## [`RTPS_SENT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15RTPS_SENT_TOPICE)

#### NOTE
**[Pro]** This topic is not available in *Fast DDS* **[Pro]**, see [Statistics topics not available in Fast DDS](#statistics-topic-names-removed) .

The `_fastdds_statistics_rtps_sent` statistics topic collects the number of RTPS packets and bytes that are being sent
from each DDS entity to each locator.

<a id="rtps-lost-topic"></a>

## [`RTPS_LOST_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15RTPS_LOST_TOPICE)

#### NOTE
**[Pro]** This topic is not available in *Fast DDS* **[Pro]**, see [Statistics topics not available in Fast DDS](#statistics-topic-names-removed) .

The `_fastdds_statistics_rtps_lost` statistics topic collects the number of RTPS packets and bytes that are being lost
in the transport layer (dropped somewhere in between) in the communication between each DDS entity and locator.

<a id="heartbeat-count-topic"></a>

## [`HEARTBEAT_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21HEARTBEAT_COUNT_TOPICE)

The `_fastdds_statistics_heartbeat_count` statistics topic collects the number of heartbeat messages sent by each
user’s DataWriter.
This topic does not apply to builtin (related to [Discovery](../../discovery/discovery.md#discovery)) and statistics DataWriters.
Heartbeat messages are only sent if the [ReliabilityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy) is set to [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).
These messages report the DataWriter’s status.

<a id="acknack-count-topic"></a>

## [`ACKNACK_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics19ACKNACK_COUNT_TOPICE)

The `_fastdds_statistics_acknack_count` statistics topic collects the number of acknack messages sent by each user’s
DataReader.
This topic does not apply to builtin DataReaders (related to [Discovery](../../discovery/discovery.md#discovery)).
Acknack messages are only sent if the [ReliabilityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy) is set to [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).
These messages report the DataReader’s status.

<a id="nackfrag-count-topic"></a>

## [`NACKFRAG_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics20NACKFRAG_COUNT_TOPICE)

The `_fastdds_statistics_nackfrag_count` statistics topic collects the number of nackfrag messages sent by each user’s
DataReader.
This topic does not apply to builtin DataReaders (related to [Discovery](../../discovery/discovery.md#discovery)).
Nackfrag messages are only sent if the [ReliabilityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy) is set to [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).
These messages report the data fragments that have not been received yet by the DataReader.

<a id="gap-count-topic"></a>

## [`GAP_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15GAP_COUNT_TOPICE)

The `_fastdds_statistics_gap_count` statistics topic collects the number of gap messages sent by each user’s
DataWriter.
This topic does not apply to builtin (related to [Discovery](../../discovery/discovery.md#discovery)) and statistics DataWriters.
Gap messages are only sent if the [ReliabilityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy) is set to [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).
These messages report that some specific samples are not relevant to a specific DataReader.

<a id="data-count-topic"></a>

## [`DATA_COUNT_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics16DATA_COUNT_TOPICE)

The `_fastdds_statistics_data_count` statistics topic collects the total number of user’s data messages and data
fragments (in case that the message size is large enough to require RTPS fragmentation) that have been sent by each
user’s DataWriter.
This topic does not apply to builtin (related to [Discovery](../../discovery/discovery.md#discovery)) and statistics DataWriters.

<a id="resent-datas-topic"></a>

## [`RESENT_DATAS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics18RESENT_DATAS_TOPICE)

The `_fastdds_statistics_resent_data` statistics topic collects the total number of user’s data messages and data
fragments (in case that the message size is large enough to require RTPS fragmentation) that have been necessary to
resend by each user’s DataWriter.
This topic does not apply to builtin (related to [Discovery](../../discovery/discovery.md#discovery)) and statistics DataWriters.

<a id="sample-datas-topic"></a>

## [`SAMPLE_DATAS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics18SAMPLE_DATAS_TOPICE)

#### NOTE
**[Pro]** This topic is not available in *Fast DDS* **[Pro]**, see [Statistics topics not available in Fast DDS](#statistics-topic-names-removed) .

The `_fastdds_statistics_sample_datas` statistics topic collects the number of user’s data messages (or data fragments
in case that the message size is large enough to require RTPS fragmentation) that have been sent by the user’s
DataWriter to completely deliver a single sample.
This topic does not apply to builtin (related to [Discovery](../../discovery/discovery.md#discovery)) and statistics DataWriters.

<a id="pdp-packets-topic"></a>

## [`PDP_PACKETS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics17PDP_PACKETS_TOPICE)

The `_fastdds_statistics_pdp_packets` statistics topic collects the number of PDP discovery traffic RTPS packets
transmitted by each DDS [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
PDP packets are the data messages exchanged during the PDP discovery phase (see [Discovery phases](../../discovery/discovery.md#disc-phases) for more
information).

<a id="edp-packets-topic"></a>

## [`EDP_PACKETS_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics17EDP_PACKETS_TOPICE)

The `_fastdds_statistics_edp_packets` statistics topic collects the number of EDP discovery traffic RTPS packets
transmitted by each DDS [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
EDP packets are the data messages exchanged during the EDP discovery phase (see [Discovery phases](../../discovery/discovery.md#disc-phases) for more
information).

<a id="statistics-topic-names-discovery"></a>

<a id="discovery-topic"></a>

## [`DISCOVERY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics15DISCOVERY_TOPICE)

#### NOTE
**[Pro]** This topic is not available in *Fast DDS* **[Pro]**, see [Statistics topics not available in Fast DDS](#statistics-topic-names-removed) .

The `_fastdds_statistics_discovered_entity` statistics topic reports the time when each local [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE)
discovers any remote DDS entity (with the exception of those DDS entities related with the *Fast DDS Statistics
module*).
This topic also carries the `PHYSICAL_DATA_TOPIC` information for the case of discovered [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE); if the
discovered entity is either a [`DataReader`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) or [`DataWriter`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE), then the physical information is empty (see
[Physical Data in Discovery Information](../../property_policies/non_consolidated_qos.md#property-policies-physical-data) for more information about how to configure the physical data conveyed on the
discovery messages).

<a id="statistics-topic-names-physical"></a>

<a id="physical-data-topic"></a>

## `PHYSICAL_DATA_TOPIC`

#### NOTE
**[Pro]** This topic is not available in *Fast DDS* **[Pro]**, see [Statistics topics not available in Fast DDS](#statistics-topic-names-removed) .

The `_fastdds_statistics_physical_data` statistics topic reports the host, user and process where the
*Fast DDS Statistics module* is running.

<a id="statistics-topic-names-removed"></a>

<a id="statistics-topics-not-available-in-fast-dds-pro"></a>

## Statistics topics not available in Fast DDS **[Pro]**

**[Pro]** The following statistics topics are published by *Fast DDS*, but they have been removed from *Fast DDS* **[Pro]**.
In *Fast DDS* **[Pro]**, enabling any of them has no effect, and a warning is logged.
Their names are still declared, and deprecated, so that applications referring to them keep building.

| **Topic name**                          | **Alias**               |
|-----------------------------------------|-------------------------|
| `_fastdds_statistics_network_latency`   | `NETWORK_LATENCY_TOPIC` |
| `_fastdds_statistics_rtps_sent`         | `RTPS_SENT_TOPIC`       |
| `_fastdds_statistics_rtps_lost`         | `RTPS_LOST_TOPIC`       |
| `_fastdds_statistics_discovered_entity` | `DISCOVERY_TOPIC`       |
| `_fastdds_statistics_sample_datas`      | `SAMPLE_DATAS_TOPIC`    |
| `_fastdds_statistics_physical_data`     | `PHYSICAL_DATA_TOPIC`   |

The Network Latency and RTPS Lost topics were derived from a dedicated submessage that *Fast DDS* appended to
every outgoing RTPS message.
*Fast DDS* **[Pro]** no longer sends that submessage, which reduces the size of every RTPS message.
The remaining topics were removed because no consumer used the data they reported.

In *Fast DDS* **[Pro]**, [`HISTORY_LATENCY_TOPIC`](../../api_reference/statistics/topic_names.md#_CPPv4N8eprosima7fastdds10statistics21HISTORY_LATENCY_TOPICE) is the closest available latency measurement.
Mind that it is reported for each pair of DataWriter and DataReader, and that it covers the complete path
between both histories, which includes the network but is not limited to it.

The host, user and process where an application runs are still announced in the participant discovery message,
so in *Fast DDS* **[Pro]** they remain available without this topic (see [Physical Data in Discovery Information](../../property_policies/non_consolidated_qos.md#property-policies-physical-data)).
