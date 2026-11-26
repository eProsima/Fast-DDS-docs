<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="monitor-service-topics"></a>

<a id="id1"></a>

# Monitor Service Topics

The following table depicts the properties of the topics within the [Monitor Service](monitor_service.md#monitor-service):

| **Topic name**                   | **Topic Alias**       | **TopicDataType**                                           |
|----------------------------------|-----------------------|-------------------------------------------------------------|
| `fastdds_monitor_service_status` | MONITOR_SERVICE_TOPIC | [Monitor Service Status Data](#monitor-service-status-data) |

<a id="monitor-service-status-topic"></a>

<a id="id2"></a>

## Monitor Service Status Topic

The `Monitor Service Status Topic` carries information about the
[monitoring information](intro.md#monitor-service-keywords) of the local entities of a
particular [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
The [monitoring information](intro.md#monitor-service-keywords) can be divided into different statuses
identified by a `StatusKind`.
The possible values are described in the following table:

<a id="monitoring-statuses"></a>

|   **Value** | **Name**                  | **Description**                                                                                 |
|-------------|---------------------------|-------------------------------------------------------------------------------------------------|
|           0 | `ProxyInfo`               | Collection of Parameters describing the<br/>`Proxy Data` of that entity. Contains optional qos. |
|           1 | `ConnectionList`          | List of connections that this entity is<br/>using with its matched remote entities.             |
|           2 | `IncompatibleQoSInfo`     | Status of the Incompatible QoS<br/>of that entity.                                              |
|           3 | `InconsistentTopicInfo`   | Status of Inconsistent topics that the<br/>topic of that entity has.                            |
|           4 | `LivelinessLostInfo`      | Tracks the status of the number of times<br/>a writer lost liveliness.                          |
|           5 | `LivelinessChangedInfo`   | Tracks the status of the number of times<br/>the liveliness changed in a reader.                |
|           6 | `DeadlineMissedInfo`      | The Status of the number of deadlines<br/>missed of a sample for that entity.                   |
|           7 | `SampleLostInfo`          | Tracks the status of the number of times<br/>this entity lost samples.                          |
|           8 | `ExtendedIncompatibleQoS` | Stores the list of remote incompatible `GUIDs`<br/>and QoS policies for each local entity guid. |

#### NOTE
The `ProxyInfo` within the `Monitor Service Status Topic`
always includes the [optional qos](../../property_policies/non_consolidated_qos.md#property-serialize-optional-qos) independently of the
`fastdds.serialize_optional_qos` property.

#### NOTE
If the service is enabled in a [RTPS layer](../../rtps_layer/rtps_layer.md#rtps-layer) context, [not all
the statuses will be published](intro.md#monitor-service-in-rtps-note),
only the `ProxyInfo` and `ConnectionList`.

The `Monitor Service Status Topic` publishes new data when new updates are received from any
of the [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)’s local entities (on-event driven) with a minimum waiting time between
publications.
In addition, it is in charge of notifying about any disposal or liveliness lost.

<a id="monitor-service-status-data"></a>

<a id="id3"></a>

### Monitor Service Status Data

The `MonitorServiceStatusData` data structure comprises the following fields:

* *local_entity:* [`Guid_t`](../../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) of the local entity.
* *status_kind:* [StatusKind](#monitoring-statuses) enumeration identifying the status.
* *value:* The value of the status.

```bash
MonitorServiceStatusData
  @Key GUID local_entity
  @Key StatusKind status_kind
  Data value
```

#### NOTE
The *local_entity* and *status_kind* are keyed fields, hence making use of instances (see [Topics, keys and instances](../../dds_layer/topic/instances.md#dds-layer-topic-instances)).
In this case, the pair <*local_entity, status_kind*> identifies a unique instance.

Each of the `StatusKind` enumeration values maps to a corresponding `Data` value.
The actual field names for the different values are described below:

* *entity_proxy:* Collection of the serialized Quality of Service Parameters in the form of a `ParameterList`.
* *connection_list:* Defines how is this entity communicating with its matched entities.
  Each of the elements is of Connection type (depicted below).

```bash
Connection
  uint32_t mode //INTRAPROCESS, DATASHARING, TRANSPORT
  LocatorList announced_locators
  LocatorList used_locators
```

* *incompatible_qos_status:* Status of the Incompatible QoS of that entity.
  * [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) Incompatible QoS Offered.
  * [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) Incompatible QoS Requested.
* *inconsistent_topic_status:* Status of Inconsistent topics of the topic of that entity.
  Asked to the topic of the requested entity.
* *liveliness_lost_status:* Tracks the status of the number of times that liveliness was lost by a
  [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter).
* *liveliness_changed_status:* Tracks the status of the number of times that liveliness was lost
  by a [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
* *deadline_status:* The Status of the number of deadlines missed that were registered in that entity.
* *sample_lost_status:* The number of samples that entity lost.
* *extended_incompatible_qos_status:* Details a list of remote GUIDs with incompatible QoS with
  a local entity and the particular [`QosPolicyId_t`](../../api_reference/dds_pim/core/policy/qospolicyid_t.md#_CPPv4N8eprosima7fastdds3dds13QosPolicyId_tE) of those incompatible policies.

```bash
ExtendedIncompatibleQoSStatus_s
  GUID remote_guid
  sequence<unsigned long> current_incompatible_policies

ExtendedIncompatibleQoSStatusSeq = sequence<ExtendedIncompatibleQoSStatus>
```

The following table depicts the relation between each of the `StatusKind` values and the `Data` field:

|   **StatusKind** | **StatusKind Name**       | **Data field Name**              | **IDL Data field Type**          |
|------------------|---------------------------|----------------------------------|----------------------------------|
|                0 | `ProxyInfo`               | entity_proxy                     | sequence<octet>                  |
|                1 | `ConnectionList`          | connection_list                  | sequence<Connection>             |
|                2 | `IncompatibleQoSInfo`     | incompatible_qos_status          | IncompatibleQoSStatus            |
|                3 | `InconsistentTopicInfo`   | inconsistent_topic_status        | InconsistentTopicStatus          |
|                4 | `LivelinessLostInfo`      | liveliness_lost_status           | LivelinessLostStatus             |
|                5 | `LivelinessLostInfo`      | liveliness_changed_status        | LivelinessChangedStatus          |
|                6 | `DeadlineMissedInfo`      | deadline_missed_status           | DeadlineMissedStatus             |
|                7 | `SampleLostInfo`          | sample_lost_status               | SampleLostStatus                 |
|                8 | `ExtendedIncompatibleQoS` | extended_incompatible_qos_status | ExtendedIncompatibleQoSStatusSeq |
