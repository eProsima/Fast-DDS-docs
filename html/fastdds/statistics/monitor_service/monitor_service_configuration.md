<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="monitor-service-configuration"></a>

<a id="id1"></a>

# Monitor Service Configuration

In *Fast DDS*, the [Monitor Service](monitor_service.md#monitor-service) can be activated using the `-DFASTDDS_STATISTICS=ON` at CMake
configuration step (for further information regarding the *Fast DDS* compilation, see [Linux installation from sources](../../../installation/sources/sources_linux.md#linux-sources) and
[Windows installation from sources](../../../installation/sources/sources_windows.md#windows-sources)).
**[Pro]** In *Fast DDS* **[Pro]**, it is always compiled in.
Once the [Monitor Service](monitor_service.md#monitor-service) feature is activated, it can be programmatically enabled in both [DDS Layer](../../dds_layer/dds_layer.md#dds-layer) and
[RTPS Layer](../../rtps_layer/rtps_layer.md#rtps-layer) through the [`enable_monitor_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22enable_monitor_serviceEv) and
[`disable_monitor_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant23disable_monitor_serviceEv) calls.
In addition, leveraging the [`PropertyPolicyQos`](../../api_reference/dds_pim/core/policy/propertypolicyqos.md#_CPPv4N8eprosima7fastdds3dds17PropertyPolicyQosE) there is new `Property` defined
for the purpose: `fastdds.enable_monitor_service`.

The following table depicts the different ways in which the *Monitor Service* can be enabled or disabled:

### C++ API

```c++

DomainParticipant* participant = DomainParticipantFactory::get_instance()->create_participant(0,
                PARTICIPANT_QOS_DEFAULT);

// Enable Fast DDS Monitor Service through API
participant->enable_monitor_service();

// Disable Fast DDS Monitor Service through API
participant->disable_monitor_service();


```

### C++ Property

```c++
DomainParticipantQos pqos;

// Enable Fast DDS Monitor Service through properties
pqos.properties().properties().emplace_back("fastdds.enable_monitor_service",
        "true");

// Enable Fast DDS Monitor Service through statistics properties (other way)
pqos.properties().properties().emplace_back("fastdds.statistics",
        "MONITOR_SERVICE_TOPIC");

DomainParticipant* participant_with_mon_srv = DomainParticipantFactory::get_instance()->create_participant(0,
                pqos);

```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
<participant profile_name="monitor_service_participant_xml_profile">
    <rtps>
        <propertiesPolicy>
            <properties>
                <!-- Enable Monitor Service -->
                <property>
                    <name>fastdds.enable_monitor_service</name>
                    <value>true</value>
                </property>
                <!-- Include Monitor Service in the Fast DDS Statistics (other way to enable it) -->
                <property>
                    <name>fastdds.statistics</name>
                    <value>MONITOR_SERVICE_TOPIC</value>
                </property>
            </properties>
        </propertiesPolicy>
    </rtps>
</participant>
</profiles>
```

### Env. Variable Linux

```bash
export FASTDDS_STATISTICS="MONITOR_SERVICE_TOPIC"
```

### Env. Variable Windows

```bash
set FASTDDS_STATISTICS=MONITOR_SERVICE_TOPIC
```

<a id="endpoints-qos"></a>

## Endpoints QoS

For any consumer application of the [Monitor Service](monitor_service.md#monitor-service), the following endpoint QoS
of the [Monitor Service Status Topic](monitor_service_topics.md#monitor-service-status-topic) DataWriter should be taken into consideration:

| **Endpoint**                                                                                                                                           | **QoS**                                                                                                                                                                                       | **Value**                                                                                                                                                                           |
|--------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| MONITOR_STATUS_WRITER                                                                                                                                  | [`ReliabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE)                                     | [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) |
| [`HistoryQosPolicyKind`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE)          | [`KEEP_LAST_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE) 1                       |                                                                                                                                                                                     |
| [`DurabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE) | [`TRANSIENT_LOCAL_DURABILITY_QOS`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind30TRANSIENT_LOCAL_DURABILITY_QOSE) |                                                                                                                                                                                     |
