<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="intraprocess-delivery"></a>

<a id="intra-process-delivery"></a>

# Intra-process delivery

eProsima Fast DDS allows to speed up communications between entities within the same process by avoiding any of the
overhead involved in the transport layer.
Instead, the [Publisher](../dds_layer/publisher/publisher/publisher.md#dds-layer-publisher-publisher) directly calls the reception functions of the
[Subscriber](../dds_layer/subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber).
This not only avoids the copy or send operations of the transport, but also ensures the message is received by the
Subscriber, avoiding the acknowledgement mechanism.

This feature is enabled by default, and can be configured using [XML profiles](../xml_configuration/xml_configuration.md#xml-profiles)
(see [Intra-process delivery profiles](../xml_configuration/library_settings.md#intra-process-delivery-xml-profile)).
Currently the following options are available:

* **INTRAPROCESS_OFF**: The feature is disabled.
* **INTRAPROCESS_USER_DATA_ONLY**: Discovery metadata keeps using ordinary transport.
* **INTRAPROCESS_FULL**: Default value. Both user data and discovery metadata using Intra-process delivery.

| **XML**                                                                                                                                                        |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ```xml<br/><library_settings><br/>    <intraprocess_delivery>FULL</intraprocess_delivery> <!-- OFF | USER_DATA_ONLY | FULL --><br/></library_settings><br/>``` |

<a id="intraprocess-delivery-guids"></a>

<a id="guid-prefix-considerations-for-intra-process-delivery"></a>

## GUID Prefix considerations for intra-process delivery

Fast DDS utilizes the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)’s [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) to identify peers running in the same process.
Two participants with identical 8 first bytes on the [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) are considered to be running in the same
process, and therefore intra-process delivery is used.
[`is_on_same_process_as()`](../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4NK8eprosima7fastdds4rtps6GUID_t21is_on_same_process_asERK6GUID_t) API is provided to check this condition.
This mechanism works out-of-the-box when letting Fast DDS set the GUID prefixes for the created DomainParticipants.
However, special consideration is required when setting the [`GuidPrefix_t`](../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) manually, either programmatically or when
using XML.

#### IMPORTANT
Fast DDS assigns GUID prefixes considering several host parameters.
Among them, the network interfaces enabled.
Thus, if at runtime, the network interfaces change, any new DomainParticipant will have a different GUID prefix and
will be considered to be running on another host.

### C++ - ASCII format.

```c++
eprosima::fastdds::rtps::GuidPrefix_t guid_prefix;
guid_prefix.value[0] = eprosima::fastdds::rtps::octet(0x77);
guid_prefix.value[1] = eprosima::fastdds::rtps::octet(0x73);
guid_prefix.value[2] = eprosima::fastdds::rtps::octet(0x71);
guid_prefix.value[3] = eprosima::fastdds::rtps::octet(0x85);
guid_prefix.value[4] = eprosima::fastdds::rtps::octet(0x69);
guid_prefix.value[5] = eprosima::fastdds::rtps::octet(0x76);
guid_prefix.value[6] = eprosima::fastdds::rtps::octet(0x95);
guid_prefix.value[7] = eprosima::fastdds::rtps::octet(0x66);
guid_prefix.value[8] = eprosima::fastdds::rtps::octet(0x65);
guid_prefix.value[9] = eprosima::fastdds::rtps::octet(0x82);
guid_prefix.value[10] = eprosima::fastdds::rtps::octet(0x82);
guid_prefix.value[11] = eprosima::fastdds::rtps::octet(0x79);

DomainParticipantQos participant_qos;
participant_qos.wire_protocol().prefix = guid_prefix;
// Use modified QoS in the creation of the DomainParticipant entity
participant_ = factory_->create_participant(domain, participant_qos);
```

### C++ - Extraction operator.

```c++
DomainParticipantQos participant_qos;
std::istringstream("77.73.71.85.69.76.95.66.65.82.82.79") >> participant_qos.wire_protocol().prefix;
// Use modified QoS in the creation of the DomainParticipant entity
participant_ = factory_->create_participant(domain, participant_qos);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_guidprefix" >
        <rtps>
            <prefix>77.73.71.85.69.76.95.66.65.82.82.79</prefix>
        </rtps>
    </participant>
</profiles>
```
