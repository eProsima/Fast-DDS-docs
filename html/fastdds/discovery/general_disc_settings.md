<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="discovery-general-settings"></a>

<a id="general-discovery-settings"></a>

# General Discovery Settings

Some discovery settings are shared across the different discovery mechanisms.
These settings are defined under the [`builtin`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos7builtinE) public data member of the
[`WireProtocolConfigQos`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQosE) class.
These are:

| Name                                                | Description                                                                                                                                                                                                               | Type                                                                                                                                                   | Default                                                                                                                                          |
|-----------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| [Discovery Protocol](#discovery-protocol)           | The discovery protocol to use<br/>(see [Discovery mechanisms](discovery.md#disc-mechanisms)).                                                                                                                             | [`DiscoveryProtocol`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocolE)                         | [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE)                       |
| [Ignore Participant flags](#discovery-ignore-flags) | Filter discovery traffic for<br/>[DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) in the<br/>same process, in different<br/>processes, or in different hosts. | [`ParticipantFilteringFlags`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlagsE) | [`NO_FILTER`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags9NO_FILTERE) |
| [Lease Duration](#discovery-lease-dur)              | Indicates for how much time<br/>should a remote<br/>DomainParticipant    consider<br/>the local DomainParticipant<br/>to be alive.                                                                                        | `Duration_t`                                                                                                                                           | 20 s                                                                                                                                             |
| [Announcement Period](#discovery-lease-announ)      | The period for the<br/>DomainParticipant<br/>to send PDP announcements.                                                                                                                                                   | `Duration_t`                                                                                                                                           | 3 s                                                                                                                                              |

<a id="discovery-protocol"></a>

<a id="id1"></a>

## Discovery Protocol

Specifies the discovery protocol to use (see [Discovery mechanisms](discovery.md#disc-mechanisms)).
The possible values are:

| Discovery Mechanism                                                                                                                     | Possible values                                                                                                                                                                                                                                                                                                                                                                                                                      | Description                                                                                                                                                                                                                                                                                                                                                   |
|-----------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Simple                                                                                                                                  | [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE)                                                                                                                                                                                                                                                                                                           | Simple discovery protocol as specified in the<br/>[RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF).                                                                                                                                                                                                                                                |
| Discovery Server                                                                                                                        | [`SERVER`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE)                                                                                                                                                                                                                                                                                                           | The DomainParticipant acts as a hub for discovery traffic, receiving<br/>and distributing discovery information.                                                                                                                                                                                                                                              |
| [`CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6CLIENTE)              | The DomainParticipant acts as a client for discovery traffic.<br/>It sends its discovery information to the server.<br/>**[Pro]** It receives only the information that is relevant to it; in the<br/>open-source version it receives all the discovery information, thus<br/>behaving as a [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE). |                                                                                                                                                                                                                                                                                                                                                               |
| [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE) | The DomainParticipant acts as a client for discovery traffic.<br/>It sends its discovery information to the server, and it receives<br/>all other discovery information from the server.                                                                                                                                                                                                                                             |                                                                                                                                                                                                                                                                                                                                                               |
| [`BACKUP`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6BACKUPE)              | Creates a SERVER DomainParticipant which has a persistent `sqlite`<br/>database. A BACKUP server can load the a database on start.<br/>This type of sever makes the Discovery Server architecture<br/>resilient to server destruction.                                                                                                                                                                                               |                                                                                                                                                                                                                                                                                                                                                               |
| [PDP Relay](pdp_relay.md#pdp-relay)                                                                                                     | [`PDP_RELAY`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol9PDP_RELAYE)                                                                                                                                                                                                                                                                                                     | **[Pro]** The DomainParticipant only re-distributes the participant<br/>announcements it receives, so that [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) DomainParticipants can<br/>discover each other without multicast. It runs no EDP and does not sit<br/>in the data path. |
| Manual                                                                                                                                  | [`NONE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol4NONEE)                                                                                                                                                                                                                                                                                                               | Disables PDP phase, therefore there is no EDP phase.<br/>All matching must be done manually through the<br/>`addReaderLocator`, `addReaderProxy`, `addWriterProxy`<br/>RTPS layer methods.                                                                                                                                                                    |

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        DiscoveryProtocol::SIMPLE;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="participant_discovery_protocol">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SIMPLE</discoveryProtocol>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="discovery-ignore-flags"></a>

<a id="ignore-participant-flags"></a>

## Ignore Participant flags

Defines a filter to ignore some discovery traffic when received.
This is useful to add an extra level of DomainParticipant isolation.
The possible values are:

| Possible values                                                                                                                                                                                                                                                                                                                                         | Description                                                               |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------|
| [`NO_FILTER`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags9NO_FILTERE)                                                                                                                                                                                                        | All Discovery traffic is processed.                                       |
| [`FILTER_DIFFERENT_HOST`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags21FILTER_DIFFERENT_HOSTE)                                                                                                                                                                               | Discovery traffic from another host is discarded.                         |
| [`FILTER_DIFFERENT_PROCESS`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags24FILTER_DIFFERENT_PROCESSE)                                                                                                                                                                         | Discovery traffic from another process on the same host<br/>is discarded. |
| [`FILTER_SAME_PROCESS`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags19FILTER_SAME_PROCESSE)                                                                                                                                                                                   | Discovery traffic from DomainParticipant’s own<br/>process is discarded.  |
| [`FILTER_DIFFERENT_PROCESS`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags24FILTER_DIFFERENT_PROCESSE) | [`FILTER_SAME_PROCESS`](../api_reference/rtps/attributes/ParticipantFilteringFlags.md#_CPPv4N8eprosima7fastdds4rtps25ParticipantFilteringFlags19FILTER_SAME_PROCESSE) | Discovery traffic from DomainParticipant’s own<br/>host is discarded.     |

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.ignoreParticipantFlags =
        static_cast<eprosima::fastdds::rtps::ParticipantFilteringFlags>(
    ParticipantFilteringFlags::FILTER_DIFFERENT_PROCESS |
    ParticipantFilteringFlags::FILTER_SAME_PROCESS);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="participant_discovery_ignore_flags">
            <rtps>
                <builtin>
                    <discovery_config>
                        <ignoreParticipantFlags>FILTER_DIFFERENT_PROCESS | FILTER_SAME_PROCESS</ignoreParticipantFlags>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### NOTE
To configure a [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) to not receive data from its own [DataWriters](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), please refer to [Ignore Local Endpoints](../property_policies/ignore_local_endpoints.md#property-ignore-local-endpoints).

<a id="discovery-lease-dur"></a>

<a id="lease-duration"></a>

## Lease Duration

Indicates for how much time should a remote DomainParticipant consider the local DomainParticipant to be alive.
If the liveliness of the local DomainParticipant has not being asserted within this time, the remote
DomainParticipant considers the local DomainParticipant dead and destroys all the information regarding the local
DomainParticipant and all its endpoints.

The local DomainParticipant’s liveliness is asserted on the remote DomainParticipant any time the remote
DomainParticipant receives any kind of traffic from the local DomainParticipant.

The lease duration is specified as a time expressed in seconds and nanosecond using a `Duration_t`.

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.leaseDuration = Duration_t(10, 20);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="participant_discovery_lease_duration">
            <rtps>
                <builtin>
                    <discovery_config>
                        <leaseDuration>
                            <sec>10</sec>
                            <nanosec>20</nanosec>
                        </leaseDuration>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="discovery-lease-announ"></a>

<a id="announcement-period"></a>

## Announcement Period

It specifies the periodicity of the DomainParticipant’s PDP announcements.  For liveliness’ sake it is recommend that
the announcement period is shorter than the lease duration, so that the DomainParticipant’s liveliness is asserted
even when there is no data traffic. It is important to note that there is a trade-off involved in the setting of the
announcement period, i.e. too frequent announcements will bloat the network with meta traffic, but too scarce ones will
delay the discovery of late joiners.

DomainParticipant’s announcement period is specified as a time expressed in seconds and nanosecond using a
`Duration_t`.

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(1, 2);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <participant profile_name="participant_discovery_lease_announcement">
            <rtps>
                <builtin>
                    <discovery_config>
                        <leaseAnnouncement>
                            <sec>1</sec>
                            <nanosec>2</nanosec>
                        </leaseAnnouncement>
                    </discovery_config>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```
