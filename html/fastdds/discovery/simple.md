<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="simple-disc-settings"></a>

<a id="simple-discovery"></a>

# SIMPLE Discovery

The SIMPLE discovery protocol resolves the establishment of the end-to-end connection between various DDS Entities.
*eProsima Fast DDS* implements the SIMPLE discovery protocol to provide compatibility with the
[RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF).
The specification splits up the SIMPLE discovery protocol into two independent protocols:

- **Simple Participant Discovery Protocol (SPDP):** specifies how DomainParticipants discover each other in the
  network; it announces and detects the presence of DomainParticipants within the same domain.
- **Simple Endpoint Discovery Protocol (SEDP):** defines the protocol adopted by the discovered DomainParticipants for
  the exchange of information in order to discover the DDS Entities contained in each of them, i.e. the [DataWriter](../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and
  [DataReader](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).

| Name                                            | Description                                                                               |
|-------------------------------------------------|-------------------------------------------------------------------------------------------|
| [Initial Announcements](#initial-announcements) | It defines the behavior of the DomainParticipants initial announcements.                  |
| [Simple EDP Attributes](#simple-edp-attributes) | It defines the use of the SIMPLE protocol as a discovery protocol.                        |
| [Initial peers](#simple-initial-peers)          | A list of DomainParticipant’s IP/port pairs to which the SPDP announcements<br/>are sent. |

<a id="initial-announcements"></a>

<a id="id1"></a>

## Initial Announcements

[RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) simple discovery mechanism requires the
DomainParticipants to send announcements of their presence in the domain.
These announcements are not delivered in a reliable fashion, and can be disposed of by the network.
In order to avoid the discovery delay induced by message disposal, the initial announcement can be set up to make
several shots, in order to increase proper reception chances.
See [`InitialAnnouncementConfig`](../api_reference/rtps/attributes/InitialAnnouncementConfig.md#_CPPv4N8eprosima7fastdds4rtps25InitialAnnouncementConfigE).

Initial announcements only take place upon participant creation.
Once this phase is over, the only announcements enforced are the standard ones based on the
[`leaseDuration_announcementperiod`](../api_reference/rtps/attributes/DiscoverySettings.md#_CPPv4N8eprosima7fastdds4rtps17DiscoverySettings32leaseDuration_announcementperiodE) period (not the [`period`](../api_reference/rtps/attributes/InitialAnnouncementConfig.md#_CPPv4N8eprosima7fastdds4rtps25InitialAnnouncementConfig6periodE)).

| Name   | Description                                                 | Type         | Default   |
|--------|-------------------------------------------------------------|--------------|-----------|
| count  | It defines the number of announcements to send at start-up. | `uint32_t`   | 5         |
| period | It defines the specific period for initial announcements.   | `Duration_t` | 100ms     |

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.initial_announcements.count = 5;
pqos.wire_protocol().builtin.discovery_config.initial_announcements.period = Duration_t(0, 100000000u);
```

### XML

```xml
<participant profile_name="participant_profile_simple_discovery">
    <rtps>
        <builtin>
            <discovery_config>
                <initialAnnouncements>
                    <count>5</count>
                    <period>
                        <nanosec>100000000</nanosec>
                    </period>
                </initialAnnouncements>
            </discovery_config>
        </builtin>
    </rtps>
</participant>
```

<a id="simple-edp-attributes"></a>

<a id="id3"></a>

## Simple EDP Attributes

| Name                                           | Description                                                                                                                                                                                                   | Type   | Default   |
|------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|-----------|
| SIMPLE EDP                                     | It defines the use of the SIMPLE protocol as a discovery<br/>protocol for the EDP phase. A DomainParticipant may<br/>create DataWriters, DataReaders, both or neither.                                        | `bool` | true      |
| Publication writer and<br/>Subscription reader | It is intended for DomainParticipants that implement only<br/>one or more DataWriters, i.e. do not implement DataReaders.<br/>It allows the creation of only DataReader discovery<br/>related EDP endpoints.  | `bool` | true      |
| Publication reader and<br/>Subscription writer | It is intended for DomainParticipants that implement only<br/>one or more DataReaders, i.e. do not implement DataWriters.<br/>It allows the creation of only DataWriter discovery<br/>related  EDP endpoints. | `bool` | true      |

### C++

```c++
DomainParticipantQos pqos;

pqos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
pqos.wire_protocol().builtin.discovery_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = false;
```

### XML

```xml
<participant profile_name="participant_profile_qos_discovery_edp">
    <rtps>
        <builtin>
            <discovery_config>
                <EDP>SIMPLE</EDP>
                <simpleEDP>
                    <PUBWRITER_SUBREADER>true</PUBWRITER_SUBREADER>
                    <PUBREADER_SUBWRITER>false</PUBREADER_SUBWRITER>
                </simpleEDP>
            </discovery_config>
        </builtin>
    </rtps>
</participant>
```

<a id="simple-initial-peers"></a>

<a id="initial-peers"></a>

## Initial peers

According to the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) (Section 9.6.1.1), each
[`RTPSParticipant`](../api_reference/rtps/participant/RTPSParticipant.md#_CPPv4N8eprosima7fastdds4rtps15RTPSParticipantE)
must listen for incoming Participant Discovery Protocol (PDP) discovery metatraffic in two different ports, one linked
to a multicast address and another one linked to a unicast address.
*Fast DDS* allows for the configuration of an initial peers list which contains one or more such IP-port address
pairs corresponding to remote DomainParticipants PDP discovery listening resources, so that the local
DomainParticipant will only send its PDP traffic to the IP-port address pairs specified in the initial peers list.

A DomainParticipant’s initial peers list contains the list of IP-port address pairs of all other DomainParticipants
with which it will communicate.
It is a list of addresses that a DomainParticipant will use in the PDP discovery mechanism, and can hold both multicast
and unicast addresses.
The default multicast address will be used if the list is empty.
Therefore, this approach also applies to those scenarios in which multicast functionality is not available.

According to the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) (Section 9.6.1.1), the RTPSParticipants’
discovery traffic unicast listening ports are calculated using the following equation:
7400 + 250 \* domainID + 10 + 2 \* participantID.
Thus, if for example a RTPSParticipant operates in Domain 0 (default
domain) and its ID is 1, its discovery traffic unicast listening port would be: 7400 + 250 \* 0 + 10 + 2 \* 1 = 7412.
By default *eProsima Fast DDS* uses as initial peers the Metatraffic Multicast Locators.

The following constitutes an example configuring an Initial Peers list with one peer on host 192.168.10.13 with
DomainParticipant ID 1 in domain 0.

#### NOTE
There is also the possibility of not defining the initial peer port.
In this case, the discovery information would be sent to every port ranging from participantID zero to the
`maxInitialPeersRange` value set in the [TransportDescriptorInterface](../transport/transport_api.md#transport-transportapi-transportdescriptor).
Consequently, setting this value to at least the maximum expected number of DomainParticipants will ensure discovery
and communication.

### C++

```c++
DomainParticipantQos qos;

// configure an initial peer on host 192.168.10.13.
// The port number corresponds to the well-known port for metatraffic unicast
// on participant ID `1` and domain `0`.
Locator_t initial_peer;
IPLocator::setIPv4(initial_peer, "192.168.10.13");
initial_peer.port = 7412;
qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer);
```

### XML

```xml
<!--
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
-->
    <participant profile_name="initial_peers_example_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>192.168.10.13</address>
                            <port>7412</port>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
```
