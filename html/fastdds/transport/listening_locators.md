<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="listening-locators"></a>

<a id="id1"></a>

# Listening Locators

Listening [Locators](transport_api.md#transport-transportapi-locator) are used to receive incoming traffic on the
[DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
These Locators can be classified according to the communication type
and to the nature of the data.

According to the communication type we have:

* **Multicast locators**: Listen to multicast communications.
* **Unicast locators**: Listen to unicast communications.

According to the nature of the data we have:

* **Metatraffic locators**: Used to receive metatraffic information, usually used by built-in endpoints to perform
  discovery.
* **User locators**: Used by the endpoints created by the user to receive user [Topic](../dds_layer/topic/topic/topic.md#dds-layer-topic-topic)
  data changes.

Applications can [provide their own Listening Locators](#listening-locators-adding),
or use the [Default Listening Locators](#listening-locators-default) provided by *eProsima Fast DDS*.

<a id="listening-locators-adding"></a>

<a id="adding-listening-locators"></a>

## Adding Listening Locators

Users can add custom Listening Locators to the
DomainParticipant using the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos).
Depending on the field where the Locator is added,
it will be treated as a *multicast*, *unicast*, *user* or *metatraffic*
Locator.

#### NOTE
Both UDP and TCP unicast Locators support to have a null address.
In that case, *Fast DDS* automatically gets and uses local network addresses.

#### NOTE
Both UDP and TCP Locators support to have a zero port.
In that case, *Fast DDS* automatically calculates and uses well-known ports for that type of traffic.
See [Well Known Ports](#listening-locators-defaultports) for details about the well-known ports.

#### WARNING
TCP does not support multicast scenarios, so the network architecture must be carefully planned.

<a id="listening-locators-metamulticast"></a>

<a id="metatraffic-multicast-locators"></a>

### Metatraffic Multicast Locators

Users can set their own metatraffic multicast locators within the [WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos):
[`builtin.metatrafficMulticastLocatorList`](../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes31metatrafficMulticastLocatorListE).

### C++

```c++
DomainParticipantQos qos;

// This locator will open a socket to listen network messages
// on UDPv4 port 22222 over multicast address 239.255.0.1
eprosima::fastdds::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 239, 255, 0, 1);
locator.port = 22222;

// Add the locator to the DomainParticipantQos
qos.wire_protocol().builtin.metatrafficMulticastLocatorList.push_back(locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="CustomMetatrafficMulticastParticipant">
        <rtps>
            <builtin>
                <metatrafficMulticastLocatorList>
                    <!-- LOCATOR_LIST -->
                    <locator>
                        <udpv4>
                            <address>239.255.0.1</address>
                            <port>22222</port>
                        </udpv4>
                    </locator>
                </metatrafficMulticastLocatorList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

<a id="listening-locators-metaunicast"></a>

<a id="metatraffic-unicast-locators"></a>

### Metatraffic Unicast Locators

Users can set their own metatraffic unicast locators within the [WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos):
[`builtin.metatrafficUnicastLocatorList`](../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes29metatrafficUnicastLocatorListE).

### C++

```c++
DomainParticipantQos qos;

// This locator will open a socket to listen network messages
// on UDPv4 port 22223 over address 192.168.0.1
eprosima::fastdds::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 192, 168, 0, 1);
locator.port = 22223;

// Add the locator to the DomainParticipantQos
qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="CustomMetatrafficUnicastParticipant">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <!-- LOCATOR_LIST -->
                    <locator>
                        <udpv4>
                            <address>192.168.0.1</address>
                            <port>22223</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

<a id="listening-locators-usermulticast"></a>

<a id="user-traffic-multicast-locators"></a>

### User-traffic Multicast Locators

Users can set their own user-traffic multicast locators within the [WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos):
[`default_multicast_locator_list`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos30default_multicast_locator_listE).

### C++

```c++
DomainParticipantQos qos;

// This locator will open a socket to listen network messages
// on UDPv4 port 22224 over multicast address 239.255.0.1
eprosima::fastdds::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 239, 255, 0, 1);
locator.port = 22224;

// Add the locator to the DomainParticipantQos
qos.wire_protocol().default_multicast_locator_list.push_back(locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="CustomUsertrafficMulticastParticipant">
        <rtps>
            <defaultMulticastLocatorList>
                <!-- LOCATOR_LIST -->
                <locator>
                    <udpv4>
                        <address>239.255.0.1</address>
                        <port>22224</port>
                    </udpv4>
                </locator>
            </defaultMulticastLocatorList>
        </rtps>
    </participant>
</profiles>
```

<a id="listening-locators-userunicast"></a>

<a id="user-traffic-unicast-locators"></a>

### User-traffic Unicast Locators

Users can set their own user-traffic unicast locators within the [WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos):
[`default_unicast_locator_list`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos28default_unicast_locator_listE).

### C++

```c++
DomainParticipantQos qos;

// This locator will open a socket to listen network messages
// on UDPv4 port 22225 over address 192.168.0.1
eprosima::fastdds::rtps::Locator_t locator;
IPLocator::setIPv4(locator, 192, 168, 0, 1);
locator.port = 22225;

// Add the locator to the DomainParticipantQos
qos.wire_protocol().default_unicast_locator_list.push_back(locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="CustomUsertrafficUnicastParticipant">
        <rtps>
            <defaultUnicastLocatorList>
                <!-- LOCATOR_LIST -->
                <locator>
                    <udpv4>
                        <address>192.168.0.1</address>
                        <port>22225</port>
                    </udpv4>
                </locator>
            </defaultUnicastLocatorList>
        </rtps>
    </participant>
</profiles>
```

<a id="listening-locators-default"></a>

<a id="default-listening-locators"></a>

## Default Listening Locators

If the application does not define any Listening Locators,
*eProsima Fast DDS* automatically enables a set of listening UDPv4 locators by default.
This allows out-of-the-box communication in most cases, without the need of
further configuring the [Transport Layer](transport.md#comm-transports-configuration).

* If the application does not define any *metatraffic* Locator
  (neither *unicast* nor *multicast*), *Fast DDS* enables one *multicast*  Locator
  that will be used during [Discovery](../discovery/discovery.md#discovery), and one *unicast* Locator
  that will be used for peer-to-peer communication with already discovered
  DomainParticipants.
* If the application does not define any *user-traffic* Locator
  (neither *unicast* nor *multicast*), *Fast DDS* enables one *unicast* Locator
  that will be used for peer-to-peer communication of [Topic](../dds_layer/topic/topic/topic.md#dds-layer-topic-topic) data.
* If the application does not define any **participantId**, *Fast DDS* will use the value given by the
  [DomainParticipantFactory](../dds_layer/domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactory), which will try always to provide the lowest available value per
  DomainParticipantFactory (per process).

For example, it is possible to prevent *multicast* traffic adding a single *metatraffic unicast* Locator
as described in [Disabling all Multicast Traffic](disabling_multicast.md#transport-disablemulticast).

Default Listening Locators always use [Well Known Ports](#listening-locators-defaultports).

<a id="listening-locators-defaultports"></a>

<a id="well-known-ports"></a>

## Well Known Ports

The [DDSI-RTPS V2.2](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) standard (Section 9.6.1.1) defines a set of rules to calculate well-known
ports for default Locators, so that DomainParticipants can communicate with these default Locators.
Well-known ports are also selected automatically by *Fast DDS* when a Locator is configured with port number 0.

Well-known ports are calculated using the following predefined rules:

#### Rules to calculate ports on default listening locators

| Traffic type                 | Well-known port expression                                       |
|------------------------------|------------------------------------------------------------------|
| Metatraffic multicast        | `PB` + `DG` \* *domainId* + `offsetd0`                           |
| Metatraffic unicast          | `PB` + `DG` \* *domainId* + `offsetd1` + `PG` \* *participantId* |
| User multicast               | `PB` + `DG` \* *domainId* + `offsetd2`                           |
| User unicast                 | `PB` + `DG` \* *domainId* + `offsetd3` + `PG` \* *participantId* |
| Discovery Server (Easy Mode) | `PB` + `DG` \* *domainId* + `offsetd4`                           |

The values used in these rules are explained on the following table.
The default values can be modified using the [`port`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos4portE) member of the
[WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos) on the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos).

#### NOTE
> Deployments where multiple DomainParticipants are created within the same host can lead into denying
> available ports for them if the amount of DomainParticipants created reaches the value of
> [`BuiltinAttributes::mutation_tries`](../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes14mutation_triesE)
> (100 by default).
> When that happens, the DomainParticipants will not be able to create the listening ports (this is notified
> with a log warning) and will be created without unicast locators configured.

Refer to [this example](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos-example) for configuring both the *mutation_tries* values.

#### Values used in the rules to calculate well-known ports

| Symbol     | Meaning            |   Default value | QoS field                                                                                                                                                                   |
|------------|--------------------|-----------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `DG`       | DomainID gain      |             250 | [`wire_protocol().port.domainIDGain`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters12domainIDGainE)           |
| `PG`       | ParticipantId gain |               2 | [`wire_protocol().port.participantIDGain`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters17participantIDGainE) |
| `PB`       | Port Base number   |            7400 | [`wire_protocol().port.portBase`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters8portBaseE)                    |
| `offsetd0` | Additional offset  |               0 | [`wire_protocol().port.offsetd0`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters8offsetd0E)                    |
| `offsetd1` | Additional offset  |              10 | [`wire_protocol().port.offsetd1`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters8offsetd1E)                    |
| `offsetd2` | Additional offset  |               1 | [`wire_protocol().port.offsetd2`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters8offsetd2E)                    |
| `offsetd3` | Additional offset  |              11 | [`wire_protocol().port.offsetd3`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters8offsetd3E)                    |
| `offsetd4` | Additional offset  |               2 | [`wire_protocol().port.offsetd4`](../api_reference/rtps/common/PortParameters/PortParameters.md#_CPPv4N8eprosima7fastdds4rtps14PortParameters8offsetd4E)                    |
