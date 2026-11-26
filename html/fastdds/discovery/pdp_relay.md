<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="pdp-relay"></a>

<a id="pdp-relay-pro"></a>

# PDP Relay **[Pro]**

A *PDP Relay* is a [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) whose only job is to re-distribute the participant
announcements (`DATA(p)`) that it receives, so that plain [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) DomainParticipants can discover
each other without multicast.

<a id="pdp-relay-why"></a>

<a id="why-use-a-pdp-relay"></a>

## Why use a PDP Relay

In the [Simple Discovery](simple.md#simple-disc-settings) protocol, each DomainParticipant announces
itself over multicast and listens for the announcements of the others.
When multicast is not available, which is common in segmented networks, cloud environments, WiFi
links, restrictive switches and containers, SIMPLE DomainParticipants never see each other, even
though they could communicate perfectly well once discovered.

The usual alternative is a [Discovery Server](discovery_server.md#discovery-server), but it requires every
DomainParticipant in the system to be reconfigured as [`CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6CLIENTE) or [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE), which changes the
discovery semantics of the whole deployment.
It is also an extension of the [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF)
specific to *Fast DDS*, so every DomainParticipant taking part in it must be a *Fast DDS* one.

A PDP Relay keeps the DomainParticipants SIMPLE.
The only change needed on them is to point their [initial peers](../use_cases/wifi/initial_peers.md#use-case-initial-peers) list at the
relay and disable multicast.
Endpoint discovery, QoS matching, liveliness and lease duration all behave exactly as in a normal
SIMPLE deployment.
Since they keep using the standard Simple Discovery protocol, a PDP Relay works with
DomainParticipants of **any vendor**: their announcements are re-distributed verbatim, so vendor
specific parameters survive the hop untouched.

#### IMPORTANT
A PDP Relay removes the dependency on multicast, and nothing else.
It does not reduce the discovery traffic: it filters nothing, and once two DomainParticipants
have discovered each other they announce directly to each other, as they would over multicast.
If the goal is to cut down discovery traffic, use a
[Discovery Server](discovery_server.md#discovery-server) instead.

Use a PDP Relay when:

* Multicast is unavailable or unreliable, **and**
* The DomainParticipants should not become Discovery Server clients, or they are not *Fast DDS*
  DomainParticipants at all, **and**
* Each DomainParticipant should keep its own SIMPLE liveliness and lease management.

If a full discovery database, discovery traffic reduction, endpoint discovery filtering, server
redundancy or secure discovery is needed, use a [Discovery Server](discovery_server.md#discovery-server) instead.

<a id="pdp-relay-how"></a>

<a id="how-it-works"></a>

## How it works

![image](01-figures/pdp_relay_diagram.png)

*Discovery of two SIMPLE DomainParticipants through a PDP Relay*

1. Each SIMPLE DomainParticipant has the relay as its only initial peer, so it sends its `DATA(p)`
   announcement directly to the relay.
2. The relay stores that announcement if the domain matches and re-publishes it verbatim to every DomainParticipant it
   knows, including the ones that announce later.
   If the domain does not match, the announcement is dropped and never re-published.
3. When a DomainParticipant receives a relayed `DATA(p)`, it learns the unicast locators of the
   announced DomainParticipant and starts announcing directly to it.
4. From that point on the relay is no longer involved: PDP announcements, lease refresh, endpoint
   discovery and user data all flow directly between the two DomainParticipants.

The relay therefore only introduces the DomainParticipants to each other. It does not sit in the
data path, and they keep communicating if it is stopped.

<a id="pdp-relay-configuration"></a>

<a id="configuring-a-pdp-relay"></a>

## Configuring a PDP Relay

The [discovery protocol](general_disc_settings.md#discovery-protocol) value is [`PDP_RELAY`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol9PDP_RELAYE).

A relay needs a well-known listening address, since that is the address the DomainParticipants set
as their initial peer.
The address must therefore be given explicitly, and *Fast DDS* will not silently move the relay to
another port if the configured one is taken.
The PDP Relay can listen on UDP, TCP or both.

The relay DomainParticipant needs no Publisher, Subscriber, Topic or data type: creating the
DomainParticipant is enough.

### C++

```c++
DomainParticipantQos relay_qos;
relay_qos.name("PDP Relay");

// Set the participant as a PDP Relay
relay_qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        DiscoveryProtocol::PDP_RELAY;

// Well-known listening locator: the address the participants will use as initial peer
Locator_t listening_locator;
listening_locator.kind = LOCATOR_KIND_UDPv4;
listening_locator.port = 11811;
IPLocator::setIPv4(listening_locator, "0.0.0.0");
relay_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(listening_locator);

// The relay is bound to a domain: it only re-distributes announcements of that domain
DomainParticipant* relay =
        DomainParticipantFactory::get_instance()->create_participant(0, relay_qos);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <participant profile_name="pdp_relay" is_default_profile="true">
            <rtps>
                <name>PDP Relay</name>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>PDP_RELAY</discoveryProtocol>
                    </discovery_config>
                    <!-- Well-known listening address -->
                    <metatrafficUnicastLocatorList>
                        <locator>
                            <udpv4>
                                <address>0.0.0.0</address>
                                <port>11811</port>
                            </udpv4>
                        </locator>
                    </metatrafficUnicastLocatorList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="pdp-relay-participants"></a>

<a id="configuring-the-domainparticipants-that-use-the-relay"></a>

## Configuring the DomainParticipants that use the relay

The DomainParticipants stay [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE). They only need to:

1. Add the relay listening locator to their [initial peers](../use_cases/wifi/initial_peers.md#use-case-initial-peers) list.
2. Disable multicast metatraffic, so that discovery goes exclusively through the relay.
   Setting a metatraffic unicast locator keeps the DomainParticipant from taking the default
   locators, multicast included.

### C++

```c++
DomainParticipantQos participant_qos;

// The participants using the relay stay SIMPLE
participant_qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        DiscoveryProtocol::SIMPLE;

// Point the initial peers list at the relay
Locator_t relay_locator;
relay_locator.kind = LOCATOR_KIND_UDPv4;
relay_locator.port = 11811;
IPLocator::setIPv4(relay_locator, "127.0.0.1");
participant_qos.wire_protocol().builtin.initialPeersList.push_back(relay_locator);

// Disable multicast metatraffic: setting a metatraffic unicast locator keeps the
// participant from taking the default locators, multicast included. Leaving its port
// unset lets the participant use its usual metatraffic port.
Locator_t unicast_locator;
unicast_locator.kind = LOCATOR_KIND_UDPv4;
participant_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(unicast_locator);

DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <participant profile_name="pdp_relay_initial_peer">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SIMPLE</discoveryProtocol>
                    </discovery_config>
                    <!-- Point the initial peers list at the relay -->
                    <initialPeersList>
                        <locator>
                            <udpv4>
                                <address>127.0.0.1</address>
                                <port>11811</port>
                            </udpv4>
                        </locator>
                    </initialPeersList>
                    <!-- Disable multicast metatraffic -->
                    <metatrafficUnicastLocatorList>
                        <locator>
                            <udpv4/>
                        </locator>
                    </metatrafficUnicastLocatorList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="pdp-relay-cli"></a>

<a id="launching-a-pdp-relay-with-the-cli"></a>

## Launching a PDP Relay with the CLI

The [fastdds discovery](../../fastddscli/cli/cli.md#cli-discovery) tool creates a PDP Relay instead of a Discovery Server
when the `--relay` argument is given.
The following command listens on all available interfaces, UDP port `11811`, domain `0`:

```bash
fastdds discovery --relay
```

The relevant arguments are:

| Argument                                   | Description                                       |
|--------------------------------------------|---------------------------------------------------|
| `-r`, `--relay`                            | Create a PDP Relay instead of a Discovery Server. |
| `-d`, `--domain`                           | DDS domain the relay serves. Defaults to `0`.     |
| `-l`, `--udp-address` / `-p`, `--udp-port` | UDP listening address and port.                   |
| `-t`, `--tcp-address` / `-q`, `--tcp-port` | TCP listening address and port.                   |
| `-x`, `--xml-file`                         | Take the configuration from an XML profile.       |
```bash
# Relay for domain 5, listening on localhost over TCP port 42100
fastdds discovery -r -d 5 -t 127.0.0.1 -q 42100

# Relay for domain 2, listening on a specific interface over UDP
fastdds discovery -r -d 2 -l 192.168.1.10 -p 11811

# Relay taking its configuration from an XML profile
fastdds discovery -x pdp_relay@relay.xml
```

Which outputs:

```bash
#### PDP Relay started ####
  Domain ID:   0
  GUID prefix: 44.53.00.5f.45.50.52.4f.53.49.4d.41
  Running on:  UDPv4:[0.0.0.0]:11811
```

<a id="pdp-relay-troubleshooting"></a>

<a id="troubleshooting"></a>

## Troubleshooting

| Symptom                                                                                                                                                                          | Likely cause                                                                                                                                      |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------|
| The DomainParticipants do not discover each other after launching the relay                                                                                                      | The initial peer address and port do not match the relay listening locator, or the<br/>DomainParticipants and the relay are on different domains. |
| The DomainParticipants discover each other with the relay stopped                                                                                                                | Multicast metatraffic was not disabled on the DomainParticipants.                                                                                 |
| `fastdds discovery -r` fails to start                                                                                                                                            | The listening port is already taken. The relay does not fall back to another port, since<br/>its address must be well known.                      |
| DomainParticipant creation fails with a [`PDP_RELAY`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol9PDP_RELAYE) profile | Security is enabled. The PDP Relay does not support security.                                                                                     |
