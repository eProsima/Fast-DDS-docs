<a id="use-case-fast-rtps-over-wifi"></a>

<a id="fast-dds-over-wifi"></a>

# Fast DDS over WIFI

The [RTPS v2.2 standard](https://www.omg.org/spec/DDSI-RTPS/2.2/) defines the SIMPLE [Discovery](../../discovery/discovery.md#discovery) as the default
mechanism for discovering participants in the network.
One of the main features of this mechanism is the use of multicast communication in the Participant Discovery
Phase (PDP).
This can be a problem in cases where WiFi communication is used, since multicast is not as reliable over WiFi
as it is over ethernet.

The recommended solution to this challenge is to configure an initial list of remote peers on the
[DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), so that it can set unicast communication with them.
This way, the use of multicast is not needed to discover these initial peers.
Furthermore, if all the peers are known and configured beforehand, all multicast communication can be
removed.

Alternatively, **Discovery Server** can be used to avoid multicast discovery.
A DomainParticipant with a well-know address acts as a discovery server,
providing the rest of the participants the information required to connect among them.
If all the peers are known and configured beforehand, STATIC discovery can be used instead,
completely avoiding the discovery phase.
Use-case [Well Known Network Deployments](../well_known_deployments/well_known_deployments.md#well-known-deployments) provides a detailed explanation on how to configure *Fast DDS* for
STATIC discovery.

* [Configuring Initial Peers](initial_peers.md)
* [Disabling multicast discovery](disable_multicast.md)
* [Discovery Server](discovery_server_use_case.md)
  * [UDPv4 basic example setup](discovery_server_use_case.md#udpv4-basic-example-setup)
  * [UDPv4 redundancy example](discovery_server_use_case.md#udpv4-redundancy-example)
  * [UDPv4 persistency example](discovery_server_use_case.md#udpv4-persistency-example)
  * [UDPv4 partitioning using servers](discovery_server_use_case.md#udpv4-partitioning-using-servers)
