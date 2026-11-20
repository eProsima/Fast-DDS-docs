<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-dds-tsn"></a>

<a id="dds-tsn-pro"></a>

# DDS-TSN **[Pro]**

<a id="tsn-intro"></a>

<a id="time-sensitive-networking-tsn"></a>

## Time-Sensitive Networking (TSN)

Time-Sensitive Networking (TSN) is a set of IEEE 802.1 standards that provide deterministic data delivery,
low latency, and high reliability over Ethernet networks.
TSN enables real-time communication for critical applications by ensuring that time-critical data is delivered
within strict timing constraints, even in the presence of non-critical traffic on the same network.

<a id="tsn-standards"></a>

<a id="omg-dds-tsn-specification"></a>

## OMG DDS-TSN Specification

The Object Management Group (OMG) created the
[DDS-TSN specification](https://www.omg.org/spec/DDS-TSN/1.0/Beta1/PDF/) to define standard mappings between
the Data Distribution Service (DDS) and Time-Sensitive Networking (TSN).
This integration allows DDS applications to leverage TSN capabilities to achieve deterministic, real-time
communication while maintaining the data-centric approach of DDS.

The specification defines:

* Mapping of DDS QoS policies to TSN capabilities
* Support for DDSI-RTPS over TSN, both for UDP/IP and raw Ethernet transports
* Flow configuration based on transport identifiers:
  : * UDPv4 / UDPv6: IPv4Tuple / IPv6Tuple
    * Ethernet: IEEE802MacAddresses & IEEE802VlanTag

<a id="tsn-fastdds"></a>

<a id="fast-dds-tsn-implementation"></a>

## Fast DDS TSN Implementation

*Fast DDS Pro* provides the required scaffolding for using DDS over TSN compliant with the OMG specification.

TSN flow configuration can be set via a mapping between a defined
[TransportPriorityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#transportpriorityqospolicy) value and a set of tuple parameters.
Then, any [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) which has been assigned such mapped
[TransportPriorityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#transportpriorityqospolicy) will use the corresponding tuple parameters for
its communication.

Note that a [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) can have its [TransportPriorityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#transportpriorityqospolicy) updated at any time, and the new tuple parameters will be applied to subsequent
sends.

<a id="supported-transports"></a>

### Supported transports

*Fast DDS Pro* currently supports two transports that can be used over TSN-capable networks:

- **UDP Transport Support (UDPv4 / UDPv6)**

  *Fast DDS* supports DDSI-RTPS communication over standard POSIX UDPv4 and UDPv6 sockets.
  *Fast DDS Pro* extends these transports to enable seamless operation on TSN-capable networks
  while preserving compatibility with existing DDS deployments.

  In this transport, the tuple parameters (IPv4Tuple / IPv6Tuple) are:
  * **Source IP address / port**
  * **Destination IP address / port**
  * **DSCP**
  * **Protocol**

  The protocol field is fixed to UDPv4 / UDPv6 by the selected
  [UDP Transport](../../transport/udp/udp.md#transport-udp-transportdescriptor).
  The rest of the fields can be set like in the following example:

  ### C++

  ```c++
  DomainParticipantQos qos;

  // Create a descriptor for the new transport.
  auto udp_transport = std::make_shared<TSN_UDPv4TransportDescriptor>();

  // Set the TSN-UDP tuples
  UDPPriorityMapping udp_priority_mapping_1;
  int32_t udp_priority_1 = -1;
  udp_priority_mapping_1.source_port = 5000;
  udp_transport->priority_mapping[udp_priority_1] = udp_priority_mapping_1;

  UDPPriorityMapping udp_priority_mapping_2;
  int32_t udp_priority_2 = 1;
  udp_priority_mapping_2.source_port = 5000;
  udp_priority_mapping_2.dscp = 1;
  udp_transport->priority_mapping[udp_priority_2] = udp_priority_mapping_2;

  UDPPriorityMapping udp_priority_mapping_3;
  int32_t udp_priority_3 = 2;
  udp_priority_mapping_3.source_port = 5000;
  udp_priority_mapping_3.dscp = 2;
  udp_priority_mapping_3.interface = "eth0";
  udp_transport->priority_mapping[udp_priority_3] = udp_priority_mapping_3;

  UDPPriorityMapping udp_priority_mapping_4;
  int32_t udp_priority_4 = 3;
  udp_priority_mapping_4.source_port = 5000;
  udp_priority_mapping_4.dscp = 3;
  udp_priority_mapping_4.interface = "10.10.1.2";
  udp_transport->priority_mapping[udp_priority_4] = udp_priority_mapping_4;

  // Link the Transport Layer to the Participant.
  qos.transport().user_transports.push_back(udp_transport);

  // Avoid using the default transport
  qos.transport().use_builtin_transports = false;
  ```

  ### XML

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds>
      <profiles xmlns="http://www.eprosima.com">
          <transport_descriptors>
              <transport_descriptor>
                  <transport_id>tsn_udp_transport</transport_id>
                  <type>UDPv4</type>
                  <udp_priority_mappings>
                      <priority value="-1" source_port="5000"/>
                      <priority value="1" source_port="5000" dscp="1"/>
                      <priority value="2" source_port="5000" dscp="2" interface="eth0"/>
                      <priority value="3" source_port="5000" dscp="3" interface="10.10.1.2"/>
                  </udp_priority_mappings>
              </transport_descriptor>
          </transport_descriptors>

          <participant profile_name="TSNUDPParticipant">
              <rtps>
                  <userTransports>
                      <transport_id>tsn_udp_transport</transport_id>
                  </userTransports>
                  <useBuiltinTransports>false</useBuiltinTransports>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```

  #### NOTE
  On Windows systems, modifying the DSCP value requires additional operating system changes.
  It is not sufficient to configure the QoS profile alone.
  You must also update the Windows registry and group policies to allow applications to set DSCP values:
  - In the regedit application, under `HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\Tcpip\Parameters`,
    create a new DWORD value named DisableUserTOSSetting, and set it to 0.
  - Under `HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\Tcpip\QoS` (create the `QoS` key if it does
    not exist), create a new string value named  `Do not use NLA` and set it to `1`.
  - Reboot the system and open `gpedit.msc`.
  - Create a new policy under *Computer Configuration → Windows Settings → Policy-based QoS*.
    Select *Specify DSCP Value* and enter the desired value.
    On the next page, select Only applications with this executable name, and specify the full path to your
    application’s executable file.
    Then click “Next” until the configuration is complete.

  After completing these steps, your application will be able to use the configured DSCP value.
- **Ethernet Transport Support**

  *Fast DDS Pro* includes a custom Ethernet transport that operates directly at Layer 2 (data link layer),
  bypassing the TCP/IP stack for reduced latency and direct control over ethernet frames.

  *Fast DDS* extends the standard Locator class with a new kind for ethernet communication,
  `LOCATOR_KIND_ETHERNET`.

  In this transport, the tuple parameters (IEEE802MacAddresses & IEEE802VlanTag) are:
  * **Source MAC address**

    Optionally, a virtual sending logical port may be defined to allow multiple DataWriters to share a MAC
    address while still offering a useful way to identify the source of the traffic.
  * **Destination MAC address**
  * **PCP**
  * **VLAN ID**

  Additionally, the Ethernet transport requires setting a default source port for non-prioritized traffic
  (i.e. traffic from DataWriters with [TransportPriorityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#transportpriorityqospolicy) set to 0).
  This traffic will have `PCP=0` and `VLAN ID=0` by default.

  All fields can be set like in the following example:

  ### C++

  ```c++
  DomainParticipantQos qos;

  // Create a descriptor for the new transport.
  auto ethernet_transport = std::make_shared<EthernetTransportDescriptor>();

  // Set the ethernet interface name
  ethernet_transport->interface_name = "lo";

  // Set default source port
  ethernet_transport->default_source_port = 6000;

  // Set the TSN-Ethernet tuples
  EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_1;
  int32_t ethernet_priority_1 = -1;
  ethernet_priority_mapping_1.source_port = 5000;
  ethernet_transport->priority_mapping[ethernet_priority_1] = ethernet_priority_mapping_1;

  EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_2;
  int32_t ethernet_priority_2 = 1;
  ethernet_priority_mapping_2.source_port = 5000;
  ethernet_priority_mapping_2.pcp = 1;
  ethernet_transport->priority_mapping[ethernet_priority_2] = ethernet_priority_mapping_2;

  EthernetTransportDescriptor::PriorityMapping ethernet_priority_mapping_3;
  int32_t ethernet_priority_3 = 2;
  ethernet_priority_mapping_3.source_port = 5000;
  ethernet_priority_mapping_3.pcp = 2;
  ethernet_priority_mapping_3.vlan_id = 100;
  ethernet_transport->priority_mapping[ethernet_priority_3] = ethernet_priority_mapping_3;

  // Link the Transport Layer to the Participant.
  qos.transport().user_transports.push_back(ethernet_transport);

  // Avoid using the default transport
  qos.transport().use_builtin_transports = false;
  ```

  ### XML

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds>
      <profiles xmlns="http://www.eprosima.com">
          <transport_descriptors>
              <transport_descriptor>
                  <transport_id>tsn_ethernet_transport</transport_id>
                  <type>ETH</type>
                  <eth_interface_name>lo</eth_interface_name>
                  <eth_output_port>6000</eth_output_port>
                  <eth_priority_mappings>
                      <priority value="-1" source_port="5000"/>
                      <priority value="1" source_port="5000" pcp="1"/>
                      <priority value="2" source_port="5000" pcp="2" vlan_id="100"/>
                  </eth_priority_mappings>
              </transport_descriptor>
          </transport_descriptors>

          <participant profile_name="TSNEthernetParticipant">
              <rtps>
                  <userTransports>
                      <transport_id>tsn_ethernet_transport</transport_id>
                  </userTransports>
                  <useBuiltinTransports>false</useBuiltinTransports>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```

  #### NOTE
  This transport is only supported on Linux systems.
  It requires either root privileges or the `CAP_NET_RAW` capability.

  #### NOTE
  The *Fast DDS Pro* installation provides a helper file `rtps_eth.lua` under the `share/fastdds`
  subfolder.
  This script allows to easily dissect packets from this transport in Wireshark.
  For details on installing custom plugins, see the
  [Wireshark documentation](https://www.wireshark.org/docs/wsug_html_chunked/ChPluginFolders.html).

<a id="tsn-compatible-qos-policies"></a>

### TSN Compatible QoS Policies

The DDS-TSN specification recommends using the following QoS configuration in the topics that need to leverage
TSN capabilities:

* **Reliability**

  Must use BEST_EFFORT_RELIABILITY_QOS
* **Durability**

  Must use VOLATILE_DURABILITY_QOS
* **History**

  Must use KEEP_LAST_HISTORY_QOS with depth=1

These restrictions are in place to align with the deterministic nature of TSN networks, where re-transmissions
and history caching would potentially disrupt the time-critical delivery of messages.
