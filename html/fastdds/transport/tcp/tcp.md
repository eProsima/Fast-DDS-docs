<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-tcp-tcp"></a>

<a id="tcp-transport"></a>

# TCP Transport

TCP is a connection oriented transport, so the [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) must establish a TCP connection
to the remote peer before sending data messages.
Therefore, one of the communicating DomainParticipants (the one acting as *server*) must open a TCP port listening for
incoming connections, and the other one (the one acting as *client*) must connect to this port.

#### NOTE
The *server* and *client* concepts are independent from the DDS concepts of
[Publisher](../../dds_layer/publisher/publisher.md#dds-layer-publisher), [Subscriber](../../dds_layer/subscriber/subscriber.md#dds-layer-subscriber),
[DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), and [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
Also, these concepts are independent from the *eProsima Discovery Server* *servers* and *clients*
([Discovery Server](../../discovery/discovery_server.md#discovery-server)).
Any of them can act as a *TCP Server* or *TCP Client* when establishing the connection,
and the DDS communication will work over this connection.

#### WARNING
This documentation assumes the reader has basic knowledge of TCP/IP concepts, since terms like
Time To Live (TTL), Cyclic Redundancy Check (CRC), Transport Layer Security (TLS),
socket buffers, and port numbering are not explained in detail.
However, it is possible to configure a basic TCP transport on *Fast DDS* without this knowledge.

<a id="transport-tcp-transportdescriptor"></a>

<a id="tcptransportdescriptor"></a>

## TCPTransportDescriptor

*eProsima Fast DDS* implements TCP transport for both TCPv4 and TCPv6.
Each of these transports is independent from the other, and has its own [`TransportDescriptorInterface`](../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE).
However, they share many of their features, and most of the [`TransportDescriptorInterface`](../../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) data members are common.

The following table describes the common data members for both TCPv4 and TCPv6.

| Member                                                                                                                                                                                                                 | Data type                                                                                                                                               | Default      | Description                                                                                                                                                                                                                                                                                                                                                    |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|--------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`sendBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14sendBufferSizeE)                                | `uint32_t`                                                                                                                                              | 0            | Size of the sending buffer of the socket (octets).<br/>**[Pro]** If 0, the operating system auto-tunes it (\*).                                                                                                                                                                                                                                                |
| [`receiveBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor17receiveBufferSizeE)                          | `uint32_t`                                                                                                                                              | 0            | Size of the receiving buffer of the socket (octets).<br/>**[Pro]** If 0, the operating system auto-tunes it.                                                                                                                                                                                                                                                   |
| [`netmask_filter`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14netmask_filterE)                                | `NetmaskFilterKind`                                                                                                                                     | `AUTO`       | See [Netmask filtering](../interfaces.md#netmask-filtering).                                                                                                                                                                                                                                                                                                   |
| [`allowlist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE)                                | `vector<pair<string, NetmaskFilterKind>>`                                                                                                               | Empty vector | List of allowed interfaces with<br/>netmask filter configuration.<br/>See [Allowlist](../interfaces.md#interfaces-allowlist).                                                                                                                                                                                                                                  |
| [`blocklist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_blocklistE)                                | `vector<string>`                                                                                                                                        | Empty vector | List of blocked interfaces.<br/>See [Blocklist](../interfaces.md#interfaces-blocklist).                                                                                                                                                                                                                                                                        |
| [`interfaceWhiteList`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor18interfaceWhiteListE)                        | `vector<string>`                                                                                                                                        | Empty vector | List of allowed interfaces<br/>See [Interface Whitelist](../whitelist.md#whitelist-interfaces).                                                                                                                                                                                                                                                                |
| [`TTL`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor3TTLE)                                                       | `uint8_t`                                                                                                                                               | 1            | Time to live, in number of hops.                                                                                                                                                                                                                                                                                                                               |
| [`listening_ports`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor15listening_portsE)                                                   | `vector<uint16_t>`                                                                                                                                      | Empty vector | List of ports to listen as *server*. If a port is set to 0, an available port will be automatically assigned.                                                                                                                                                                                                                                                  |
| [`keep_alive_frequency_ms`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor23keep_alive_frequency_msE)                                   | `uint32_t`                                                                                                                                              | 0            | **[Pro]** Frequency of RTCP keep alive requests (in ms).                                                                                                                                                                                                                                                                                                       |
| [`keep_alive_timeout_ms`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor21keep_alive_timeout_msE)                                       | `uint32_t`                                                                                                                                              | 0            | **[Pro]** Time since sending the last keep alive request to consider a connection as broken (in ms).                                                                                                                                                                                                                                                           |
| [`max_logical_port`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor16max_logical_portE)                                                 | `uint16_t`                                                                                                                                              | 100          | Maximum number of logical ports to try during RTCP negotiation.                                                                                                                                                                                                                                                                                                |
| [`logical_port_range`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor18logical_port_rangeE)                                             | `uint16_t`                                                                                                                                              | 20           | Maximum number of logical ports per request to try during RTCP negotiation.                                                                                                                                                                                                                                                                                    |
| [`logical_port_increment`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor22logical_port_incrementE)                                     | `uint16_t`                                                                                                                                              | 2            | Increment between logical ports to try during RTCP negotiation.                                                                                                                                                                                                                                                                                                |
| [`enable_tcp_nodelay`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor18enable_tcp_nodelayE)                                             | `bool`                                                                                                                                                  | `false`      | Enables the TCP_NODELAY socket option.                                                                                                                                                                                                                                                                                                                         |
| [`non_blocking_send`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17non_blocking_sendE)                                               | `bool`                                                                                                                                                  | `false`      | Do not block on send operations (\*).                                                                                                                                                                                                                                                                                                                          |
| [`calculate_crc`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor13calculate_crcE)                                                       | `bool`                                                                                                                                                  | `true`       | True to calculate and send CRC on message headers.                                                                                                                                                                                                                                                                                                             |
| [`check_crc`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9check_crcE)                                                                | `bool`                                                                                                                                                  | `true`       | True to check the CRC of incoming message headers.                                                                                                                                                                                                                                                                                                             |
| [`apply_security`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor14apply_securityE)                                                     | `bool`                                                                                                                                                  | `false`      | True to use TLS. See [TLS over TCP](tls.md#transport-tcp-tls).                                                                                                                                                                                                                                                                                                 |
| [`tls_config`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor10tls_configE)                                                             | [`TLSConfig`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfigE) |              | Configuration for TLS. See [TLS over TCP](tls.md#transport-tcp-tls).                                                                                                                                                                                                                                                                                           |
| [`default_reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor25default_reception_threadsEv) | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)                                                                   |              | Default [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads.                                                                                                                                                                                                                                       |
| [`reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor17reception_threadsEv)                 | `std::map<uint32_t, ThreadSettings>`                                                                                                                    |              | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads on specific ports.                                                                                                                                                                                                                             |
| [`keep_alive_thread`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17keep_alive_threadE)                                               | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)                                                                   |              | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the thread keeping alive TCP connections.                                                                                                                                                                                                                            |
| [`accept_thread`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor13accept_threadE)                                                       | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos)                                                                   |              | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the threads processing incoming TCP connection requests.                                                                                                                                                                                                             |
| [`tcp_negotiation_timeout`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor23tcp_negotiation_timeoutE)                                   | `uint32_t`                                                                                                                                              | 0            | Time to wait for logical port negotiation (in ms). If a logical port is under negotiation, it waits for the<br/>negotiation to finish up to this timeout before trying to send a message to that port. Setting this option to<br/>non-zero values increases the discovery time. Setting it to zero means no wait but could lead to loss of first<br/>messages. |

#### WARNING
Although the member [`listening_ports`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor15listening_portsE) accepts multiple ports, only the first listening port
will be effectively used. The rest of the ports will be ignored.

#### NOTE
If [`listening_ports`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor15listening_portsE) is left empty, the participant will not be able to receive incoming
connections but will be able to connect to other participants that have configured their listening ports.

#### NOTE
When [`non_blocking_send`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17non_blocking_sendE) is set to `true`, send operations will return immediately if the
send buffer might get full, but no error will be returned to the upper layer.
This means that the application will behave as if the packet is sent and lost.

When set to `false`, send operations will block until the network buffer has space for the
packet.

**[Pro]** As this check requires a fixed send buffer size, enabling [`non_blocking_send`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17non_blocking_sendE) with
[`sendBufferSize`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor14sendBufferSizeE) set to 0 disables the send buffer auto-tuning, and the system
default size is used instead.

<a id="transport-tcp-v4transportdescriptor"></a>

<a id="tcpv4transportdescriptor"></a>

### TCPv4TransportDescriptor

The following table describes the data members that are exclusive for [`TCPv4TransportDescriptor`](../../api_reference/transport/tcp_transport/tcpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptorE).

| Member                                                                                                                                                    | Data type   | Default      | Description                                                                                |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|--------------|--------------------------------------------------------------------------------------------|
| [`wan_addr`](../../api_reference/transport/tcp_transport/tcpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptor8wan_addrE) | `octet[4]`  | [0, 0, 0, 0] | Configuration for WAN. See [WAN or Internet Communication over TCPv4](#transport-tcp-wan). |

#### NOTE
The [`kind`](../../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv) value for a [`TCPv4TransportDescriptor`](../../api_reference/transport/tcp_transport/tcpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptorE) is given by the value
[`LOCATOR_KIND_TCPv4`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_TCPv4).

<a id="transport-tcp-v6transportdescriptor"></a>

<a id="tcpv6transportdescriptor"></a>

### TCPv6TransportDescriptor

[`TCPv6TransportDescriptor`](../../api_reference/transport/tcp_transport/tcpv6_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptorE) has no additional data members from the common ones described in
[TCPTransportDescriptor](#transport-tcp-transportdescriptor).

#### NOTE
The [`kind`](../../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv) value for a [`TCPv6TransportDescriptor`](../../api_reference/transport/tcp_transport/tcpv6_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptorE) is given by the value
[`LOCATOR_KIND_TCPv6`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_TCPv6).

<a id="transport-tcp-enabling"></a>

<a id="enabling-tcp-transport"></a>

## Enabling TCP Transport

There are several ways of enabling TCP transport in *eprosima Fast DDS*. According to the facet of each
scenario, one method might suit better than the others.

<a id="configuration-of-builtin-transports"></a>

### Configuration of Builtin Transports

The first option is to modify the builtin transports that are responsible of the creation of the DomainParticipant
transports. The existing configuration that enables TCP transports is `LARGE_DATA`.
This option instantiates a UDPv4, a TCPv4 and a SHM transport, respectively. UDP protocol will be used for multicast
announcements during the participant discovery phase (see [Discovery phases](../../discovery/discovery.md#disc-phases)) while the participant liveliness and
the application data delivery occurs over TCP or SHM. This configuration enables auto discovery and does not
require to manually set up each participant IP and listening port. Hence, avoiding the typical Server-Client
configuration.

Builtin Transports can be configured using the `FASTDDS_BUILTIN_TRANSPORTS` environment variable (see
[FASTDDS_BUILTIN_TRANSPORTS](../../env_vars/env_vars.md#env-vars-builtin-transports)), XML profiles (see [RTPS element type](../../xml_configuration/domainparticipant.md#rtps)) or via code.

### Environment Variable

```bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <!--
            UDP transport for PDP and SHM/TCPv4 transport for both EDP and application data
        -->
        <participant profile_name="large_data_builtin_transports" is_default_profile="true">
            <rtps>
                <builtinTransports>LARGE_DATA</builtinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### C++

```c++
eprosima::fastdds::dds::DomainParticipantQos qos;
qos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);
```

#### NOTE
Note that `LARGE_DATA` configuration of the builtin transports will also create a SHM transport along the UDP
and TCP transports. Shared Memory will be used whenever it is possible. Manual configuration will be required
if a TCP communication is required when SHM is feasible. (See [Large Data Mode](../../use_cases/tcp/tcp_with_multicast_discovery.md#use-case-tcp-multicast)).

<a id="transport-tcp-server-client"></a>

<a id="server-client-configuration"></a>

### Server-Client Configuration

To set up a Server-Client configuration you need to
create an instance of [TCPv4TransportDescriptor](#transport-tcp-v4transportdescriptor) (for TCPv4) or
[TCPv6TransportDescriptor](#transport-tcp-v6transportdescriptor) (for TCPv6), and add it to the user transport list of the
DomainParticipant.

Depending on the TCP transport descriptor settings and network locators defined, the DomainParticipant can act as a
TCP Server or TCP Client.

* **TCP Server**: If you provide [`listening_ports`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor15listening_portsE) on the descriptor, the DomainParticipant
  will act as *TCP server*, listening for incoming remote connections on the given ports.
  The examples below show this procedure in both C++ code and XML file, for TCPv4 and TCPv6.

  ### C++ TCPv4

  ```c++
  eprosima::fastdds::dds::DomainParticipantQos qos;

  // Create a descriptor for the new transport.
  auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
  tcp_transport->add_listener_port(5100);

  // [OPTIONAL] ThreadSettings configuration
  tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
  tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
  tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
  tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

  // Link the Transport Layer to the Participant.
  qos.transport().user_transports.push_back(tcp_transport);

  // Avoid using the default transport
  qos.transport().use_builtin_transports = false;

  // [OPTIONAL] Set unicast locators
  eprosima::fastdds::rtps::Locator_t locator;
  locator.kind = LOCATOR_KIND_TCPv4;
  eprosima::fastdds::rtps::IPLocator::setIPv4(locator, "192.168.1.10");
  eprosima::fastdds::rtps::IPLocator::setPhysicalPort(locator, 5100);
  // [OPTIONAL] Logical port default value is 0, automatically assigned.
  eprosima::fastdds::rtps::IPLocator::setLogicalPort(locator, 5100);

  qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
  qos.wire_protocol().default_unicast_locator_list.push_back(locator);
  ```

  ### XML TCPv4

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds>
      <profiles xmlns="http://www.eprosima.com">
          <transport_descriptors>
              <transport_descriptor>
                  <transport_id>tcp_server_transport</transport_id>
                  <type>TCPv4</type>
                  <listening_ports>
                      <port>5100</port>
                  </listening_ports>
                  <!-- Optional thread configuration -->
                  <default_reception_threads>
                      <scheduling_policy>-1</scheduling_policy>
                      <priority>0</priority>
                      <affinity>0</affinity>
                      <stack_size>-1</stack_size>
                  </default_reception_threads>
                  <reception_threads>
                      <reception_thread port="12345">
                          <scheduling_policy>-1</scheduling_policy>
                          <priority>0</priority>
                          <affinity>0</affinity>
                          <stack_size>-1</stack_size>
                      </reception_thread>
                  </reception_threads>
                  <keep_alive_thread>
                      <scheduling_policy>-1</scheduling_policy>
                      <priority>0</priority>
                      <affinity>0</affinity>
                      <stack_size>-1</stack_size>
                  </keep_alive_thread>
                  <accept_thread>
                      <scheduling_policy>-1</scheduling_policy>
                      <priority>0</priority>
                      <affinity>0</affinity>
                      <stack_size>-1</stack_size>
                  </accept_thread>
              </transport_descriptor>
          </transport_descriptors>

          <participant profile_name="tcp_server_participant">
              <rtps>
                  <userTransports>
                      <transport_id>tcp_server_transport</transport_id>
                  </userTransports>
                  <useBuiltinTransports>false</useBuiltinTransports>
                  <!-- Optional unicast locator set -->
                  <defaultUnicastLocatorList>
                      <locator>
                          <tcpv4>
                              <address>192.168.1.10</address>
                              <physical_port>5100</physical_port>
                              <!-- Optional logical port set -->
                              <port>5100</port>
                          </tcpv4>
                      </locator>
                  </defaultUnicastLocatorList>
                  <!-- Optional metatraffic unicast locator set -->
                  <builtin>
                      <metatrafficUnicastLocatorList>
                          <locator>
                              <tcpv4>
                                  <address>192.168.1.10</address>
                                  <physical_port>5100</physical_port>
                                  <!-- Optional logical port set -->
                                  <port>5100</port>
                              </tcpv4>
                          </locator>
                      </metatrafficUnicastLocatorList>
                  </builtin>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```

  ### C++ TCPv6

  ```c++
  eprosima::fastdds::dds::DomainParticipantQos qos;

  // Create a descriptor for the new transport.
  auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv6TransportDescriptor>();
  tcp_transport->add_listener_port(5100);

  // [RECOMMENDED] Set the interfaces to be used to avoid ambiguity
  tcp_transport->interface_allowlist.emplace_back("eth0");

  // Link the Transport Layer to the Participant.
  qos.transport().user_transports.push_back(tcp_transport);

  // Avoid using the default transport
  qos.transport().use_builtin_transports = false;

  // [RECOMMENDED] Set unicast locators with the address to be announced
  eprosima::fastdds::rtps::Locator_t locator;
  locator.kind = LOCATOR_KIND_TCPv6;
  eprosima::fastdds::rtps::IPLocator::setIPv6(locator, "2001:db8::10");
  eprosima::fastdds::rtps::IPLocator::setPhysicalPort(locator, 5100);
  // [OPTIONAL] Logical port default value is 0, automatically assigned.
  eprosima::fastdds::rtps::IPLocator::setLogicalPort(locator, 5100);

  qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
  qos.wire_protocol().default_unicast_locator_list.push_back(locator);
  ```

  ### XML TCPv6

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds>
      <profiles xmlns="http://www.eprosima.com">
          <transport_descriptors>
              <transport_descriptor>
                  <transport_id>tcpv6_server_transport</transport_id>
                  <type>TCPv6</type>
                  <listening_ports>
                      <port>5100</port>
                  </listening_ports>
                  <!-- Recommended interfaces set to avoid ambiguity -->
                  <interfaces>
                      <allowlist>
                          <interface name="eth0"/>
                      </allowlist>
                  </interfaces>
              </transport_descriptor>
          </transport_descriptors>

          <participant profile_name="tcpv6_server_participant">
              <rtps>
                  <userTransports>
                      <transport_id>tcpv6_server_transport</transport_id>
                  </userTransports>
                  <useBuiltinTransports>false</useBuiltinTransports>
                  <!-- Recommended unicast locator set with the address to be announced -->
                  <defaultUnicastLocatorList>
                      <locator>
                          <tcpv6>
                              <address>2001:db8::10</address>
                              <physical_port>5100</physical_port>
                              <!-- Optional logical port set -->
                              <port>5100</port>
                          </tcpv6>
                      </locator>
                  </defaultUnicastLocatorList>
                  <!-- Recommended metatraffic unicast locator set with the address to be announced -->
                  <builtin>
                      <metatrafficUnicastLocatorList>
                          <locator>
                              <tcpv6>
                                  <address>2001:db8::10</address>
                                  <physical_port>5100</physical_port>
                                  <!-- Optional logical port set -->
                                  <port>5100</port>
                              </tcpv6>
                          </locator>
                      </metatrafficUnicastLocatorList>
                  </builtin>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```
* **TCP Client**: If you provide [`initialPeersList`](../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes16initialPeersListE) to the DomainParticipant, it will act as
  *TCP client*, trying to connect to the remote *servers* at the given addresses and ports.
  The examples below show this procedure in both C++ code and XML file, for TCPv4 and TCPv6.
  See [Initial peers](../../discovery/simple.md#simple-initial-peers) for more information about their configuration.

  ### C++ TCPv4

  ```c++
  eprosima::fastdds::dds::DomainParticipantQos qos;

  // Disable the built-in Transport Layer.
  qos.transport().use_builtin_transports = false;

  // Create a descriptor for the new transport.
  // Do not configure any listener port
  auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
  qos.transport().user_transports.push_back(tcp_transport);

  // [OPTIONAL] ThreadSettings configuration
  tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
  tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
  tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
  tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

  // Set initial peers.
  eprosima::fastdds::rtps::Locator_t initial_peer_locator;
  initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
  eprosima::fastdds::rtps::IPLocator::setIPv4(initial_peer_locator, "192.168.1.10");
  eprosima::fastdds::rtps::IPLocator::setPhysicalPort(initial_peer_locator, 5100);
  // If the logical port is set in the server side, it must be also set here with the same value.
  // If not set in the server side in a unicast locator, do not set it here.
  eprosima::fastdds::rtps::IPLocator::setLogicalPort(initial_peer_locator, 5100);

  qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);
  ```

  ### XML TCPv4

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds>
      <profiles xmlns="http://www.eprosima.com">
          <transport_descriptors>
              <transport_descriptor>
                  <transport_id>tcp_client_transport</transport_id>
                  <type>TCPv4</type>
                  <!-- Optional thread configuration -->
                  <default_reception_threads>
                      <scheduling_policy>-1</scheduling_policy>
                      <priority>0</priority>
                      <affinity>0</affinity>
                      <stack_size>-1</stack_size>
                  </default_reception_threads>
                  <reception_threads>
                      <reception_thread port="12345">
                          <scheduling_policy>-1</scheduling_policy>
                          <priority>0</priority>
                          <affinity>0</affinity>
                          <stack_size>-1</stack_size>
                      </reception_thread>
                  </reception_threads>
                  <keep_alive_thread>
                      <scheduling_policy>-1</scheduling_policy>
                      <priority>0</priority>
                      <affinity>0</affinity>
                      <stack_size>-1</stack_size>
                  </keep_alive_thread>
                  <accept_thread>
                      <scheduling_policy>-1</scheduling_policy>
                      <priority>0</priority>
                      <affinity>0</affinity>
                      <stack_size>-1</stack_size>
                  </accept_thread>
              </transport_descriptor>
          </transport_descriptors>

          <participant profile_name="tcp_client_participant">
              <rtps>
                  <userTransports>
                      <transport_id>tcp_client_transport</transport_id>
                  </userTransports>
                  <useBuiltinTransports>false</useBuiltinTransports>
                  <builtin>
                      <initialPeersList>
                          <locator>
                              <tcpv4>
                                  <address>192.168.1.10</address>
                                  <physical_port>5100</physical_port>
                                  <!-- To be set if set in server -->
                                  <port>5100</port>
                              </tcpv4>
                          </locator>
                      </initialPeersList>
                  </builtin>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```

  ### C++ TCPv6

  ```c++
  eprosima::fastdds::dds::DomainParticipantQos qos;

  // Disable the built-in Transport Layer.
  qos.transport().use_builtin_transports = false;

  // Create a descriptor for the new transport.
  // Do not configure any listener port
  auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv6TransportDescriptor>();
  // [RECOMMENDED] Set the interfaces to be used to avoid ambiguity
  tcp_transport->interface_allowlist.emplace_back("eth0");
  qos.transport().user_transports.push_back(tcp_transport);

  // [RECOMMENDED] Set unicast locators with the address to be announced.
  // Do not set the physical port, as the client has no listening port.
  eprosima::fastdds::rtps::Locator_t locator;
  locator.kind = LOCATOR_KIND_TCPv6;
  eprosima::fastdds::rtps::IPLocator::setIPv6(locator, "2001:db8::20");

  qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
  qos.wire_protocol().default_unicast_locator_list.push_back(locator);

  // Set initial peers. The address must be one of the addresses announced by the server.
  eprosima::fastdds::rtps::Locator_t initial_peer_locator;
  initial_peer_locator.kind = LOCATOR_KIND_TCPv6;
  eprosima::fastdds::rtps::IPLocator::setIPv6(initial_peer_locator, "2001:db8::10");
  eprosima::fastdds::rtps::IPLocator::setPhysicalPort(initial_peer_locator, 5100);
  // If the logical port is set in the server side, it must be also set here with the same value.
  // If not set in the server side in a unicast locator, do not set it here.
  eprosima::fastdds::rtps::IPLocator::setLogicalPort(initial_peer_locator, 5100);

  qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);
  ```

  ### XML TCPv6

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <dds>
      <profiles xmlns="http://www.eprosima.com">
          <transport_descriptors>
              <transport_descriptor>
                  <transport_id>tcpv6_client_transport</transport_id>
                  <type>TCPv6</type>
                  <!-- Recommended interfaces set to avoid ambiguity -->
                  <interfaces>
                      <allowlist>
                          <interface name="eth0"/>
                      </allowlist>
                  </interfaces>
              </transport_descriptor>
          </transport_descriptors>

          <participant profile_name="tcpv6_client_participant">
              <rtps>
                  <userTransports>
                      <transport_id>tcpv6_client_transport</transport_id>
                  </userTransports>
                  <useBuiltinTransports>false</useBuiltinTransports>
                  <!-- Recommended unicast locator set with the address to be announced -->
                  <!-- No physical port set, as the client has no listening port -->
                  <defaultUnicastLocatorList>
                      <locator>
                          <tcpv6>
                              <address>2001:db8::20</address>
                          </tcpv6>
                      </locator>
                  </defaultUnicastLocatorList>
                  <builtin>
                      <!-- Recommended metatraffic unicast locator set with the address to be announced -->
                      <metatrafficUnicastLocatorList>
                          <locator>
                              <tcpv6>
                                  <address>2001:db8::20</address>
                              </tcpv6>
                          </locator>
                      </metatrafficUnicastLocatorList>
                      <!-- The address must be one of the addresses announced by the server -->
                      <initialPeersList>
                          <locator>
                              <tcpv6>
                                  <address>2001:db8::10</address>
                                  <physical_port>5100</physical_port>
                                  <!-- To be set if set in server -->
                                  <port>5100</port>
                              </tcpv6>
                          </locator>
                      </initialPeersList>
                  </builtin>
              </rtps>
          </participant>
      </profiles>
  </dds>
  ```

#### NOTE
Manually setting unicast locators is optional. If not setting them or setting them with a logical
port of `0`, the client’s initial peer shouldn’t set its logical port (or set it to `0`). Otherwise,
initial peer’s logical port must match server’s unicast logical port.

For TCPv6, see [TCPv6 Addresses and Interfaces](#transport-tcp-ipv6-addresses) for the considerations on the addresses and interfaces used.

[Delivery Mechanisms example](#transport-tcp-example) shows how to use and configure a TCP transport.

<a id="transport-tcp-ipv6-addresses"></a>

<a id="tcpv6-addresses-and-interfaces"></a>

## TCPv6 Addresses and Interfaces

A TCPv6 locator holds a 16 bytes IPv6 address, which can be a global unicast, a unique local, or a link-local address.
Global unicast (GUA) and unique local (ULA) addresses are reached through the routing table, so connecting to them
needs no additional configuration.

#### NOTE
Communication through IPv6 link-local interfaces is only supported in Fast DDS **[Pro]**.

**[Pro]** Link-local addresses (`fe80::/10`) are only valid within a network link, so the same address may be
reachable through several interfaces of the host.
Connecting to them requires selecting the local interface the connection goes through, which is given by the zone
index of the address (e.g. `fe80::1%eth0`).
The zone index only has meaning on the local host, so it is never part of a locator nor announced to remote
participants.
Instead, the TCPv6 transport selects it when connecting to a link-local locator:

* If the address belongs to the host, the interface owning it is used.
* Otherwise, the only link-local interface allowed by the transport via the allowlist field is used
  (see [Allowlist](../interfaces.md#interfaces-allowlist)).
* If several or none allowed link-local interfaces are available (multiple entries in the allowlist), the interface
  cannot be determined, an error is logged and no connection is attempted to that locator.

By default, the DomainParticipant announces a locator for each address of the allowed interfaces, link-local addresses
included (see [Default Announced Locators](../announced_locators.md#default-announced-locators)).
Allowing an interface by name allows all its addresses, so all of them are announced.
It is recommended to always configure the interfaces to be used in the
[`allowlist`](../../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor19interface_allowlistE), in order to avoid ambiguity, and to set the address to be
announced in the unicast locator lists of both the *TCP server* and the *TCP client*, as shown in the TCPv6 examples
of [Server-Client Configuration](#transport-tcp-server-client).
The *TCP client* must not set the physical port of its unicast locators, as it has no listening port.
The initial peers of a *TCP client* must use one of the addresses announced by the *TCP server*.

#### NOTE
**[Pro]** Allowlist and blocklist entries given by IP address are compared without the zone index.
Hence, link-local interfaces should be specified by interface name.

<a id="transport-tcp-wan"></a>

<a id="wan-or-internet-communication-over-tcpv4"></a>

## WAN or Internet Communication over TCPv4

*Fast DDS* is able to connect through the Internet or other WAN networks when configured properly.
To achieve this kind of scenarios, the involved network devices such as routers and firewalls
must add the rules to allow the communication.

For example, imagine we have the scenario represented on the following figure:

![image](01-figures/TCP_WAN.png)
* A DomainParticipant acts as a *TCP server* listening on port `5100`
  and is connected to the WAN through a router with public IP `80.80.99.45`.
* Another DomainParticipant acts as a *TCP client* and has configured
  the server’s IP address and port in its [Initial peers](../../discovery/simple.md#simple-initial-peers) list.

By using `set_WAN_address(wan_ip)`, the WAN IP address is set on the participant’s locators that
are communicated during the discovery phase.

Like in the LAN case, manually setting unicast locators is optional. However, in this case, there are
some considerations to take into account when setting its IP addresses:

* Setting the WAN IP address using the `setWAN()` method in unicast locators is ineffective because it
  gets overridden by the `set_WAN_address()` call.
* For assigning IP addresses to unicast locators, use only the `setIPv4()` or `setIPv6()` methods, which
  are LAN IP setters. There are some configurations which allow using these setters with a WAN IP address.

Depending on whether the server has manually set its metatraffic unicast locators and default unicast
locators, the client needs to adjust its initial peer list accordingly:

* If the server’s unicast locators are configured with the LAN IP address:
  * The initial peer can be set up with only the server’s WAN IP using the LAN IP setter.
  * Alternatively, it can be configured with both the server’s LAN and WAN IP addresses using the LAN
    setter for the LAN IP and the WAN setter for the WAN IP.
* If the server’s unicast locators are configured with the WAN IP address:
  * The initial peer must be set up with only the server’s WAN IP using the LAN setter.
  * Alternatively, it can be configured with the WAN IP address using both the LAN and WAN setters.
* If the server has not set any unicast locators:
  * The initial peer can be configured with only the server’s WAN IP using the LAN setter.
  * Alternatively, it can be configured with both the server’s LAN and WAN IP addresses using the LAN
    setter for the LAN IP and the WAN setter for the WAN IP.

#### NOTE
Manually setting unicast locators is optional. If not setting them or setting them with a logical
port of `0`, the client’s initial peer shouldn’t set its logical port (or set it to `0`). Otherwise,
initial peer’s logical port must match server’s unicast logical port.

On the server side, the router must be configured to forward to the *TCP server*
all traffic incoming to port `5100`. Typically, a NAT routing of port `5100` to our
machine is enough. Any existing firewall should be configured as well.

In addition, to allow incoming connections through a WAN,
the [TCPv4TransportDescriptor](#transport-tcp-v4transportdescriptor) must indicate its **public** IP address
in the [`wan_addr`](../../api_reference/transport/tcp_transport/tcpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptor8wan_addrE) data member.
The following examples show how to configure the DomainParticipant both in C++ and XML.

### C++

```c++
eprosima::fastdds::dds::DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
tcp_transport->add_listener_port(5100);
tcp_transport->set_WAN_address("80.80.99.45");

// [OPTIONAL] ThreadSettings configuration
tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(tcp_transport);

// Avoid using the default transport
qos.transport().use_builtin_transports = false;

// [OPTIONAL] Set unicast locators (do not use setWAN(), set_WAN_address() overwrites it)
eprosima::fastdds::rtps::Locator_t locator;
locator.kind = LOCATOR_KIND_TCPv4;
// [RECOMMENDED] Use the LAN address of the server
eprosima::fastdds::rtps::IPLocator::setIPv4(locator, "192.168.1.10");
// [ALTERNATIVE] Use server's WAN address. In that case, initial peers must be configured
// only with server's WAN address.
// eprosima::fastdds::rtps::IPLocator::setIPv4(locator, "80.80.99.45");
eprosima::fastdds::rtps::IPLocator::setPhysicalPort(locator, 5100);
// [OPTIONAL] Logical port default value is 0, automatically assigned.
eprosima::fastdds::rtps::IPLocator::setLogicalPort(locator, 5100);

qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(locator);
qos.wire_protocol().default_unicast_locator_list.push_back(locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>tcp_server_wan_transport</transport_id>
                <type>TCPv4</type>
                <listening_ports>
                    <port>5100</port>
                </listening_ports>
                <wan_addr>80.80.99.45</wan_addr>
                <!-- Optional thread configuration -->
                <default_reception_threads>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </default_reception_threads>
                <reception_threads>
                    <reception_thread port="12345">
                        <scheduling_policy>-1</scheduling_policy>
                        <priority>0</priority>
                        <affinity>0</affinity>
                        <stack_size>-1</stack_size>
                    </reception_thread>
                </reception_threads>
                <keep_alive_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </keep_alive_thread>
                <accept_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </accept_thread>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="tcp_server_wan_participant">
            <rtps>
                <userTransports>
                    <transport_id>tcp_server_wan_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
                <!-- Optional unicast locator set -->
                <defaultUnicastLocatorList>
                    <locator>
                        <tcpv4>
                            <address>192.168.1.10</address>
                            <!-- Alternatively use WAN address -->
                            <!-- <address>80.80.99.45</address> -->
                            <physical_port>5100</physical_port>
                            <!-- Optional logical port set -->
                            <port>5100</port>
                        </tcpv4>
                    </locator>
                </defaultUnicastLocatorList>
                <!-- Optional metatraffic unicast locator set -->
                <builtin>
                    <metatrafficUnicastLocatorList>
                        <locator>
                            <tcpv4>
                                <address>192.168.1.10</address>
                                <!-- Alternatively use WAN address -->
                                <!-- <address>80.80.99.45</address> -->
                                <physical_port>5100</physical_port>
                                <!-- Optional logical port set -->
                                <port>5100</port>
                            </tcpv4>
                        </locator>
                    </metatrafficUnicastLocatorList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

On the client side, the DomainParticipant must be configured
with the **public** IP address and [`listening_ports`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor15listening_portsE) of the *TCP server* as
[Initial peers](../../discovery/simple.md#simple-initial-peers).

### C++

```c++
eprosima::fastdds::dds::DomainParticipantQos qos;

// Disable the built-in Transport Layer.
qos.transport().use_builtin_transports = false;

// Create a descriptor for the new transport.
// Do not configure any listener port
auto tcp_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
// [RECOMMENDED] Use client's WAN address if there are more clients in other local networks.
tcp_transport->set_WAN_address("80.80.99.47");
qos.transport().user_transports.push_back(tcp_transport);

// [OPTIONAL] ThreadSettings configuration
tcp_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
tcp_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
tcp_transport->keep_alive_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};
tcp_transport->accept_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

// Set initial peers.
eprosima::fastdds::rtps::Locator_t initial_peer_locator;
initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
// [RECOMMENDED] Use both WAN and LAN server addresses
eprosima::fastdds::rtps::IPLocator::setIPv4(initial_peer_locator, "192.168.1.10");
eprosima::fastdds::rtps::IPLocator::setWan(initial_peer_locator, "80.80.99.45");
// [ALTERNATIVE] Use server's WAN address only. Valid if server specified its unicast locators
// with its LAN or WAN address.
// eprosima::fastdds::rtps::IPLocator::setIPv4(initial_peer_locator, "80.80.99.45");
eprosima::fastdds::rtps::IPLocator::setPhysicalPort(initial_peer_locator, 5100);
// If the logical port is set in the server side, it must be also set here with the same value.
// If not set in the server side in a unicast locator, do not set it here.
eprosima::fastdds::rtps::IPLocator::setLogicalPort(initial_peer_locator, 5100);

qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>tcp_client_wan_transport</transport_id>
                <type>TCPv4</type>
                <!-- Recommended client's WAN set -->
                <wan_addr>80.80.99.47</wan_addr>
                <!-- Optional thread configuration -->
                <default_reception_threads>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </default_reception_threads>
                <reception_threads>
                    <reception_thread port="12345">
                        <scheduling_policy>-1</scheduling_policy>
                        <priority>0</priority>
                        <affinity>0</affinity>
                        <stack_size>-1</stack_size>
                    </reception_thread>
                </reception_threads>
                <keep_alive_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </keep_alive_thread>
                <accept_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </accept_thread>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="tcp_client_wan_participant">
            <rtps>
                <userTransports>
                    <transport_id>tcp_client_wan_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
                <builtin>
                    <initialPeersList>
                        <locator>
                            <tcpv4>
                                <!-- Recommended use of both WAN and LAN server addresses -->
                                <wan_address>80.80.99.45</wan_address>
                                <address>192.168.1.10</address>
                                <!-- Alternatively use server's WAN addresses only -->
                                <!-- <address>80.80.99.45</address> -->
                                <physical_port>5100</physical_port>
                                <!-- To be set if set in server -->
                                <port>5100</port>
                            </tcpv4>
                        </locator>
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="transport-tcp-wan-v6"></a>

<a id="wan-or-internet-communication-over-tcpv6"></a>

## WAN or Internet Communication over TCPv6

IPv6 networks do not usually rely on NAT, so a host connected to the Internet owns a global unicast address that remote
peers can reach directly.
Therefore, the WAN configuration of TCPv4 ([`wan_addr`](../../api_reference/transport/tcp_transport/tcpv4_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv4TransportDescriptor8wan_addrE) and the WAN address of the
locators) do not apply to TCPv6.

To communicate through a WAN over TCPv6:

* On the *TCP server*, set its global unicast address in both the metatraffic and default unicast locator lists, with
  the listening port as physical port, so that it is the address announced to remote participants.
* On the *TCP client*, set the global unicast address announced by the *TCP server* as [Initial peers](../../discovery/simple.md#simple-initial-peers).
  It is recommended to set the interface to be used in the allowlist field of the [`TCPv6TransportDescriptor`](../../api_reference/transport/tcp_transport/tcpv6_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps24TCPv6TransportDescriptorE) and to
  also configure the unicast locators of the *TCP client* with the desired address to be announced.
* Configure the firewalls in the path to allow incoming connections to the listening port of the *TCP server*.

The TCPv6 examples of [Server-Client Configuration](#transport-tcp-server-client) show this configuration.

<a id="transport-tcp-example"></a>

<a id="delivery-mechanisms-example"></a>

## Delivery Mechanisms example

A hello world example suitable for supported delivery mechanisms can be found in the
[delivery_mechanisms folder](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/delivery_mechanisms).
It shows a publisher and a subscriber that communicate through the desired delivery mechanism (which can be set to TCP
only).
