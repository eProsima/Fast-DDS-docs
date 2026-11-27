<!-- {{{ X-Types API -->
<!-- }}} -->

#### WARNING
Interface whitelist will be deprecated in future versions of Fast DDS.
The use of the new [interfaces allowlist](interfaces.md#interfaces-allowlist) is recommended.

<a id="whitelist-interfaces"></a>

<a id="interface-whitelist"></a>

# Interface Whitelist

Using *Fast DDS*, it is possible to limit the network interfaces used by [TCP Transport](tcp/tcp.md#transport-tcp-tcp) and
[UDP Transport](udp/udp.md#transport-udp-udp).
This is achieved by adding the interfaces to the [`interfaceWhiteList`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor18interfaceWhiteListE)
field in the [TCPTransportDescriptor](tcp/tcp.md#transport-tcp-transportdescriptor) or [UDPTransportDescriptor](udp/udp.md#transport-udp-transportdescriptor).
Thus, the communication interfaces used by the [DomainParticipants](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) whose [`TransportDescriptorInterface`](../api_reference/transport/transport_generic_interfaces/transport_descriptor_interface.md#_CPPv4N8eprosima7fastdds4rtps28TransportDescriptorInterfaceE) defines an
[`interfaceWhiteList`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor18interfaceWhiteListE) is limited to the interfaces’ addresses defined in that list,
therefore avoiding the use of the rest of the network interfaces available in the system.
The interfaces in [`interfaceWhiteList`](../api_reference/transport/transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptor18interfaceWhiteListE) can be specified both by IP address or interface
name.
For example:

* Interface whitelist filled with IP address:

  ### C++

  ```c++
  DomainParticipantQos qos;

  // Create a descriptor for the new transport.
  auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();

  // Add loopback to the whitelist by IP address
  tcp_transport->interfaceWhiteList.emplace_back("127.0.0.1");

  // Link the Transport Layer to the Participant.
  qos.transport().user_transports.push_back(tcp_transport);

  // Avoid using the builtin transports
  qos.transport().use_builtin_transports = false;
  ```

  ### XML

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <profiles xmlns="http://www.eprosima.com">
      <transport_descriptors>
          <transport_descriptor>
              <transport_id>CustomTcpTransportWhitelistAddress</transport_id>
              <type>TCPv4</type>
              <interfaceWhiteList>
                  <address>127.0.0.1</address>
              </interfaceWhiteList>
          </transport_descriptor>
      </transport_descriptors>

      <participant profile_name="CustomTcpTransportWhitelistAddressParticipant">
          <rtps>
              <useBuiltinTransports>false</useBuiltinTransports>
              <userTransports>
                  <transport_id>CustomTcpTransportWhitelistAddress</transport_id>
              </userTransports>
          </rtps>
      </participant>
  </profiles>
  ```
* Interface whitelist filled with interface names:

  ### C++

  ```c++
  DomainParticipantQos qos;

  // Create a descriptor for the new transport.
  auto tcp_transport = std::make_shared<TCPv4TransportDescriptor>();

  // Add loopback to the whitelist by interface name
  tcp_transport->interfaceWhiteList.emplace_back("lo");

  // Link the Transport Layer to the Participant.
  qos.transport().user_transports.push_back(tcp_transport);

  // Avoid using the builtin transports
  qos.transport().use_builtin_transports = false;
  ```

  ### XML

  ```xml
  <?xml version="1.0" encoding="UTF-8" ?>
  <profiles xmlns="http://www.eprosima.com">
      <transport_descriptors>
          <transport_descriptor>
              <transport_id>CustomTcpTransportWhitelistName</transport_id>
              <type>TCPv4</type>
              <interfaceWhiteList>
                  <interface>lo</interface>
              </interfaceWhiteList>
          </transport_descriptor>
      </transport_descriptors>

      <participant profile_name="CustomTcpTransportWhitelistNameParticipant">
          <rtps>
              <useBuiltinTransports>false</useBuiltinTransports>
              <userTransports>
                  <transport_id>CustomTcpTransportWhitelistName</transport_id>
              </userTransports>
          </rtps>
      </participant>
  </profiles>
  ```

#### IMPORTANT
If none of the values in the transport descriptor’s whitelist match the interfaces on the host,
then all the interfaces in the whitelist are filtered out and therefore no communication will be established through that transport.

#### NOTE
**[Pro]** IPv6 addresses are compared without their zone index (e.g. `fe80::1%eth0` matches `fe80::1` on any interface).
Hence, link-local interfaces should be specified by interface name.

#### WARNING
The interface whitelist feature applies to network interfaces.
Therefore, it is only available on [TCP Transport](tcp/tcp.md#transport-tcp-tcp) and [UDP Transport](udp/udp.md#transport-udp-udp).
