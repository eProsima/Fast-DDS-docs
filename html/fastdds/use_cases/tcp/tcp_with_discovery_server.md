<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-tcp-discovery-server"></a>

<a id="tcp-communication-with-discovery-server"></a>

# TCP Communication with Discovery Server

*Fast DDS* [Discovery Server](../wifi/discovery_server_use_case.md#discovery-server-use-case) consists on a client-server discovery mechanism, in which a server
[DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) operates as the central point of communication. It collects and processes the metatraffic
sent by the client DomainParticipants, and then distributes it among the rest of
the clients. An extended description of the feature can be found at [Discovery Server](../../discovery/discovery_server.md#discovery-server).

To use TCP communication along with Discovery Server, both the server participant and the client participant
need to use custom user transports. There exists several ways of configuring the server participant, being
*Fast DDS* [CLI](../../../fastddscli/cli/cli.md#ffastddscli-cli) the fastest solution:

### Fast DDS CLI

It can be configured to work over a TCP transport layer by using the arguments `-t` and `-q` to set
up the IP address and the TCP port, respectively. After sourcing the environment, the following command
can be used to instantiate a server listening on localhost and port 12345 (see [CLI](../../../fastddscli/cli/cli.md#ffastddscli-cli)).

```bash
fastdds discovery -t 127.0.0.1 -q 12345
```

### C++

The following snippet can be used to instantiate a server on IP 192.168.10.57 listening on port 12345.

```c++
eprosima::fastdds::dds::DomainParticipantQos qos = PARTICIPANT_QOS_DEFAULT;

// Configure the current participant as SERVER
qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        eprosima::fastdds::rtps::DiscoveryProtocol::SERVER;

// Add custom user transport with TCP port 12345
auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
data_transport->add_listener_port(12345);
qos.transport().user_transports.push_back(data_transport);

// Define the listening locator to be on interface 192.168.10.57 and port 12345
constexpr uint16_t tcp_listening_port = 12345;
eprosima::fastdds::rtps::Locator_t listening_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(listening_locator, "192.168.10.57");
eprosima::fastdds::rtps::IPLocator::setPhysicalPort(listening_locator, tcp_listening_port);
eprosima::fastdds::rtps::IPLocator::setLogicalPort(listening_locator, tcp_listening_port);
qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(listening_locator);
```

### XML

The following snippet can be used to instantiate a server on IP 192.168.10.57 listening on port 12345.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>server_tcp_transport</transport_id>
                <type>TCPv4</type>
                <!--
                    Set listening port for the transport.
                    This port is where the clients will connect.
                -->
                <listening_ports>
                    <port>12345</port>
                </listening_ports>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="TCP_SERVER" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SERVER</discoveryProtocol>
                    </discovery_config>
                    <metatrafficUnicastLocatorList>
                        <locator>
                            <tcpv4>
                                <address>192.168.10.57</address>
                                <port>12345</port>
                                <physical_port>12345</physical_port>
                            </tcpv4>
                        </locator>
                    </metatrafficUnicastLocatorList>
                </builtin>
                <useBuiltinTransports>false</useBuiltinTransports>
                <userTransports>
                    <transport_id>server_tcp_transport</transport_id>
                </userTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

### Fast DDS Discovery Server Example

It can be configured to work over a TCP transport layer by using the argument `--transport tcpv4`. The IP
address and the TCP port can be set up with arguments `--listening-address` and `--listening-port`,
respectively. From the *DiscoveryServerExample* folder, the following command can be used to instantiate a
server listening on localhost and port 12345.

```bash
./DiscoveryServerExample server --transport tcpv4 --listening-address 127.0.0.1 --listening-port 12345
```

The client participant can be configured by either using the `ROS_DISCOVERY_SERVER` environment variable (see
[ROS_DISCOVERY_SERVER](../../env_vars/env_vars.md#env-vars-ros-discovery-server)) or by manually setting it.

### Environment Variable

To configure a client participant to communicate over the TCP transport layer with the
`ROS_DISCOVERY_SERVER` environment variable, the prefix TCPv4 needs to be used. The following command
can be used to configure the variable to set up a client using TCP communication and connecting to a
server on localhost and port 12345.

```bash
export ROS_DISCOVERY_SERVER=TCPv4:[127.0.0.1]:12345
```

### C++

The following snippet can be used to instantiate a client that will try to connect to a server on IP
192.168.10.57 and port 12345, that is, the server instantiated above.

```c++
eprosima::fastdds::dds::DomainParticipantQos qos = PARTICIPANT_QOS_DEFAULT;

// Configure the current participant as SERVER
qos.wire_protocol().builtin.discovery_config.discoveryProtocol =
        eprosima::fastdds::rtps::DiscoveryProtocol::CLIENT;

// Add custom user transport with TCP port 0 (automatic port assignation)
auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
data_transport->add_listener_port(0);
qos.transport().user_transports.push_back(data_transport);

// Define the server locator to be on interface 192.168.10.57 and port 12345
constexpr uint16_t server_port = 12345;
eprosima::fastdds::rtps::Locator_t server_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(server_locator, "192.168.10.57");
eprosima::fastdds::rtps::IPLocator::setPhysicalPort(server_locator, server_port);
eprosima::fastdds::rtps::IPLocator::setLogicalPort(server_locator, server_port);

// Add the server
qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(server_locator);
```

### XML

The following snippet can be used to instantiate a client that will try to connect to a server on IP
192.168.10.57 and port 12345, that is, the server instantiated above.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                    <transport_id>client_tcp_transport</transport_id>
                    <type>TCPv4</type>
                    <!--
                        Set listening port for the transport to 0.
                        This automatically assigns a port.
                    -->
                    <listening_ports>
                        <port>0</port>
                    </listening_ports>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="TCP_CLIENT" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>CLIENT</discoveryProtocol>
                        <discoveryServersList>
                            <!--
                                Server locator specifying where it is listening
                            -->
                            <locator>
                                <tcpv4>
                                    <address>192.168.10.57</address>
                                    <port>12345</port>
                                    <physical_port>12345</physical_port>
                                </tcpv4>
                            </locator>
                        </discoveryServersList>
                    </discovery_config>
                </builtin>
                <useBuiltinTransports>false</useBuiltinTransports>
                <userTransports>
                    <transport_id>client_tcp_transport</transport_id>
                </userTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### NOTE
For TCPv6, see [TCPv6 Addresses and Interfaces](../../transport/tcp/tcp.md#transport-tcp-ipv6-addresses) for the considerations on the addresses and interfaces used.
