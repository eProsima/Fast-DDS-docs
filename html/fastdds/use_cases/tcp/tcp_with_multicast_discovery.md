<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-tcp-multicast"></a>

<a id="large-data-mode"></a>

# Large Data Mode

The following snippets show how to configure *Fast DDS* [DomainParticipants](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) to run the
[PDP discovery](../../discovery/discovery.md#disc-phases) phase over UDP multicast and communicate application data over a
[TCP Transport](../../transport/tcp/tcp.md#transport-tcp-tcp) or [Shared Memory Transport](../../transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory), which is called the `LARGE_DATA`
configuration (See [Managing the Builtin Transports](../../rtps_layer/rtps_layer.md#rtps-layer-builtin-transports)).
With this approach, applications managing large samples can benefit from transmitting their data over TCP or SHM,
while at the same time have the flexibility of automatic discovery.
The `LARGE_DATA` mode can be set using the `FASTDDS_BUILTIN_TRANSPORTS` environment variable
(see [FASTDDS_BUILTIN_TRANSPORTS](../../env_vars/env_vars.md#env-vars-builtin-transports)), XML profiles or via code.

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
eprosima::fastdds::dds::DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;

/* Transports configuration */
// UDPv4 transport for PDP over multicast and SHM / TCPv4 transport for EDP and application data
pqos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA);

/* Create participant as usual */
eprosima::fastdds::dds::DomainParticipant* participant =
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
```

#### NOTE
`LARGE_DATA` configuration of the builtin transports will also create a SHM transport along the UDP and TCP
transports. Shared Memory will be used whenever it is possible. Manual configuration will be required if a TCP
communication is required when SHM is feasible.

### C++

```c++
eprosima::fastdds::dds::DomainParticipantQos pqos = PARTICIPANT_QOS_DEFAULT;

/* Transports configuration */
// UDPv4 transport for PDP over multicast
auto pdp_transport = std::make_shared<eprosima::fastdds::rtps::UDPv4TransportDescriptor>();
pqos.transport().user_transports.push_back(pdp_transport);

// TCPv4 transport for EDP and application data (The listening port must to be unique for
// each participant in the same host)
constexpr uint16_t tcp_listening_port = 0;
auto data_transport = std::make_shared<eprosima::fastdds::rtps::TCPv4TransportDescriptor>();
data_transport->add_listener_port(tcp_listening_port);
pqos.transport().user_transports.push_back(data_transport);

pqos.transport().use_builtin_transports = false;

/* Locators */
// Define locator for PDP over multicast
eprosima::fastdds::rtps::Locator_t pdp_locator;
pdp_locator.kind = LOCATOR_KIND_UDPv4;
eprosima::fastdds::rtps::IPLocator::setIPv4(pdp_locator, "239.255.0.1");
pqos.wire_protocol().builtin.metatrafficMulticastLocatorList.push_back(pdp_locator);

// Define locator for EDP and user data
eprosima::fastdds::rtps::Locator_t tcp_locator;
tcp_locator.kind = LOCATOR_KIND_TCPv4;
eprosima::fastdds::rtps::IPLocator::setIPv4(tcp_locator, "0.0.0.0");
eprosima::fastdds::rtps::IPLocator::setPhysicalPort(tcp_locator, tcp_listening_port);
eprosima::fastdds::rtps::IPLocator::setLogicalPort(tcp_locator, tcp_listening_port);
pqos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(tcp_locator);
pqos.wire_protocol().default_unicast_locator_list.push_back(tcp_locator);

/* Create participant as usual */
eprosima::fastdds::dds::DomainParticipant* participant =
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <transport_descriptors>
            <!--
                UDP transport for PDP
            -->
            <transport_descriptor>
                    <transport_id>pdp_transport</transport_id>
                    <type>UDPv4</type>
            </transport_descriptor>
            <!--
                TCP transport for both EDP and application data
            -->
            <transport_descriptor>
                <transport_id>data_transport</transport_id>
                <type>TCPv4</type>
                <!--
                    Set listening port for the transport.
                    This needs to be unique for each participant in the host
                -->
                <listening_ports>
                    <port>0</port>
                </listening_ports>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="udp_multicast_pdp_w_tcp_data" is_default_profile="true">
            <rtps>
                <!--
                    Set participant transports
                -->
                <userTransports>
                    <transport_id>pdp_transport</transport_id>
                    <transport_id>data_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
                <!--
                    Discovery configuration
                -->
                <builtin>
                    <!--
                        PDP over UDP multicast
                    -->
                    <metatrafficMulticastLocatorList>
                        <locator>
                            <udpv4>
                                <address>239.255.0.1</address>
                            </udpv4>
                        </locator>
                    </metatrafficMulticastLocatorList>
                    <!--
                        EDP over TCP
                    -->
                    <metatrafficUnicastLocatorList>
                        <locator>
                            <tcpv4>
                                <address>0.0.0.0</address>
                                <physical_port>12345</physical_port>
                                <port>12345</port>
                            </tcpv4>
                        </locator>
                    </metatrafficUnicastLocatorList>
                </builtin>
                <!--
                    Application data over TCP
                -->
                <defaultUnicastLocatorList>
                    <locator>
                        <tcpv4>
                            <address>0.0.0.0</address>
                            <physical_port>12345</physical_port>
                            <port>12345</port>
                        </tcpv4>
                    </locator>
                </defaultUnicastLocatorList>
            </rtps>
        </participant>
    </profiles>
</dds>
```
