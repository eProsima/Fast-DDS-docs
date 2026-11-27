<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transportdescriptors"></a>

<a id="transport-descriptors"></a>

# Transport descriptors

This section defines the XML elements available for configuring the transport layer parameters in *Fast DDS*.
These elements are defined within the XML tag `<transports_descriptors>`.
The `<transport_descriptors>` can contain one or more `<transport_descriptor>` XML elements.
Each `<transport_descriptor>` element defines a configuration for a specific type of transport protocol.
Each of these `<transport_descriptor>` elements are uniquely identified by a transport ID with the `<transport_id>`
XML tag.
Once the user defines a valid `<transports_descriptor>`, i.e. defines the transport layer parameters, these
can be loaded
into the XML profile of the DomainParticipant using the `<transport_id>` XML tag.
An example of how to load the `<transport_descriptor>` into the XML profile of the DomainParticipant is found in
[DomainParticipant profiles](domainparticipant.md#participantprofiles).

The following table lists all the available XML elements that can be defined within the `<transport_descriptor>`
element for the configuration of the transport layer.
A more detailed explanation of each of these elements can be found in [Transport Layer](../transport/transport.md#comm-transports-configuration).

| Name                          | Description                                                                                                                                                                                                                                                                             | Values                                                                             | Default   |
|-------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------|-----------|
| `<transport_id>`              | Unique name to identify each transport descriptor.                                                                                                                                                                                                                                      | `string`                                                                           |           |
| `<type>`                      | Type of the transport descriptor.                                                                                                                                                                                                                                                       | `UDPv4`, `UDPv6`, `TCPv4`, `TCPv6`, `SHM`, `POSIX_UDPv4` **[Pro]**                 | `UDPv4`   |
| `<sendBufferSize>`            | Size in bytes of the send socket buffer. If the value is zero, *Fast DDS* will use the system default socket size.<br/>**[Pro]** On TCP transports, a zero value lets the operating system auto-tune the buffer.                                                                        | `uint32_t`                                                                         | `0`       |
| `<receiveBufferSize>`         | Size in bytes of the reception socket buffer. If the value is zero, *Fast DDS* will use the system default socket size.<br/>**[Pro]** On TCP transports, a zero value lets the operating system auto-tune the buffer.                                                                   | `uint32_t`                                                                         | `0`       |
| `<maxMessageSize>`            | The maximum size in bytes of the transport’s message buffer.                                                                                                                                                                                                                            | `uint32_t`                                                                         | `65500`   |
| `<maxInitialPeersRange>`      | Number of channels opened with each initial remote peer.                                                                                                                                                                                                                                | `uint32_t`                                                                         | `4`       |
| `<netmask_filter>`            | Transport’s [Netmask filtering](../transport/interfaces.md#netmask-filtering) configuration.                                                                                                                                                                                            | `string`                                                                           | `AUTO`    |
| `<interfaces>`                | Allows defining an [Interfaces configuration](../transport/interfaces.md#ifaces-config).                                                                                                                                                                                                | `string`                                                                           |           |
| `<interfaceWhiteList>`        | Allows defining an interfaces [Whitelist](../transport/whitelist.md#whitelist-interfaces).                                                                                                                                                                                              | `string`                                                                           |           |
| `<TTL>`                       | Time To Live (UDP only). See  [UDP Transport](../transport/udp/udp.md#transport-udp-udp).                                                                                                                                                                                               | `uint8_t`                                                                          | `1`       |
| `<non_blocking_send>`         | Whether to set the non-blocking send mode on the socket (NOT available for SHM and POSIX_UDPv4 types). See [UDPTransportDescriptor](../transport/udp/udp.md#transport-udp-transportdescriptor) and [TCPTransportDescriptor](../transport/tcp/tcp.md#transport-tcp-transportdescriptor). | `bool`                                                                             | `false`   |
| `<output_port>`               | Port used for output bound. If this field isn’t defined, the output port will be random (UDPv4 and UDPv6 only).                                                                                                                                                                         | `uint16_t`                                                                         | `0`       |
| `<wan_addr>`                  | Public WAN address when using TCPv4 transports. This field is optional if the transport doesn’t need to define a WAN address (TCPv4 only).                                                                                                                                              | `string`                                                                           |           |
| `<keep_alive_frequency_ms>`   | **[Pro]** Frequency in milliseconds for sending [RTCP](#rtcpdefinition) keep-alive requests (TCP only).                                                                                                                                                                                 | `uint32_t`                                                                         | `0`       |
| `<keep_alive_timeout_ms>`     | **[Pro]** Time in milliseconds since the last keep-alive request was sent to consider a connection as broken (TCP only).                                                                                                                                                                | `uint32_t`                                                                         | `0`       |
| `<max_logical_port>`          | The maximum number of logical ports to try during [RTCP](#rtcpdefinition) negotiations (TCP only).                                                                                                                                                                                      | `uint16_t`                                                                         | `100`     |
| `<logical_port_range>`        | The maximum number of logical ports per request to try during [RTCP](#rtcpdefinition) negotiations (TCP only).                                                                                                                                                                          | `uint16_t`                                                                         | `20`      |
| `<logical_port_increment>`    | Increment between logical ports to try during [RTCP](#rtcpdefinition) negotiation (TCP only).                                                                                                                                                                                           | `uint16_t`                                                                         | `2`       |
| `<listening_ports>`           | Local port to work as TCP acceptor for input connections. If not set, the transport will work as TCP client only. If set to 0, an available port will be automatically assigned (TCP only).                                                                                             | `List<uint16_t>`                                                                   |           |
| `<tls>`                       | Allows to define TLS related parameters and options (TCP only).                                                                                                                                                                                                                         | [TLS Configuration](#tcp-tls)                                                      |           |
| `<calculate_crc>`             | Calculates the Cyclic Redundancy Code (CRC) for error control (TCP only).                                                                                                                                                                                                               | `bool`                                                                             | `true`    |
| `<check_crc>`                 | Check the CRC for error control (TCP only).                                                                                                                                                                                                                                             | `bool`                                                                             | `true`    |
| `<enable_tcp_nodelay>`        | Socket option for disabling the Nagle algorithm (TCP only).                                                                                                                                                                                                                             | `bool`                                                                             | `false`   |
| `<tcp_negotiation_timeout>`   | Time to wait for logical port negotiation (in ms) (TCP only).                                                                                                                                                                                                                           | `uint32_t`                                                                         | `0`       |
| `<keep_alive_thread>`         | Settings for the keep-alive thread.                                                                                                                                                                                                                                                     | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |           |
| `<accept_thread>`             | Settings for the accept thread.                                                                                                                                                                                                                                                         | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |           |
| `<segment_size>`              | Size (in bytes) of the shared-memory segment (Optional, SHM only).                                                                                                                                                                                                                      | `uint32_t`                                                                         | `262144`  |
| `<port_queue_capacity>`       | Capacity (in number of messages) available to every Listener (Optional, SHM only).                                                                                                                                                                                                      | `uint32_t`                                                                         | `512`     |
| `<healthy_check_timeout_ms>`  | Maximum time-out (in milliseconds) used when checking whether a Listener is alive (Optional, SHM only).                                                                                                                                                                                 | `uint32_t`                                                                         | `1000`    |
| `<rtps_dump_file>`            | Complete path (including file) where RTPS messages will be stored for debugging purposes. An empty string indicates no trace will be performed (Optional, SHM only).                                                                                                                    | `string`                                                                           | `Empty`   |
| `<default_reception_threads>` | Default [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads.                                                                                                                                                                   | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |           |
| `<reception_threads>`         | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads on specific ports.                                                                                                                                                         | `std::map<uint32_t, ThreadSettings>`                                               |           |
| `<dump_thread>`               | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the SHM dump thread.                                                                                                                                                                             | [ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |           |

The following XML code shows an example of transport protocol configuration using all configurable parameters.
More examples of transports descriptors can be found in the [Transport Layer](../transport/transport.md#comm-transports-configuration) section.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>my_udpv4_transport</transport_id>
                <type>UDPv4</type>
                <sendBufferSize>8192</sendBufferSize>
                <receiveBufferSize>8192</receiveBufferSize>
                <maxMessageSize>16384</maxMessageSize>
                <maxInitialPeersRange>100</maxInitialPeersRange>
                <netmask_filter>AUTO</netmask_filter>
                <interfaces>
                    <allowlist>
                        <interface name="wlp59s0" netmask_filter="ON"/>
                    </allowlist>
                    <blocklist>
                        <interface name="127.0.0.1"/>
                        <interface name="docker0"/>
                    </blocklist>
                </interfaces>
                <interfaceWhiteList>
                    <address>192.168.1.41</address>
                    <interface>lo</interface>
                </interfaceWhiteList>
                <TTL>250</TTL>
                <non_blocking_send>false</non_blocking_send>
                <output_port>5101</output_port>
                <wan_addr>80.80.55.44</wan_addr>
                <keep_alive_frequency_ms>5000</keep_alive_frequency_ms>
                <keep_alive_timeout_ms>25000</keep_alive_timeout_ms>
                <max_logical_port>9000</max_logical_port>
                <logical_port_range>100</logical_port_range>
                <logical_port_increment>2</logical_port_increment>
                <listening_ports>
                    <port>5100</port>
                    <port>5200</port>
                </listening_ports>
                <tls><!-- TLS Section --></tls>
                <calculate_crc>false</calculate_crc>
                <check_crc>false</check_crc>
                <enable_tcp_nodelay>false</enable_tcp_nodelay>
                <segment_size>262144</segment_size>
                <port_queue_capacity>512</port_queue_capacity>
                <healthy_check_timeout_ms>1000</healthy_check_timeout_ms>
                <rtps_dump_file>rtsp_messages.log</rtps_dump_file>
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
                <dump_thread>
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </dump_thread>
            </transport_descriptor>
        </transport_descriptors>
    </profiles>
</dds>
```

<a id="rtcpdefinition"></a>

#### NOTE
The Real-time Transport Control Protocol ([RTCP](https://tools.ietf.org/html/rfc3550)) is the control protocol
for communications with RTPS over TCP/IP connections.

<a id="tcp-tls"></a>

<a id="tls-configuration"></a>

## TLS Configuration

*Fast DDS* provides mechanisms to configure the Transport Layer Security (TLS) protocol parameters
through the `<tls>` XML element of its `<transport_descriptor>`.
Please, refer to [TLS over TCP](../transport/tcp/tls.md#transport-tcp-tls) for a detailed explanation of the entire TLS configuration in *Fast DDS*.
More information on how to set up secure communication in *Fast DDS* can be found in the [Security](../security/security.md#security) section.

#### WARNING
For the full understanding of this section, a basic knowledge of network security
in terms of SSL/TLS, Certificate Authority (CA), Public Key Infrastructure (PKI), and Diffie-Hellman is required;
encryption protocols are not explained in detail.

The full list of available XML elements that can be defined within the `<tls>` element to configure the TLS
protocol are listed in the following table:

| Name                          | Description                                                                                                                                     | Values                | Default   |
|-------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------|-----------|
| `<password>`                  | Password of the<br/>`<private_key_file>`<br/>or<br/>`<rsa_private_key_file>`<br/>if provided.                                                   | `string`              |           |
| `<private_key_file>`          | Path to the private key<br/>certificate file.                                                                                                   | `string`              |           |
| `<rsa_private_key_file>`      | Path to the private key<br/>RSA certificate file.                                                                                               | `string`              |           |
| `<cert_chain_file>`           | Path to the public certificate<br/>chain file.                                                                                                  | `string`              |           |
| `<tmp_dh_file>`               | Path to the Diffie-Hellman<br/>parameters file                                                                                                  | `string`              |           |
| `<verify_file>`               | Path to the Certification<br/>Authority (CA) file.                                                                                              | `string`              |           |
| `<verify_mode>`               | Establishes the verification<br/>mode mask. Several<br/>verification options can be<br/>combined in the same<br/>`<transport_descriptor>`.      | `VERIFY_NONE`         |           |
| `VERIFY_PEER`                 |                                                                                                                                                 |                       |           |
| `VERIFY_FAIL_IF_NO_PEER_CERT` |                                                                                                                                                 |                       |           |
| `VERIFY_CLIENT_ONCE`          |                                                                                                                                                 |                       |           |
| `<options>`                   | Establishes the SSL Context<br/>options mask. Several<br/>options can be combined in the<br/>same<br/>`<transport_descriptor>`.                 | `DEFAULT_WORKAROUNDS` |           |
| `NO_COMPRESSION`              |                                                                                                                                                 |                       |           |
| `NO_SSLV2`                    |                                                                                                                                                 |                       |           |
| `NO_SSLV3`                    |                                                                                                                                                 |                       |           |
| `NO_TLSV1`                    |                                                                                                                                                 |                       |           |
| `NO_TLSV1_1`                  |                                                                                                                                                 |                       |           |
| `NO_TLSV1_2`                  |                                                                                                                                                 |                       |           |
| `NO_TLSV1_3`                  |                                                                                                                                                 |                       |           |
| `SINGLE_DH_USE`               |                                                                                                                                                 |                       |           |
| `<verify_paths>`              | Paths where the system will<br/>look for verification      files.                                                                               | `List <string>`       |           |
| `<verify_depth>`              | Maximum allowed depth to<br/>verify intermediate<br/>certificates.                                                                              | `uint32_t`            |           |
| `<default_verify_path>`       | Specifies whether the system<br/>will look on the<br/>default paths for the<br/>verification files.                                             | `bool`                | `false`   |
| `<handshake_role>`            | Role that the transport will<br/>take on handshaking.<br/>On default, the acceptors act<br/>as `SERVER` and the<br/>connectors as<br/>`CLIENT`. | `DEFAULT`             | `DEFAULT` |
| `SERVER`                      |                                                                                                                                                 |                       |           |
| `CLIENT`                      |                                                                                                                                                 |                       |           |
| `<server_name>`               | server name or host name required<br/>in case Server Name Indication<br/>(SNI) is used.                                                         | `string`              |           |

An example of TLS protocol parameter configuration is shown below.

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>Test</transport_id>
                <type>TCPv4</type>
                <tls>
                    <password>Password</password>
                    <private_key_file>Key_file.pem</private_key_file>
                    <rsa_private_key_file>RSA_file.pem</rsa_private_key_file>
                    <cert_chain_file>Chain.pem</cert_chain_file>
                    <tmp_dh_file>DH.pem</tmp_dh_file>
                    <verify_file>verify.pem</verify_file>
                    <verify_mode>
                        <verify>VERIFY_PEER</verify>
                    </verify_mode>
                    <options>
                        <option>NO_TLSV1</option>
                        <option>NO_TLSV1_1</option>
                    </options>
                    <verify_paths>
                        <verify_path>Path1</verify_path>
                        <verify_path>Path2</verify_path>
                        <verify_path>Path3</verify_path>
                    </verify_paths>
                    <verify_depth>55</verify_depth>
                    <default_verify_path>true</default_verify_path>
                    <handshake_role>SERVER</handshake_role>
                    <server_name>my_server.com</server_name>
                </tls>
            </transport_descriptor>
<!-->
    </profiles>
```
