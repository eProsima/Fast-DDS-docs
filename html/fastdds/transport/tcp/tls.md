<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-tcp-tls"></a>

<a id="tls-over-tcp"></a>

# TLS over TCP

#### WARNING
This documentation assumes the reader has basic knowledge of TLS concepts
since terms like Certificate Authority (CA), Private Key, Rivest–Shamir–Adleman (RSA) cryptosystem,
and Diffie-Hellman encryption protocol are not explained in detail.

*Fast DDS* allows configuring TCP Transports to use TLS (Transport Layer Security).
In order to set up TLS, the [TCPTransportDescriptor](tcp.md#transport-tcp-transportdescriptor) must
have its [`apply_security`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor14apply_securityE) data member set to `true`, and its
[`tls_config`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor10tls_configE) data member filled with the desired configuration on the
[`TCPTransportDescriptor`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE).
The following is an example of configuration of TLS on the *TCP server*.

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
auto tls_transport = std::make_shared<TCPv4TransportDescriptor>();
tls_transport->sendBufferSize = 9216;
tls_transport->receiveBufferSize = 9216;
tls_transport->add_listener_port(5100);

// Create the TLS configuration
using TLSOptions = eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions;
tls_transport->apply_security = true;
tls_transport->tls_config.password = "test";
tls_transport->tls_config.cert_chain_file = "server.pem";
tls_transport->tls_config.private_key_file = "serverkey.pem";
tls_transport->tls_config.tmp_dh_file = "dh2048.pem";
tls_transport->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
tls_transport->tls_config.add_option(TLSOptions::SINGLE_DH_USE);
tls_transport->tls_config.add_option(TLSOptions::NO_SSLV2);

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(tls_transport);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>tls_transport_server</transport_id>
            <type>TCPv4</type>
            <tls>
                <password>test</password>
                <private_key_file>serverkey.pem</private_key_file>
                <cert_chain_file>server.pem</cert_chain_file>
                <tmp_dh_file>dh2048.pem</tmp_dh_file>
                <options>
                    <option>DEFAULT_WORKAROUNDS</option>
                    <option>SINGLE_DH_USE</option>
                    <option>NO_SSLV2</option>
                </options>
            </tls>
            <sendBufferSize>9216</sendBufferSize>
            <receiveBufferSize>9216</receiveBufferSize>
            <listening_ports>
                <port>5100</port>
            </listening_ports>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="TLSServerParticipant">
        <rtps>
            <userTransports>
                <transport_id>tls_transport_server</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

The corresponding configuration on the *TCP client* is shown in the following example.

### C++

```c++
DomainParticipantQos qos;

// Set initial peers.
Locator_t initial_peer_locator;
initial_peer_locator.kind = LOCATOR_KIND_TCPv4;
IPLocator::setIPv4(initial_peer_locator, "192.168.1.10");
initial_peer_locator.port = 5100;
qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer_locator);

// Create a descriptor for the new transport.
auto tls_transport = std::make_shared<TCPv4TransportDescriptor>();

// Create the TLS configuration
using TLSOptions = eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSOptions;
using TLSVerifyMode = eprosima::fastdds::rtps::TCPTransportDescriptor::TLSConfig::TLSVerifyMode;
tls_transport->apply_security = true;
tls_transport->tls_config.verify_file = "ca.pem";
tls_transport->tls_config.add_verify_mode(TLSVerifyMode::VERIFY_PEER);
tls_transport->tls_config.add_verify_mode(TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT);
tls_transport->tls_config.add_option(TLSOptions::DEFAULT_WORKAROUNDS);
tls_transport->tls_config.add_option(TLSOptions::SINGLE_DH_USE);
tls_transport->tls_config.add_option(TLSOptions::NO_SSLV2);
tls_transport->tls_config.server_name = "my_server.com";

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(tls_transport);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>tls_transport_client</transport_id>
            <type>TCPv4</type>
            <tls>
                <verify_file>ca.pem</verify_file>
                <verify_mode>
                    <verify>VERIFY_PEER</verify>
                    <verify>VERIFY_FAIL_IF_NO_PEER_CERT</verify>
                </verify_mode>
                <options>
                    <option>DEFAULT_WORKAROUNDS</option>
                    <option>SINGLE_DH_USE</option>
                    <option>NO_SSLV2</option>
                </options>
                <server_name>my_server.com</server_name>
            </tls>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="TLSClientParticipant">
        <rtps>
            <userTransports>
                <transport_id>tls_transport_client</transport_id>
            </userTransports>
            <builtin>
                <initialPeersList>
                    <locator>
                        <tcpv4>
                            <address>192.168.1.10</address>
                            <physical_port>5100</physical_port>
                        </tcpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

#### NOTE
For TCPv6, see [TCPv6 Addresses and Interfaces](tcp.md#transport-tcp-ipv6-addresses) for the considerations on the addresses and interfaces used.

The following table describes the data members that are configurable on [`TLSConfig`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfigE).

| Member                                                                                                                                                                                   | Data type                                                                                                                                                                        | Default                                                                                                                                                                                           | Description                                                                                                                                                                                                                                                                                                                                                                                       |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`password`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig8passwordE)                          | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Password of the [`private_key_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16private_key_fileE) or<br/>[`rsa_private_key_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig20rsa_private_key_fileE). |
| [`private_key_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16private_key_fileE)         | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Path to the private key certificate file.                                                                                                                                                                                                                                                                                                                                                         |
| [`rsa_private_key_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig20rsa_private_key_fileE) | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Path to the private key RSA certificate file.                                                                                                                                                                                                                                                                                                                                                     |
| [`cert_chain_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig15cert_chain_fileE)           | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Path to the public certificate chain file.                                                                                                                                                                                                                                                                                                                                                        |
| [`tmp_dh_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig11tmp_dh_fileE)                   | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Path to the Diffie-Hellman parameters file.                                                                                                                                                                                                                                                                                                                                                       |
| [`verify_file`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig11verify_fileE)                   | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Path to the CA (Certification- Authority) file.                                                                                                                                                                                                                                                                                                                                                   |
| [`verify_mode`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig11verify_modeE)                   | [`TLSVerifyMode`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyModeE)       | [`TLSVerifyMode::UNUSED`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode6UNUSEDE)         | Establishes the verification mode mask.<br/>See [TLS Verification Mode](#transport-tcp-tls-verifymode).                                                                                                                                                                                                                                                                                           |
| [`options`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig7optionsE)                            | [`TLSOptions`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptionsE)             | [`TLSOptions::NONE`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions4NONEE)                   | Establishes the SSL Context options mask.<br/>See [TLS Options](#transport-tcp-tls-options).                                                                                                                                                                                                                                                                                                      |
| [`verify_paths`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig12verify_pathsE)                 | `vector<string>`                                                                                                                                                                 | Empty vector                                                                                                                                                                                      | Paths where the system will look for verification files.                                                                                                                                                                                                                                                                                                                                          |
| [`verify_depth`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig12verify_depthE)                 | `int32_t`                                                                                                                                                                        | -1                                                                                                                                                                                                | Maximum allowed depth for verifying intermediate<br/>certificates.                                                                                                                                                                                                                                                                                                                                |
| [`default_verify_path`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig19default_verify_pathE)   | `bool`                                                                                                                                                                           | `false`                                                                                                                                                                                           | Look for verification files on the default paths.                                                                                                                                                                                                                                                                                                                                                 |
| [`handshake_role`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig14handshake_roleE)             | [`TLSHandShakeRole`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRoleE) | [`TLSHandShakeRole::DEFAULT`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRole7DEFAULTE) | Role that the transport will take on handshaking.<br/>See [TLS Handshake Role](#transport-tcp-tls-role).                                                                                                                                                                                                                                                                                          |
| [`server_name`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig11server_nameE)                   | `string`                                                                                                                                                                         | `""`                                                                                                                                                                                              | Server name or host name required in case<br/>Server Name Indication (SNI) is used.                                                                                                                                                                                                                                                                                                               |

#### NOTE
*Fast DDS* uses the [Boost.Asio](https://www.boost.org/doc/libs/1_73_0/doc/html/boost_asio.html) library to handle TLS secure connections.
These data members are used to build the asio library context, and most of them are mapped directly into this context
without further manipulation.
You can find more information about the implications of each member on the [Boost.Asio context](https://www.boost.org/doc/libs/1_73_0/doc/html/boost_asio/reference/ssl__context.html) documentation.

<a id="transport-tcp-tls-verifymode"></a>

<a id="tls-verification-mode"></a>

## TLS Verification Mode

The verification mode defines how the peer node will be verified.
The following table describes the available verification options.
Several verification options can be combined in the same [`TCPTransportDescriptor`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE)
using the [`add_verify_mode()`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig15add_verify_modeEK13TLSVerifyMode) member function.

| Value                                                                                                                                                                                                                                | Description                                                                                                                                                                                                                                                                              |
|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`TLSVerifyMode::VERIFY_NONE`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode11VERIFY_NONEE)                                 | Perform no verification.                                                                                                                                                                                                                                                                 |
| [`TLSVerifyMode::VERIFY_PEER`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode11VERIFY_PEERE)                                 | Perform verification of the peer.                                                                                                                                                                                                                                                        |
| [`TLSVerifyMode::VERIFY_FAIL_IF_NO_PEER_CERT`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode27VERIFY_FAIL_IF_NO_PEER_CERTE) | Fail verification if the peer has no certificate.<br/>Ignored unless [`TLSVerifyMode::VERIFY_PEER`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode11VERIFY_PEERE) is also set.   |
| [`TLSVerifyMode::VERIFY_CLIENT_ONCE`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode18VERIFY_CLIENT_ONCEE)                   | Do not request client certificate on renegotiation.<br/>Ignored unless [`TLSVerifyMode::VERIFY_PEER`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode11VERIFY_PEERE) is also set. |

#### NOTE
For a complete description of the different verification modes, please refer to the
[OpenSSL documentation](https://www.openssl.org/docs/man1.0.2/man3/SSL_CTX_set_verify.html).

<a id="transport-tcp-tls-options"></a>

<a id="tls-options"></a>

## TLS Options

These options define which TLS features are to be supported.
The following table describes the available options.
Several options can be combined in the same TransportDescriptor
using the [`add_option()`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10add_optionEK10TLSOptions) member function.

| Value                                                                                                                                                                                                          | Description                                                                                                                                             |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| [`TLSOptions::DEFAULT_WORKAROUNDS`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions19DEFAULT_WORKAROUNDSE) | Implement various bug workarounds.<br/>See [Boost.Asio context](https://www.boost.org/doc/libs/1_73_0/doc/html/boost_asio/reference/ssl__context.html). |
| [`TLSOptions::NO_COMPRESSION`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions14NO_COMPRESSIONE)           | Disable compression.                                                                                                                                    |
| [`TLSOptions::NO_SSLV2`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions8NO_SSLV2E)                        | Disable SSL v2.                                                                                                                                         |
| [`TLSOptions::NO_SSLV3`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions8NO_SSLV3E)                        | Disable SSL v3.                                                                                                                                         |
| [`TLSOptions::NO_TLSV1`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions8NO_TLSV1E)                        | Disable TLS v1.                                                                                                                                         |
| [`TLSOptions::NO_TLSV1_1`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions10NO_TLSV1_1E)                   | Disable TLS v1.1.                                                                                                                                       |
| [`TLSOptions::NO_TLSV1_2`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions10NO_TLSV1_2E)                   | Disable TLS v1.2.                                                                                                                                       |
| [`TLSOptions::NO_TLSV1_3`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions10NO_TLSV1_3E)                   | Disable TLS v1.3.                                                                                                                                       |
| [`TLSOptions::SINGLE_DH_USE`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions13SINGLE_DH_USEE)             | Always create a new key when using *Diffie-Hellman* parameters.                                                                                         |

<a id="transport-tcp-tls-role"></a>

<a id="tls-handshake-role"></a>

## TLS Handshake Role

The role can take the following values:

| Value                                                                                                                                                                                             | Description                                                  |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------|
| [`TLSHandShakeRole::DEFAULT`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRole7DEFAULTE) | Configured as client if connector, and as server if acceptor |
| [`TLSHandShakeRole::CLIENT`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRole6CLIENTE)   | Configured as client.                                        |
| [`TLSHandShakeRole::SERVER`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRole6SERVERE)   | Configured as server.                                        |
