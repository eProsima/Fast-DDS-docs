<a id="api-tcp-transport-descriptor"></a>

<a id="tcptransportdescriptor"></a>

# TCPTransportDescriptor

### struct TCPTransportDescriptor : public eprosima::fastdds::rtps::[SocketTransportDescriptor](../transport_generic_interfaces/socket_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps25SocketTransportDescriptorE)

TCP Transport configuration

* `listening_ports:` list of ports to listen as server.
* `keep_alive_frequency_ms:` frequency of RTCP keep alive requests (in ms).
* `keep_alive_timeout_ms:` time since sending the last keep alive request to consider a connection as broken (in ms).
* `max_logical_port:` maximum number of logical ports to try during RTCP negotiation.
* `logical_port_range:` maximum number of logical ports per request to try during RTCP negotiation.
* `logical_port_increment:` increment between logical ports to try during RTCP negotiation.
* `enable_tcp_nodelay:` enables the TCP_NODELAY socket option.
* `calculate_crc:` true to calculate and send CRC on message headers.
* `check_crc:` true to check the CRC of incoming message headers.
* `apply_security:` true to use TLS (Transport Layer Security).
* `tls_config:` Configuration for TLS.
* `non_blocking_send:` do not block on send operations. When it is set to true, send operations will return immediately if the buffer might get full, but no error will be returned to the upper layer. This means that the application will behave as if the datagram is sent and lost.
* `tcp_negotiation_timeout:` time to wait for logical port negotiation (in ms). 

Subclassed by [eprosima::fastdds::rtps::TCPv4TransportDescriptor](tcpv4_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1TCPv4TransportDescriptor), [eprosima::fastdds::rtps::TCPv6TransportDescriptor](tcpv6_transport_descriptor.md#structeprosima_1_1fastdds_1_1rtps_1_1TCPv6TransportDescriptor)

### Public Functions

### inline void add_listener_port(std::uint16_t port)

Add listener port to the listening_ports list. 

### TCPTransportDescriptor()

Constructor. 

### TCPTransportDescriptor(const [TCPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor22TCPTransportDescriptorERK22TCPTransportDescriptor) &t)

Copy constructor. 

### [TCPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE) &operator=(const [TCPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE) &t)

Copy assignment. 

### virtual ~TCPTransportDescriptor() = default

Destructor. 

### bool operator==(const [TCPTransportDescriptor](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptorE) &t) const

Comparison operator. 

### Public Members

### std::vector<std::uint16_t> listening_ports

List of ports to listen as server. 

### std::uint32_t keep_alive_frequency_ms

Frequency of RTCP keep alive requests (ms) 

### std::uint32_t keep_alive_timeout_ms

Time since sending the last keep alive request to consider a connection as broken (ms) 

### std::uint16_t max_logical_port

Maximum number of logical ports to try during RTCP negotiation. 

### std::uint16_t logical_port_range

Maximum number of logical ports per request to try during RTCP negotiation. 

### std::uint16_t logical_port_increment

Increment between logical ports to try during RTCP negotiation. 

### std::uint32_t tcp_negotiation_timeout

Time to wait for logical port negotiation (ms). If a logical port is under negotiation, it waits for the negotiation to finish up to this timeout before trying to send a message to that port. Zero value means no waiting (default). 

### bool enable_tcp_nodelay

Enables the TCP_NODELAY socket option. 

### bool calculate_crc

Enables the calculation and sending of CRC on message headers. 

### bool check_crc

Enables checking the CRC of incoming message headers. 

### bool apply_security

Enables the use of TLS (Transport Layer Security) 

### [TLSConfig](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfigE) tls_config

Configuration of the TLS (Transport Layer Security) 

### [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) keep_alive_thread

Thread settings for keep alive thread. 

### [ThreadSettings](../../rtps/attributes/ThreadSettings.md#_CPPv4N8eprosima7fastdds4rtps14ThreadSettingsE) accept_thread

Thread settings for the accept connections thread. 

### bool non_blocking_send

Whether to use non-blocking calls to send().

When set to true, calls to send() will return immediately if the send buffer might get full. This may happen when receive buffer on reader’s side is full. No error will be returned to the upper layer. This means that the application will behave as if the datagram is sent but lost (i.e. throughput may be reduced). This value is specially useful on high-frequency writers.

When set to false, which is the default, calls to send() will block until the send buffer has space for the datagram. This may cause application lock. 

### struct TLSConfig

TLS Configuration

* `password:` password of the `private_key_file` or `rsa_private_key_file`.
* `private_key_file:` path to the private key certificate file.
* `rsa_private_key_file:` path to the private key RSA certificate file.
* `cert_chain_file:` path to the public certificate chain file.
* `tmp_dh_file:` path to the Diffie-Hellman parameters file.
* `verify_file:` path to the CA (Certification-Authority) file.
* `verify_mode:` establishes the verification mode mask.
* `options:` establishes the SSL Context options mask.
* `verify_paths:` paths where the system will look for verification files.
* `default_verify_path:` look for verification files on the default paths.
* `handshake_role:` role that the transport will take on handshaking.
* `server_name:` server name or host name required in case Server Name Indication (SNI) is used. 

### Public Types

### enum TLSOptions

Supported TLS features. Several options can be combined in the same TransportDescriptor using the [add_option()](#structeprosima_1_1fastdds_1_1rtps_1_1TCPTransportDescriptor_1_1TLSConfig_1a24dc96cdfb3c3b91043e82a676595c67) member function.

* DEFAULT_WORKAROUNDS: implement various bug workarounds.
* NO_COMPRESSION: disable compression.
* NO_SSLV2: disable SSL v2.
* NO_SSLV3: disable SSL v3.
* NO_TLSV1: disable TLS v1.
* NO_TLSV1_1: disable TLS v1.1.
* NO_TLSV1_2: disable TLS v1.2.
* NO_TLSV1_3: disable TLS v1.3.
* SINGLE_DH_USE: always create a new key using Diffie-Hellman parameters. 

*Values:*

### enumerator NONE

### enumerator DEFAULT_WORKAROUNDS

### enumerator NO_COMPRESSION

### enumerator NO_SSLV2

### enumerator NO_SSLV3

### enumerator NO_TLSV1

### enumerator NO_TLSV1_1

### enumerator NO_TLSV1_2

### enumerator NO_TLSV1_3

### enumerator SINGLE_DH_USE

### enum TLSVerifyMode

Peer node verification options. Several verification options can be combined in the same TransportDescriptor using the [add_verify_mode()](#structeprosima_1_1fastdds_1_1rtps_1_1TCPTransportDescriptor_1_1TLSConfig_1aacde2ec6a56354771fcc634d30b06a3e) member function.

* VERIFY_NONE: perform no verification.
* VERIFY_PEER: perform verification of the peer.
* VERIFY_FAIL_IF_NO_PEER_CERT: fail verification if the peer has no certificate. Ignored unless VERIFY_PEER is also set.
* VERIFY_CLIENT_ONCE: do not request client certificate on renegotiation. Ignored unless VERIFY_PEER is also set. 

*Values:*

### enumerator UNUSED

### enumerator VERIFY_NONE

### enumerator VERIFY_PEER

### enumerator VERIFY_FAIL_IF_NO_PEER_CERT

### enumerator VERIFY_CLIENT_ONCE

### enum TLSHandShakeRole

Role that the transport will take on handshaking.

* DEFAULT: configured as client if connector, and as server if acceptor.
* CLIENT: configured as client.
* SERVER: configured as server. 

*Values:*

### enumerator DEFAULT

### enumerator CLIENT

### enumerator SERVER

### Public Functions

### inline void add_verify_mode(const [TLSVerifyMode](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyModeE) verify)

Add verification modes to the verification mode mask. 

### inline bool get_verify_mode(const [TLSVerifyMode](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyModeE) verify) const

Get the verification mode mask. 

### inline void add_option(const [TLSOptions](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptionsE) option)

Add TLS features to the SSL Context options mask. 

### inline bool get_option(const [TLSOptions](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptionsE) option) const

Get the SSL Context options mask. 

### inline bool operator==(const [TLSConfig](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfigE) &t) const

Comparison operator. 

### Public Members

### std::string password

Password of the `private_key_file` or `rsa_private_key_file`. 

### std::uint32_t options = [TLSOptions](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptionsE)::[NONE](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig10TLSOptions4NONEE)

SSL context options mask. 

### std::string cert_chain_file

Path to the public certificate chain file. 

### std::string private_key_file

Path to the private key certificate file. 

### std::string tmp_dh_file

Path to the Diffie-Hellman parameters file. 

### std::string verify_file

Path to the CA (Certification-Authority) file. 

### std::uint8_t verify_mode = [TLSVerifyMode](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyModeE)::[UNUSED](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig13TLSVerifyMode6UNUSEDE)

Verification mode mask. 

### std::vector<std::string> verify_paths

Paths where the system will look for verification files. 

### bool default_verify_path = false

Look for verification files on the default paths. 

### std::int32_t verify_depth = -1

Maximum allowed depth for verifying intermediate certificates. Do not override. 

### std::string rsa_private_key_file

Path to the private key RSA certificate file. 

### [TLSHandShakeRole](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRoleE) handshake_role = [TLSHandShakeRole](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRoleE)::[DEFAULT](#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9TLSConfig16TLSHandShakeRole7DEFAULTE)

Role that the transport will take on handshaking. 

### std::string server_name

Server name or host name required in case Server Name Indication (SNI) is used.
