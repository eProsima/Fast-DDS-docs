<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="env-vars"></a>

<a id="environment-variables"></a>

# Environment variables

This is the list of environment variables that affect the behavior of *Fast DDS*:

<a id="env-vars-fastdds-default-profiles-file"></a>

<a id="fastdds-default-profiles-file"></a>

## `FASTDDS_DEFAULT_PROFILES_FILE`

Defines the location of the default profile configuration XML file.
If this variable is set and its value corresponds with an existing file, *Fast DDS* will load its profiles.
For more information about XML profiles, please refer to [XML profiles](../xml_configuration/xml_configuration.md#xml-profiles).

| **Linux**                                                                        |
|----------------------------------------------------------------------------------|
| ```bash<br/>export FASTDDS_DEFAULT_PROFILES_FILE=/home/user/profiles.xml<br/>``` |
| **Windows**                                                                      |
| ```bash<br/>set FASTDDS_DEFAULT_PROFILES_FILE=C:\profiles.xml<br/>```            |

<a id="env-vars-skip-default-xml"></a>

<a id="skip-default-xml"></a>

## `SKIP_DEFAULT_XML`

Skips looking for a default profile configuration XML file.
If this variable is set to 1, *Fast DDS* will load the configuration parameters directly from the classes’
definitions without looking for the *DEFAULT_FASTDDS_PROFILES.xml* in the working directory.
For more information about XML profiles, please refer to [XML profiles](../xml_configuration/xml_configuration.md#xml-profiles).

| **Linux**                                     |
|-----------------------------------------------|
| ```bash<br/>export SKIP_DEFAULT_XML=1<br/>``` |
| **Windows**                                   |
| ```bash<br/>set SKIP_DEFAULT_XML=1<br/>```    |

<a id="env-vars-builtin-transports"></a>

<a id="fastdds-builtin-transports"></a>

## `FASTDDS_BUILTIN_TRANSPORTS`

Setting this variable allows to modify the builtin transports that are initialized during the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
creation. It is a simple way of changing the default configuration of the [Transport Layer](../transport/transport.md#comm-transports-configuration)
and it directly affects how DDS entities communicate between them.

All existing values, along with a brief description, are shown below:

| Builtin Transports Options   | Description                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|------------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `NONE`                       | No transport will be instantiated. Hence, the user must manually add<br/>the desired  transports. Otherwise, the participant creation will fail.                                                                                                                                                                                                                                                                                                                                                  |
| `DEFAULT`                    | UDPv4 and SHM transports will be instantiated. SHM transport has priority<br/>over the UDPv4  transport. Meaning that SHM will always be used<br/>when possible.                                                                                                                                                                                                                                                                                                                                  |
| `DEFAULTv6`                  | UDPv6 and SHM transports will be instantiated. SHM transport has priority<br/>over the UDPv6 transport. Meaning that SHM will always be used<br/>when possible.                                                                                                                                                                                                                                                                                                                                   |
| `SHM`                        | Only a SHM transport will be instantiated.                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
| `UDPv4`                      | Only a UDPv4 transport will be instantiated.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `UDPv6`                      | Only a UDPv6 transport will be instantiated.                                                                                                                                                                                                                                                                                                                                                                                                                                                      |
| `LARGE_DATA`                 | UDPv4, TCPv4, and SHM transports will be instantiated. However, UDP will<br/>only be used  for multicast announcements during the participant<br/>discovery phase (see [Discovery phases](../discovery/discovery.md#disc-phases))  while the participant<br/>liveliness and the application data delivery occurs over TCP or SHM.<br/>This configuration is useful when working with large data.(See<br/>[Large Data mode and Fast DDS over TCP](../use_cases/tcp/tcp_use_case.md#use-case-tcp)). |

#### NOTE
The environment variable is only used in the case where [`use_builtin_transports`](../api_reference/dds_pim/core/policy/transportconfigqos.md#_CPPv4N8eprosima7fastdds3dds18TransportConfigQos22use_builtin_transportsE) is set
to `TRUE`. In any other case, the environment variable has no effect.

#### NOTE
TCPv4 transport is initialized with the following configuration:

* [`calculate_crc`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor13calculate_crcE), [`check_crc`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor9check_crcE) and
  [`apply_security`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor14apply_securityE) are set to false.
* [`enable_tcp_nodelay`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor18enable_tcp_nodelayE) is set to true.
* [`keep_alive_thread`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17keep_alive_threadE) and
  [`accept_thread`](../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor13accept_threadE) use the default configuration.

<a id="env-vars-large-data-options"></a>

<a id="configuring-builtin-transports-options"></a>

### Configuring builtin transports options

Fast DDS offers a straightforward method to configure three main parameters of every builtin transport via the
environment variable.
However, this feature proves particularly valuable when employing the `LARGE_DATA` builtin
transports option.
The `LARGE_DATA` mode has been designed to improve performance when working with large data.
However, according to each specific use case, the user might want to configure several options to better fit their
needs.
This mode can also be configured with the `tcp_negotiation_timeout` parameter:

| Builtin Transports Options   | Description                                                                                                                                                                                                                                                                | Type     |
|------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|----------|
| `max_msg_size`               | It determines the maximum message size that will be specified<br/>in the transport layer. Selecting a message size large<br/>enough to accommodate the largest data message will<br/>prevent fragmentation, which can significantly enhance<br/>the overall sending rate.  | uint32_t |
| `sockets_size`               | It determines the size of the send and receive socket buffers.<br/>This parameter needs to be higher or equal to the maximum<br/>message size specified in order to be valid.<br/>**[Pro]** If not set, the TCP socket buffers are auto-tuned by the<br/>operating system. | uint32_t |
| `non_blocking`               | It determines whether to use non-blocking send calls or not.<br/>When activated, the transport will discard messages if the<br/>socket buffers are full.                                                                                                                   | bool     |
| `tcp_negotiation_timeout`    | It determines the time to wait for logical port negotiation.<br/>Only valid if the `LARGE_DATA` mode is being used.<br/>It only accepts milliseconds.                                                                                                                      | uint32_t |

The environment variable accepts several types of units to specify the values of the parameters.
Also, both lowercase and uppercase letters are valid.
The following list shows the available units and their corresponding symbols:

+ `B`: Bytes. This is the default unit, so it is not necessary to specify it.
+ `KB`: Kilobytes.
+ `MB`: Megabytes.
+ `GB`: Gigabytes.
+ `KIB`: Kibibyte.
+ `MIB`: Mebibyte.
+ `GIB`: Gibibyte.

```bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=200KB&sockets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50
```

#### NOTE
When working with `LARGE_DATA`, it is recommended to set the `max_msg_size` and `sockets_size` to a value
large enough to accommodate the largest data message and to set the `non_blocking` to `TRUE`.
Note that activating the `non_blocking` option with a small message size (with fragmentation)
can lead to an increase of messages drop rate and produce undesired results.
For more information, please refer to [Large Data with configuration options](../use_cases/tcp/tcp_large_data_with_options.md#use-case-large-data-options).

#### WARNING
Setting a `max_msg_size` higher than 65500 KB is only possible when using the `LARGE_DATA` builtin transports
option.
Trying to set it with any other builtin transports will result in an error and the creation of the participant
will fail.

<a id="env-vars-ros-discovery-server"></a>

<a id="ros-discovery-server"></a>

## `ROS_DISCOVERY_SERVER`

#### WARNING
The environment variable is only used in the case where [discovery protocol](../discovery/general_disc_settings.md#discovery-protocol)
is set to [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE), [`SERVER`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE), or [`BACKUP`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6BACKUPE).
In any other case, the environment variable has no effect.

Setting this variable configures the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) to connect to one or more
*servers* using the [Discovery Server](../discovery/discovery_server.md#discovery-server) discovery mechanism.

* If `ROS_DISCOVERY_SERVER` is defined, and the `DomainParticipant`’s [discovery protocol](../discovery/general_disc_settings.md#discovery-protocol)
  is set to [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE), then Fast DDS will instead configure it as [`CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6CLIENTE) of the given *server*.
* If `ROS_DISCOVERY_SERVER` is defined, and the `DomainParticipant`’s [discovery protocol](../discovery/general_disc_settings.md#discovery-protocol)
  is set to [`SERVER`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE) or [`BACKUP`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6BACKUPE), then the variable is used to add remote *servers* to the given *server*, leaving the
  [discovery protocol](../discovery/general_disc_settings.md#discovery-protocol) as [`SERVER`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE) or [`BACKUP`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6BACKUPE) respectively.
* The value of the variable must list the locator of the server in the form of:
  + An IPv4 address like `192.168.2.23`. The UDP protocol is used by default. The UDP port can be appended using :
    as in `192.168.2.23:35665`.
  + An IPv6 address that follows [RFC3513](https://www.rfc-editor.org/rfc/rfc3513) address convention like `1080::8:800:200C:417A`. Again, it uses the UDP
    protocol by default. An UDP port can be appended like in `[1080::8:800:200C:417A]:35665`. Note the use of square
    brackets to avoid ambiguities.
  + TCPv4 specifier + IPv4 address like `TCPv4:[127.0.0.1]`. The TCP protocol is used to communicate with the server.
    The TCP port can be appended using : as in `TCPv4:[127.0.0.1]:42100`.
  + TCPv6 specifier + IPv6 address like `TCPv6:[::1]`. The TCP protocol is used to communicate with the server. The
    TCP port can be appended using : as in `TCPv6:[::1]:42100`.
  + A DNS name can be specified. This name will be used to query known hosts and available DNS servers to try to
    resolve valid IP addresses. Several formats are acceptable:
    - Plain domain name: `eprosima.com`. This will include all available IP addresses.
    - Domain name + port: `eprosima.com:35665`. As above but using a specific port.
    - UDPv4 specifier + domain name: `UDPv4:[eprosima.com]`. Only the first IPv4 address resolved will be used.
    - UDPv4 specifier + domain name + port: `UDPv4:[eprosima.com]:35665`. As above but using a specific port.
    - UDPv6 specifier + domain name: `UDPv6:[<dns>]`. Only the first IPv6 address resolved will be used.
    - UDPv6 specifier + domain name + port: `UDPv6:[<dns>]:35665`. As above but using a specific port.
    - TCPv4 specifier + domain name: `TCPv4:[eprosima.com]`. Only the first IPv4 address resolved will be used.
    - TCPv4 specifier + domain name + port: `TCPv4:[eprosima.com]:42100`. As above but using a specific port.
    - TCPv6 specifier + domain name: `TCPv6:[<dns>]`. Only the first IPv6 address resolved will be used.
    - TCPv6 specifier + domain name + port: `TCPv6:[<dns>]:42100`. As above but using a specific port.
* If no port is specified when using default UDP transport, the default port 11811 is used.
* If no port is specified when using TCP transport, the default port 42100 is used.
* To set more than one *server*’s address, they must be separated by semicolons.
* When using IPv6 with DNS, the specified domain name space ( *<dns>*) must be able to resolve to an IPv6
  address. Otherwise an error will be raised.

The following example shows how to set the address of two remote discovery servers with addresses
‘84.22.259.329:8888’ and ‘localhost:1234’.

| **Linux**                                                                           |
|-------------------------------------------------------------------------------------|
| ```bash<br/>export ROS_DISCOVERY_SERVER="84.22.259.329:8888;localhost:1234"<br/>``` |
| **Windows**                                                                         |
| ```bash<br/>set ROS_DISCOVERY_SERVER=84.22.259.329:8888;localhost:1234<br/>```      |

#### IMPORTANT
IP addresses specified in `ROS_DISCOVERY_SERVER` must be either valid IPv4/IPv6 addresses or domain names.
If a name can be resolved into several addresses, it is possible to either use them all or restrict the selection to
the first IPv4 using the UDPv4 or TCPv4 prefixes or to the first IPv6 address using the UDPv6 or TCPv6
prefixes.

#### IMPORTANT
This environment variable can be changed at runtime adding new remote servers to a [`SERVER`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SERVERE), [`BACKUP`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6BACKUPE) or [`CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6CLIENTE)
(that has been initialized with this environment variable previously) if loaded from an environment file using
[FASTDDS_ENVIRONMENT_FILE](#env-vars-fastdds-environment-file).

<a id="env-vars-easy-mode"></a>

<a id="ros2-easy-mode"></a>

## `ROS2_EASY_MODE`

Setting `ROS2_EASY_MODE` to an IP value allows a participant to automatically enter the
[Discovery Server Easy Mode](https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html).
This mode completely disables **multicast communication**, and relies on Discovery Servers for discovery purposes.

With `ROS2_EASY_MODE` a new Discovery Server will be automatically spawned locally in the given
[domain](../dds_layer/domain/domain.md#dds-layer-domain), pointing to another Discovery Server located in the specified IP.
If the specified IP belongs to the same host, it will only work in localhost, until another host connects to it.
If there exists a Discovery Server for that domain, the spawn process will be skipped, relying on the existing server
for discovery purposes.
Therefore, only one Discovery Server per host will be present in the domain.

In order for this variable to take effect, the participant must have its
[discovery protocol](../discovery/general_disc_settings.md#discovery-protocol) set to [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) (default), to automatically enter the
[Discovery Server Easy Mode](https://docs.vulcanexus.org/en/latest/rst/enhancements/easy_mode/easy_mode.html).
If this happens, the participant will be configured as a [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE) pointing to the local server.

The following example will configure participants as [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE) pointing to a local Discovery Server,
which will try to connect to another Discovery Server located in the host `10.0.0.1`.

```bash
export ROS2_EASY_MODE=10.0.0.1
```

The port of the Discovery Server is calculated using the rules explained in the [Well Known Ports](../transport/listening_locators.md#listening-locators-defaultports).
The transports configured in this new mode include [TCP](../transport/tcp/tcp.md#transport-tcp-tcp) unicast for discovery and
[TCP](../transport/tcp/tcp.md#transport-tcp-tcp) and [Shared Memory](../transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) for user data.

A detailed tutorial can be found in the
[Vucanexus Easy Mode Tutorial](https://docs.vulcanexus.org/en/latest/rst/tutorials/core/wifi/easy_mode/easy_mode.html) documentation.

#### NOTE
When `ROS2_EASY_MODE` is enabled, Fast DDS automatically loads a custom XML profile named `service`.
This profile increases the server’s response timeout for ROS 2 services by modifying the
[`max_blocking_time`](../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE).
However, if the user provides an XML file that already contains a profile with the same name, Fast DDS will not
load any extra profile.
Instead, the [`max_blocking_time`](../api_reference/dds_pim/core/policy/reliabilityqospolicy.md#_CPPv4N8eprosima7fastdds3dds20ReliabilityQosPolicy17max_blocking_timeE) value defined in the user’s XML file will be used.

#### WARNING
Discovery Server `ROS2_EASY_MODE` is not yet available for Windows platforms.

<a id="env-vars-ros-super-client"></a>

<a id="ros-super-client"></a>

## `ROS_SUPER_CLIENT`

If the `DomainParticipant`’s [discovery protocol](../discovery/general_disc_settings.md#discovery-protocol) is set to [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE),
and `ROS_SUPER_CLIENT` is set to TRUE, the participant is automatically promoted to a [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE).

#### IMPORTANT
This environment variable is meant to be used in combination with `ROS_DISCOVERY_SERVER` to promote a participant from [`SIMPLE`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol6SIMPLEE) to [`SUPER_CLIENT`](../api_reference/rtps/attributes/DiscoveryProtocol.md#_CPPv4N8eprosima7fastdds4rtps17DiscoveryProtocol12SUPER_CLIENTE).
The participant will have the *servers* list defined in `ROS_DISCOVERY_SERVER`.

The possible values are: **TRUE**, **true**, **True**, **1**, **FALSE**, **false**, **False**, **0**.

#### IMPORTANT
If the variable is not set, the default behavior of Fast DDS is equivalent to the case in which the variable is set to false.

The following example shows how to set the environment variable to true.

| **Linux**                                        |
|--------------------------------------------------|
| ```bash<br/>export ROS_SUPER_CLIENT=TRUE<br/>``` |
| **Windows**                                      |
| ```bash<br/>set ROS_SUPER_CLIENT=TRUE<br/>```    |

<a id="env-vars-fastdds-statistics"></a>

<a id="fastdds-statistics"></a>

## `FASTDDS_STATISTICS`

Setting this variable configures the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) to enable the statistics
DataWriters which topics are contained in the list set in this environment variable.
The elements of the list should be separated by semicolons and match the
[statistics topic name aliases](../statistics/dds_layer/topic_names.md#statistics-topic-names).

#### WARNING
The environment variable is only used in the case where the CMake option FASTDDS_STATISTICS has been enabled.
In any other case, the environment variable has no effect.
Please, refer to [CMake options](../../installation/configuration/cmake_options.md#cmake-options) for more information.
**[Pro]** In *Fast DDS* **[Pro]**, the statistics module is always compiled in, so the environment variable is always used.

For example, to enable the statistics DataWriters that report the throughput measurements, the environment variable
should be set as follows:

| **Linux**                                                                                                  |
|------------------------------------------------------------------------------------------------------------|
| ```bash<br/>export FASTDDS_STATISTICS="PUBLICATION_THROUGHPUT_TOPIC;SUBSCRIPTION_THROUGHPUT_TOPIC"<br/>``` |
| **Windows**                                                                                                |
| ```bash<br/>set FASTDDS_STATISTICS=PUBLICATION_THROUGHPUT_TOPIC;SUBSCRIPTION_THROUGHPUT_TOPIC<br/>```      |

#### IMPORTANT
This environment variable can be used together with the XML profiles
(for more information please refer to [Automatically enabling statistics DataWriters](../statistics/dds_layer/domainParticipant.md#auto-enabling-statistics-datawriters)).
The statistics DataWriters that will be enabled is the union between the ones specified in the XML file (if loaded)
and the ones stated in the environment variable (if set).

<a id="env-vars-fastdds-environment-file"></a>

<a id="fastdds-environment-file"></a>

## `FASTDDS_ENVIRONMENT_FILE`

Setting this environment variable to an existing `json` file allows to load the environment variables from the file
instead of from the environment.
This allows to change the value of some environment variables at run time with just modifying and saving the changes to
the file.
The environment value can be either an absolute or relative path.
The file format is as follows:

```JSON
{
    "environment_variable_name_1": "environment_variable_value_1",
    "environment_variable_name_2": "environment_variable_value_2"
}
```

#### IMPORTANT
The environment variables set in the environment file have precedence over the environment.

#### WARNING
Currently only `ROS_DISCOVERY_SERVER` environment variable allows for changes at run time. (see
[Modifying remote servers list at run time](../discovery/discovery_server.md#ds-modify-server-list))
