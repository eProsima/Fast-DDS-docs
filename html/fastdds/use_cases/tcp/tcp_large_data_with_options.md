<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-large-data-options"></a>

<a id="large-data-with-configuration-options"></a>

# Large Data with configuration options

As it has been observed in [Large Data Mode](tcp_with_multicast_discovery.md#use-case-tcp-multicast), `LARGE_DATA` builtin transports option offers an easy
and efficient way to improve performance when working with large data.
Nonetheless, custom configuration can help to enhance the performance even further.

Fast DDS provides configuration options to adjust the behavior of the builtin transports.
This becomes particularly relevant when using the `LARGE_DATA` mode, as it enables increasing the maximum message
size beyond 65500 KB and prevents fragmentation, leveraging the TCP and SHM transports.

All builtin transports can be configured by adjusting the following parameters:

+ `max_msg_size`: Message maximum size that can be sent over the transport.
  Sending messages larger than this size will result in fragmentation.
  Its maximum value is (2^32)-1 B for TCP and SHM and 65500 KB for UDP.
+ `sockets_size`: Size of the send and receive socket buffers.
  This value must be higher or equal than the `max_msg_size` to obtain a valid configuration.
  It also defines the size of the shared memory segment, calculated as twice the value set.
  Its maximum value is (2^32)-1 B.
  **[Pro]** If not set, the TCP socket buffers are auto-tuned by the operating system, enabling dynamic growth at runtime.
+ `non_blocking`: If set to true, the transport will use non-blocking sockets.
  This can be useful to avoid blocking the application if the socket buffers are full.
  However, some messages will be lost. Its default value is false.
  **[Pro]** If enabled without setting `sockets_size`, the TCP send buffer is not auto-tuned and the system default
  size is used instead.
+ `tcp_negotiation_timeout`: It specifies the timeout duration for logical port negotiation.
  This parameter is useful for ensuring the availability of the logical port before data transmission,
  thus preventing message loss during the negotiation process.
  Conversely, it can delay the discovery process.
  The default value is 0, implying that discovery will occur as soon as possible, but the initial messages
  might be lost if reliability is set to [`BEST_EFFORT_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE).
  This parameter is only valid for the `LARGE_DATA` mode.

Adjusting the maximum message size and the socket buffer sizes to a large enough value to accommodate the data to be
sent can help improve the performance with large messages, as well as setting the transport to non-blocking mode.
In *Fast DDS* **[Pro]**, leaving `sockets_size` unset lets the operating system auto-tune the TCP socket buffers,
which usually performs well in blocking mode, but it leaves the shared memory segment with its default size.
TCP auto tuning allows the socket buffers to grow dynamically at runtime until the value specified in
`net.ipv4.tcp_wmem` and `net.ipv4.tcp_rmem` is reached.

In this way, it is possible to take advantage of the TCP transport to avoid fragmentation and use the non-blocking
mode to avoid blocking the application when the socket buffers are full.
This configuration can be used, for example, when streaming video, which will result in a significant increase
in fluidity.

Note that even when using the `LARGE_DATA` mode within the same machine, the configuration options
can prove useful for improving performance, as they affect the SHM transport.
It is highly recommended to set a shared memory segment size large enough to accommodate the data to be sent.
To achieve this, the `sockets_size` parameter must be set to a value at least half of the message size.

The following snippets show how to configure the `LARGE_DATA` mode:

### Environment Variable

```bash
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA?max_msg_size=1MB&sockets_size=1MB&non_blocking=true&tcp_negotiation_timeout=50
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <profiles>
        <participant profile_name="large_data_builtin_transports_options" is_default_profile="true">
            <rtps>
                <builtinTransports max_msg_size="200KB" sockets_size="200KB" non_blocking="true" tcp_negotiation_timeout="50">LARGE_DATA</builtinTransports>
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
// Message Size and Sockets sizes of 200 KB + Non-blocking send + 50ms negotiation timeout
eprosima::fastdds::rtps::BuiltinTransportsOptions large_data_options;
large_data_options.maxMessageSize = 200000;
large_data_options.sockets_buffer_size = 200000;
large_data_options.non_blocking_send = true;
large_data_options.tcp_negotiation_timeout = 50;
pqos.setup_transports(eprosima::fastdds::rtps::BuiltinTransports::LARGE_DATA, large_data_options);

/* Create participant as usual */
eprosima::fastdds::dds::DomainParticipant* participant =
        eprosima::fastdds::dds::DomainParticipantFactory::get_instance()->create_participant(0, pqos);
```

#### NOTE
To learn how to check and modify the default maximum system value for the socket buffers size, please refer to
[Finding out system maximum values](../large_data/large_data.md#finding-out-maximum-socket-values).

#### WARNING
Setting a `max_msg_size` value higher than 65500 KB with the `DEFAULT` or `UDP` modes will result in an error
and the participant creation will fail.
