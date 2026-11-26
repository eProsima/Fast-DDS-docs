<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-largedata"></a>

<a id="large-data-rates"></a>

# Large Data Rates

When the amount of data exchanged between a [Publisher](../../dds_layer/publisher/publisher.md#dds-layer-publisher) and a [Subscriber](../../dds_layer/subscriber/subscriber.md#dds-layer-subscriber)
is large, some extra configuration may be required to compensate for side effects on the network and CPU load.
This large amount of data can be a result of the data types being large, a high message rate, or
a combination of both.

In this scenario, several approaches can be considered depending on the problem:

* For the cases in which the data samples are large (in the order of MB) such as transmitting raw video frames,
  point clouds, images, etc. between different hosts, TCP based communications may yield better reception rates
  with lower message loss, specially in the cases where a best effort transport layer is more susceptible to
  data loss, such as WiFi.
  To tackle these cases, [Large Data mode and Fast DDS over TCP](../tcp/tcp_use_case.md#use-case-tcp) documents several ways to configure Fast DDS to communicate over TCP.
* Network packages could be dropped because the transmitted amount of data fills the socket buffer
  before it can be processed.
  The solution is to [increase the buffers size](#tuning-socket-buffer).
  **[Pro]** On TCP transports, leaving the buffer sizes unconfigured lets the operating system auto-tune them.
  Auto tuning is the recommended configuration when using TCP transports.
* It is also possible to limit the rate at which the Publisher sends data using
  [Flow Controllers](#flow-controllers), in order to limit the effect of message bursts, and avoid to flood
  the Subscribers faster than they can process the messages.
* [Congestion Control](#congestion-control) **[Pro]** uses a special flow controller that adapts the data rate
  to the capabilities of the network and the Subscribers.
* On [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) mode,
  the overall message rate can be affected due to the retransmission of lost packets.
  Selecting the Heartbeat period allows to tune between increased meta traffic or faster response to lost packets.
  See [Tuning Heartbeat Period](#tuning-heartbeat-period).
* Also on [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) mode,
  with high message rates, the history of the [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter)
  can be filled up, blocking the publication of new messages.
  A [non-strict reliable mode](#tuning-nonstrict-reliability) can be configured to avoid this blocking,
  at the cost of potentially losing some messages on some of the Subscribers.

#### WARNING
*eProsima Fast DDS* defines a conservative default message size of 64kB,
which roughly corresponds to TCP and UDP payload sizes.
If the topic data is bigger, it will automatically be fragmented into several transport packets.

#### WARNING
The loss of a fragment means the loss of the entire message.
This has the most impact on [`BEST_EFFORT_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) mode, where the message loss
probability increases with the number of fragments

<a id="tuning-socket-buffer"></a>

<a id="increasing-socket-buffers-size"></a>

## Increasing socket buffers size

In high rate scenarios or large data scenarios, network packages can be dropped because
the transmitted amount of data fills the socket buffer before it can be processed.
Using [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) mode,
*Fast DDS* will try to recover lost samples, but with the penalty of
retransmission.
With [`BEST_EFFORT_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) mode,
samples will be definitely lost.

By default *eProsima Fast DDS* creates socket buffers with the system default size.
In *Fast DDS* **[Pro]**, this only applies to UDP transports (see the note below).
However, these sizes can be modified using the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos),
as shown in the example below.

#### NOTE
**[Pro]** On TCP transports, if no buffer size is configured, *Fast DDS* **[Pro]** does not set the size of the connection sockets,
so the operating system can dynamically adjust it according to the connection needs (auto-tuning).
Configuring a buffer size explicitly disables the auto-tuning of that buffer.
As [`non_blocking_send`](../../api_reference/transport/tcp_transport/tcp_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps22TCPTransportDescriptor17non_blocking_sendE) requires a fixed send buffer size, enabling it without
configuring the send buffer size disables the send buffer auto-tuning and the system default size is used instead.
This also applies to the TCP transport of the `LARGE_DATA` builtin transports and the *Easy Mode*.

### C++

```c++
DomainParticipantQos participant_qos;

// Increase the sending buffer size
participant_qos.transport().send_socket_buffer_size = 1048576;

// Increase the receiving buffer size
participant_qos.transport().listen_socket_buffer_size = 4194304;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_xml_profile_qos_socketbuffers">
        <rtps>
            <sendSocketBufferSize>1048576</sendSocketBufferSize>
            <listenSocketBufferSize>4194304</listenSocketBufferSize>
        </rtps>
    </participant>
</profiles>
```

<a id="finding-out-maximum-socket-values"></a>

<a id="finding-out-system-maximum-values"></a>

### Finding out system maximum values

Operating systems set a maximum value for socket buffer sizes.
If the buffer sizes are tuned with DomainParticipantQos, the values set
cannot exceed the maximum value of the system.

<a id="linux"></a>

#### Linux

The maximum buffer size values can be retrieved with the command `sysctl`.
For socket buffers used to send data, use the following command:

```bash
$> sudo sysctl -a | grep net.core.wmem_max
net.core.wmem_max = 1048576
```

For socket buffers used to receive data the command is:

```bash
$> sudo sysctl -a | grep net.core.rmem_max
net.core.rmem_max = 4194304
```

However, these maximum values are also configurable and can be increased if needed.
The following command increases the maximum buffer size of sending sockets:

```bash
$> sudo sysctl -w net.core.wmem_max=12582912
```

For receiving sockets, the command is:

```bash
$> sudo sysctl -w net.core.rmem_max=12582912
```

Linux also defines per-socket TCP buffer sizes as triplets:

```text
$> net.ipv4.tcp_wmem = <min> <default> <max> (TCP send buffer)
$> net.ipv4.tcp_rmem = <min> <default> <max> (TCP receive buffer)
```

The middle value is the initial size of each connection buffer, and the last one the maximum size it can reach.
When the TCP buffer sizes are left unconfigured in *Fast DDS* **[Pro]**, the auto-tuning grows each socket buffer from the
default up to the maximum value of these triplets as needed.
Auto-tuned sockets are not limited by `net.core.wmem_max` and `net.core.rmem_max`, which only limit explicitly
configured sizes.
The following command sets both the default and the maximum values for sending sockets:

```bash
$> sudo sysctl -w net.ipv4.tcp_wmem="4096 12582912 12582912"
```

For receiving sockets, the command is:

```bash
$> sudo sysctl -w net.ipv4.tcp_rmem="4096 12582912 12582912"
```

<a id="windows"></a>

#### Windows

The following command changes the maximum buffer size of sending sockets:

```default
C:\> reg add HKLM\SYSTEM\CurrentControlSet\services\AFD\Parameters /v DefaultSendWindow /t REG_DWORD /d 12582912
```

For receiving sockets, the command is:

```default
C:\> reg add HKLM\SYSTEM\CurrentControlSet\services\AFD\Parameters /v DefaultReceiveWindow /t REG_DWORD /d 12582912
```

<a id="increasing-the-transmit-queue-length-of-an-interface-linux-only"></a>

## Increasing the Transmit Queue Length of an interface (Linux only)

The Transmit Queue Length (`txqueuelen`) is a TCP/UDP/IP stack network interface value.
This value sets the number of packets allowed per kernel transmit queue of a network interface device.
By default, the `txqueuelen` value for Ethernet interfaces is set to `1000` in Linux.
This value is adequate for most Gigabit network devices.
However, in some specific cases, the `txqueuelen` setting should be increased to avoid overflows that drop packets.
Similarly, choosing a value that is too large can cause added overhead resulting in higher network latencies.

Note that this information only applies to the sending side, and not the receiving side.
Also increasing the `txqueuelen` should go together with increasing the buffer sizes of the UDP and/or TCP buffers.
(this must be applied for both the sending and receiving sides).

The settings for a specific network adapter can be viewed using the one of the following commands:

### `ip`

```bash
ip link show ${interface}
```

### `ifconfig`

```bash
ifconfig ${interface}
```

This will display the configuration of the adapter, and among the parameters the `txqueuelen`.
This parameter can be a value between 1000 and 20000.

#### IMPORTANT
If the `ip` command is used, the Transmit Queue Length parameter is called `qlen`.

The `txqueuelen` can be modified for the current session using either the `ifconfig` or `ip` commands.
However, take into account that after rebooting the default values will be configured again.

### `ip`

```bash
ip link set txqueuelen ${value} dev ${interface}
```

### `ifconfig`

```bash
ifconfig ${interface} txqueuelen ${size}
```

<a id="flow-controllers"></a>

<a id="id1"></a>

## Flow Controllers

*eProsima Fast DDS* provides a mechanism to limit the rate at which the data is sent by a DataWriter.
These controllers should be registered on the creation of the DomainParticipant using [FlowControllersQos](../../dds_layer/core/policy/eprosimaExtensions.md#flowcontrollersqos), and
then referenced by the DataWriter whose traffic is to be controlled:

* User DataWriters:
  The flow controller should be assigned through [PublishModeQosPolicy](../../dds_layer/core/policy/eprosimaExtensions.md#publishmodeqospolicy) to regulate the transmission of application
  data.
* Built-in DataWriters:
  The flow controller can be configured to control meta-traffic generated by built-in DataWriters responsible for
  PDP and EDP (`DiscoveryProtocol`); WLP ([LivelinessQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#livelinessqospolicy)), and TypeLookupService
  ([TypeObject representation](../../xtypes/discovery_matching.md#xtypes-discovery-matching)). This allows for fine-grained control over discovery and built-in traffic. The flow
  controller should be assigned through [WireProtocolConfigQos](../../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos).

A new thread is spawned the first time a flow controller is referenced by an asynchronous DataWriter.
This thread will be responsible for arbitrating the network output of the samples being transmitted by all the
DataWriters referencing the same flow controller.

Flow controllers should be given a name so they can later on be referenced by the DataWriters.
A default, unlimited, [`FIFO`](../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy4FIFOE) flow controller is always available with name
[`FASTDDS_FLOW_CONTROLLER_DEFAULT`](../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps31FASTDDS_FLOW_CONTROLLER_DEFAULTE).

<a id="scheduling-policy"></a>

### Scheduling policy

There are different kinds of flow controllers, depending on the scheduling policy used.
All of them will limit the number of bytes sent to the network to no more than
[`max_bytes_per_period`](../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor20max_bytes_per_periodE) bytes during [`period_ms`](../../api_reference/rtps/flowcontrol/FlowControllerDescriptor.md#_CPPv4N8eprosima7fastdds4rtps24FlowControllerDescriptor9period_msE)
milliseconds.
They only differ in the way they decide the order in which the samples are sent.

* [`FIFO`](../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy4FIFOE) will output samples on a first come, first served order.
* [`ROUND_ROBIN`](../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy11ROUND_ROBINE) will output one sample from each DataWriter in circular order.
* [`HIGH_PRIORITY`](../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy13HIGH_PRIORITYE) will output samples from DataWriters with the highest priority first.
  The priority of a DataWriter is configured using property `fastdds.sfc.priority`.
  Allowed values are from -10 (highest priority) to 10 (lowest priority).
  If the property is not present, it will be set to the lowest priority.
  Samples for DataWriters with the same priority are handled with FIFO order.
* [`PRIORITY_WITH_RESERVATION`](../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy25PRIORITY_WITH_RESERVATIONE) works as the previous one, but allows the DataWriters to reserve
  part of the output bandwidth.
  This is done with the property `fastdds.sfc.bandwidth_reservation`.
  Allowed values are from 0 to 100, and express a percentage of the total flow controller limit.
  If the property is not present, it will be set to 0 (no bandwidth is reserved for the DataWriter).
  After the reserved bandwidth has been consumed, the rest of the samples will be handled with the rules of
  [`HIGH_PRIORITY`](../../api_reference/rtps/flowcontrol/FlowControllerSchedulerPolicy.md#_CPPv4N8eprosima7fastdds4rtps29FlowControllerSchedulerPolicy13HIGH_PRIORITYE).

<a id="example-configuration"></a>

### Example configuration

### C++

```c++
// Limit to 300kb per second.
static const char* flow_controller_name = "example_flow_controller";
auto flow_control_300k_per_sec = std::make_shared<eprosima::fastdds::rtps::FlowControllerDescriptor>();
flow_control_300k_per_sec->name = flow_controller_name;
flow_control_300k_per_sec->scheduler = eprosima::fastdds::rtps::FlowControllerSchedulerPolicy::FIFO;
flow_control_300k_per_sec->max_bytes_per_period = 300 * 1000;
flow_control_300k_per_sec->period_ms = 1000;

// [OPTIONAL] Configure sender thread settings
flow_control_300k_per_sec->sender_thread = eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1};

// Register flow controller on participant
DomainParticipantQos participant_qos;
participant_qos.flow_controllers().push_back(flow_control_300k_per_sec);

// [OPTIONAL] Link builtin writers to any of the registered flow controllers
participant_qos.wire_protocol().builtin.flow_controller_name = "example_flow_controller";

// .... create participant and publisher

// Link writer to the registered flow controller.
// Note that ASYNCHRONOUS_PUBLISH_MODE must be used
DataWriterQos qos;
qos.publish_mode().kind = ASYNCHRONOUS_PUBLISH_MODE;
qos.publish_mode().flow_controller_name = flow_controller_name;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_profile_qos_flowcontroller">
        <rtps>
            <flow_controller_descriptor_list>
                <flow_controller_descriptor>
                    <name>example_flow_controller</name>
                    <scheduler>FIFO</scheduler>
                    <max_bytes_per_period>4096</max_bytes_per_period>
                    <period_ms>500</period_ms>
                    <sender_thread>
                        <scheduling_policy>-1</scheduling_policy>
                        <priority>0</priority>
                        <affinity>0</affinity>
                        <stack_size>-1</stack_size>
                    </sender_thread>
                </flow_controller_descriptor>
            </flow_controller_descriptor_list>

            <builtin> <!-- Optional use of flow controller for participant's meta traffic-->
                <flow_controller_name>example_flow_controller</flow_controller_name>
            </builtin>
        </rtps>
    </participant>

    <data_writer profile_name="writer_profile_qos_flowcontroller">
        <qos>
            <publishMode>
                <kind>ASYNCHRONOUS</kind>
                <flow_controller_name>example_flow_controller</flow_controller_name>
            </publishMode>
        </qos>
    </data_writer>
</profiles>
```

#### WARNING
Specifying a flow controller with a size smaller than the transport buffer size
can cause the messages to never be sent.

<a id="congestion-control"></a>

<a id="congestion-control-pro"></a>

## Congestion Control **[Pro]**

When transmitting large amounts of data over a network, it is crucial to manage congestion to ensure optimal
performance and prevent packet loss.
Fast DDS **[Pro]** allows enabling a congestion control mechanism on a participant, which makes every DataWriter use
a different bandwidth limitation on each DataReader according to the network conditions.

This feature can be customized through [congestion control plugins](#congestion-control-plugins), which define
the algorithm used to adjust the bandwidth based on the detected congestion.

To enable congestion control, the DomainParticipant must be configured with the
[appropriate properties](../../property_policies/congestion_control.md#property-policies-congestion-control).

<a id="congestion-control-plugins"></a>

<a id="congestion-control-plugins-pro"></a>

### Congestion Control Plugins **[Pro]**

The congestion control mechanism in Fast DDS **[Pro]** can be customized through plugins that define the algorithm used to
adjust the bandwidth associated to each DataReader.
The following plugins are available:

- The [basic](#basic-congestion-control-plugin) plugin, a lightweight increase/decrease controller.
- The [reno](#reno-congestion-control-plugin) plugin, a TCP Reno/NewReno-like controller for
  bandwidth-constrained or lossy paths.

The plugin is selected with the `fastdds.congestion.plugin` property (see
[here](../../property_policies/congestion_control.md#property-policies-congestion-control)).

With either plugin, the DataWriter property `fastdds.sfc.bandwidth_reservation` sets the share of the sent
traffic guaranteed to retransmissions (default `30` percent), so that recovering lost samples is not starved by a
steady stream of new samples.
This share does not change when congestion control lowers the bandwidth limit, and it is not a hard partition: if
no retransmissions are pending, new samples can use the whole bandwidth, and vice versa.
When several DataWriters share the same congestion control, the highest requested value applies.

<a id="basic-congestion-control-plugin"></a>

<a id="the-basic-congestion-control"></a>

#### The `basic` Congestion Control

The `basic` congestion control plugin dynamically adjusts the sending rate to each
remote DataReader based on observed network feedback.
It operates independently per remote DataReader and only applies to **reliable**, non-builtin DataWriters.
Best-effort writers are unaffected.

#### NOTE
If a DataWriter already has a custom flow controller configured, it will be **overridden** by the
congestion control.

The `basic` congestion control is governed by the following properties, further detailed
[here](../../property_policies/congestion_control.md#property-policies-congestion-control):

- `fastdds.congestion.period`: interval at which the bandwidth limit of each DataReader is evaluated.
- `fastdds.congestion.initial_bandwidth`: starting limit, in bytes per second, for each newly discovered
  DataReader.
- `fastdds.congestion.increase_multiplier`: factor applied to the limit when it is increased.
- `fastdds.congestion.decrease_multiplier`: factor applied to the limit when it is decreased.
- `fastdds.congestion.maximum_bandwidth`: hard upper cap, in bytes per second, on the limit.

At the end of each period, the algorithm evaluates each tracked remote DataReader
independently resulting in three possible cases:

- **Decrease**: If the DataWriter had to retransmit data to the reader during the period
  (signaling packet loss or network congestion), the current bandwidth limit is multiplied
  by `decrease_multiplier`.
  Note that the first repair attempt for a given sequence number is not counted to avoid
  overreacting to isolated packet losses.

  #### NOTE
  The bandwidth limit has a lower bound equal to the participant’s maximum datagram size,
  ensuring that at least one message can always be sent per period regardless of how many
  times the limit is decreased.
- **Increase**: If no retransmissions occurred and the reader’s acknowledged throughput
  exceeded 80% of the current bandwidth limit, the limit is multiplied by `increase_multiplier`.
  The 80% threshold allows the limit to grow before full saturation, preserving some space to
  absorb traffic bursts.
- **No change**: If neither condition is met, the bandwidth limit remains unchanged.

<a id="reno-congestion-control-plugin"></a>

<a id="the-reno-congestion-control"></a>

#### The `reno` Congestion Control

The `reno` congestion control plugin adapts the per-DataReader sending rate using a control law modelled on
TCP Reno / NewReno.
It is designed to keep a reliable DataWriter from overrunning a bandwidth-constrained or lossy path (which would
otherwise collapse into a retransmission storm), while ramping the rate back up when capacity returns.
As with the `basic` plugin, it operates independently per remote DataReader and only applies to **reliable**,
non-builtin DataWriters.

It is a rate-based controller: once per `period` it evaluates the delivery feedback of each remote DataReader
(data sent, acknowledged and retransmitted) and adjusts the bandwidth limit of that DataReader.
Its behavior follows TCP Reno:

- While no congestion is detected and the DataWriter is actually using its current budget (see
  `cap_binding_ratio`), the limit grows: first quickly (`slow_start_factor`) until `initial_ss_threshold` is
  reached, and then gently (`additive_increment` per period).
  A DataWriter that does not fill its budget keeps its current limit instead of ramping up.
- Congestion is detected from an increase in retransmissions together with a drop in delivered throughput (see
  `repair_ratio_threshold` and `worsening_margin`).
  A burst of retransmissions caused only by draining a backlog is not treated as congestion.
  On congestion, the limit is reduced once by `fast_recovery_factor` and then recovers gradually.
- If the DataWriter keeps sending but nothing is acknowledged or retransmitted for `ack_progress_timeout_periods`
  consecutive periods, the path is considered stalled.
  The limit is then cut harder (`timeout_recovery_factor`), and the controller starts probing again from the
  fast-growth phase.

As with the `basic` plugin, the first retransmission of a sample is ignored, so isolated losses do not cause a
reduction.
The limit never exceeds `maximum_bandwidth` and never drops below the size of one datagram, so at least one
message can always be sent per period.

<a id="configuration"></a>

##### Configuration

The `reno` plugin is governed by the `fastdds.congestion.*` properties described
[here](../../property_policies/congestion_control.md#property-policies-congestion-control).
Beyond the shared `plugin`, `period`, `initial_bandwidth` and `maximum_bandwidth` properties, it accepts the
algorithm-specific properties `initial_ss_threshold`, `slow_start_factor`, `additive_increment`,
`fast_recovery_factor`, `timeout_recovery_factor`, `repair_ratio_threshold`, `ack_progress_timeout_periods`,
`cap_binding_ratio` and `worsening_margin`.

<a id="tuning-heartbeat-period"></a>

<a id="id2"></a>

## Tuning Heartbeat Period

On [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE) ([ReliabilityQosPolicy](../../dds_layer/core/policy/standardQosPolicies.md#reliabilityqospolicy)), RTPS protocol can detect which messages have been lost
and retransmit them.
This mechanism is based on meta-traffic information exchanged between
DataWriters and DataReaders,
namely, Heartbeat and Ack/Nack messages.

A smaller Heartbeat period increases the CPU and network overhead, but speeds up the system response when
a piece of data is lost.
Therefore, users can customize the Heartbeat period to match their needs.
This can be done with the DataWriterQos.

```c++
DataWriterQos qos;
qos.reliable_writer_qos().times.heartbeat_period.seconds = 0;
qos.reliable_writer_qos().times.heartbeat_period.nanosec = 500000000;     //500 ms
```

<a id="tuning-nonstrict-reliability"></a>

<a id="using-non-strict-reliability"></a>

## Using Non-strict Reliability

When [HistoryQosPolicyKind](../../dds_layer/core/policy/standardQosPolicies.md#historyqospolicykind) is set as [`KEEP_ALL_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE), all samples have to be received
(and acknowledged) by all subscribers before they can be overridden by the DataWriter.
If the message rate is high and the network is not reliable (i.e., lots of packets get lost), the history of the
DataWriter can be filled up, blocking the publication of new messages until any
of the old messages is acknowledged by all subscribers.

If this strictness is not needed, [HistoryQosPolicyKind](../../dds_layer/core/policy/standardQosPolicies.md#historyqospolicykind) can be set as [`KEEP_LAST_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind21KEEP_LAST_HISTORY_QOSE).
In this case, when the history of the DataWriter is full, the oldest message that has not
been fully acknowledged yet is overridden with the new one.
If any subscriber did not receive the discarded message, the publisher
will send a GAP message to inform the subscriber that the message is lost forever.

<a id="practical-examples"></a>

## Practical Examples

<a id="use-case-largedata-examplefile"></a>

<a id="example-sending-a-large-file"></a>

### Example: Sending a large file

Consider the following scenario:

* A Publisher needs to send a file with a size of 9.9 MB.
* The Publisher and Subscriber are connected through a network
  with a bandwidth of 100 MB/s

With a fragment size of 64 kB, the Publisher has to send about 1100 fragments to send the whole file.
A possible configuration for this scenario could be:

* Using [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE),
  since a losing a single fragment would mean the loss of the complete file.
* Decreasing the heartbeat period, in order to increase the reactivity of the Publisher.
* Limiting the data rate using a [Flow Controller](#flow-controllers),
  to avoid this transmission cannibalizing the whole bandwidth.
  A reasonable rate for this application could be 5 MB/s, which represents only 5% of the total bandwidth.

#### NOTE
Using [Shared Memory Transport](../../transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) the only limit to the fragment size is the available memory.
Therefore, all fragmentation can be avoided in SHM by increasing the size of the shared buffers.

<a id="use-case-largedata-examplestreaming"></a>

<a id="example-video-streaming"></a>

### Example: Video streaming

In this scenario, the application transmits a video stream between a Publisher
and a Subscriber, at 50 fps. In real-time audio or video transmissions,
it is usually preferred to have a high stable datarate feed, even at the cost of losing some
samples.
Losing one or two samples per second at 50 fps is more acceptable than freezing the video waiting for the retransmission
of lost samples.
Therefore, in this case [`BEST_EFFORT_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind27BEST_EFFORT_RELIABILITY_QOSE) can be appropriate.
