<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-sharedmemory-sharedmemory"></a>

<a id="shared-memory-transport"></a>

# Shared Memory Transport

The shared memory (SHM) transport enables fast communications between entities running in the same
processing unit/machine, relying on the shared memory mechanisms provided by the host operating system.

#### NOTE
Fast DDS utilizes the [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)’s [`GuidPrefix_t`](../../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) to identify peers running in the same host.
Two participants with identical 4 first bytes on the [`GuidPrefix_t`](../../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) are considered to be running in the same
host.
[`is_on_same_host_as()`](../../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4NK8eprosima7fastdds4rtps6GUID_t18is_on_same_host_asERK6GUID_t) API is provided to check this condition.
Please, take also into account the caveats included in [GUID Prefix considerations for intra-process delivery](../intraprocess.md#intraprocess-delivery-guids).

SHM transport provides better performance than other network transports like UDP / TCP,
even when these transports use loopback interface.
This is mainly due to the following reasons:

* Large message support: Network protocols need to fragment data in order to comply with the specific protocol and
  network stacks requirements, increasing communication overhead.
  SHM transport allows the copy of full messages where the only size limit is the machine’s memory capacity.
* Reduce the number of memory copies: When sending the same message to different endpoints, SHM transport can
  directly share the same memory buffer with all the destination endpoints.
  Other protocols require to perform one copy of the message per endpoint.
* Less operating system overhead: Once initial setup is completed, shared memory transfers require much less system
  calls than the other protocols.
  Therefore, there is a performance/time consume gain by using SHM.

<a id="transport-sharedmemory-concepts"></a>

<a id="definition-of-concepts"></a>

## Definition of Concepts

This section describes basic concepts to help explain how the Shared Memory Transport works in order
to deliver the data messages to the appropriate [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).
The purpose is not to be an exhaustive reference of the implementation, but to be a comprehensive explanation
of each concept, so that users can configure the transport to their needs.

Many of the descriptions in this section will be made following the example use case depicted in the following figure,
where *Participant 1* sends a data message to *Participant 2*.
Please, refer to the figure when following the definitions.

![image](01-figures/shm_comm_sequence_diagram.svg)

*Sequence diagram for Shared Memory Transport*

<a id="transport-sharedmemory-concepts-segment"></a>

<a id="segment"></a>

### Segment

A *Segment* is a block of shared memory that can be accessed from different processes.
Every DomainParticipant that has been configured with Shared Memory Transport
creates a segment of shared memory.
The DomainParticipant writes to this segment any data it needs to deliver to other
DomainParticipants, and the remote
DomainParticipants are able to read it directly using the
shared memory mechanisms.

#### NOTE
Launching any of the processes with a higher privileged user (for instance, *root*)
can lead to communication problems, as processes run by non-privileged users may
not be able to write into the memory segment.

Every segment has a *segmentId*, a 16-character UUID that uniquely identifies each shared memory segment.
These *segmentIds* are used to identify and access the segment of each DomainParticipant.

<a id="transport-sharedmemory-concepts-buffer"></a>

<a id="segment-buffer"></a>

### Segment Buffer

A buffer allocated in the shared memory Segment.
It works as a container for a DDS message that is placed in the Segment.
In other words, each message that the DomainParticipant writes on the
Segment will be placed in a different buffer.

<a id="transport-sharedmemory-concepts-bufferdescriptor"></a>

<a id="buffer-descriptor"></a>

### Buffer Descriptor

It acts as a pointer to a specific Segment Buffer
in a specific Segment.
It contains the *segmentId* and the offset of the Segment Buffer from the base of the
Segment.
When communicating a message to other DomainParticipants,
Shared Memory Transport only distributes the Buffer Descriptor, avoiding the copy of
the message from a DomainParticipant to another.
With this descriptor, the receiving DomainParticipant can access the message written in the buffer,
as is uniquely identifies the Segment (through the *segmentId*)
and the Segment Buffer (through its offset).

<a id="transport-sharedmemory-concepts-port"></a>

<a id="port"></a>

### Port

Represents a channel to communicate Buffer Descriptors.
It is implemented as a ring-buffer in shared memory, so that any DomainParticipant
can potentially read or write information on it.
Each port has a unique identifier, a 32 bit number that can be used to refer to the port.
Every DomainParticipant that has been configured with Shared Memory Transport
creates a port to receive Buffer Descriptors.
The identifier of this port is shared during the [Discovery](../../discovery/discovery.md#discovery), so that remote peers know which port to use
when they want to communicate with each DomainParticipant.

DomainParticipants create a listener to their receiving port,
so that they can be notified when a new Buffer Descriptor is pushed to the port.

<a id="transport-sharedmemory-concepts-porthealthcheck"></a>

<a id="port-health-check"></a>

### Port Health Check

Every time a DomainParticipant opens a Port
(for reading or writing), a health check is performed to assess its correctness.
The reason is that if one of the processes involved crashes while using a Port,
that port can be left inoperative.
If the attached listeners do not respond in a given timeout, the Port
is considered damaged, and it is destroyed and created again.

<a id="transport-sharedmemory-transportdescriptor"></a>

<a id="sharedmemtransportdescriptor"></a>

## SharedMemTransportDescriptor

In addition to the data members defined in the [TransportDescriptorInterface](../transport_api.md#transport-transportapi-transportdescriptor),
the TransportDescriptor for Shared Memory defines the following ones:

| Member                                                                                                                                                                                                                 | Data type                                                                             | Default    | Accessor / Mutator                                                                                                                                                                                                     | Description                                                                                                                        |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------------------------------------|------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------|
| `segment_size_`                                                                                                                                                                                                        | `uint32_t`                                                                            | `512*1024` | [`segment_size()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor12segment_sizeEv)                                        | Size of the shared memory segment<br/>(in octets).                                                                                 |
| `port_queue_capacity_`                                                                                                                                                                                                 | `uint32_t`                                                                            | `512`      | [`port_queue_capacity()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor19port_queue_capacityEv)                          | The size of the listening port<br/>(in messages).                                                                                  |
| `healthy_check_timeout_ms_`                                                                                                                                                                                            | `uint32_t`                                                                            | `1000`     | [`healthy_check_timeout_ms()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor24healthy_check_timeout_msEv)                | Timeout for the health check of ports<br/>(in milliseconds).                                                                       |
| `rtps_dump_file_`                                                                                                                                                                                                      | `string`                                                                              | `""`       | [`rtps_dump_file()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor14rtps_dump_fileEv)                                    | Full path of the protocol dump file.                                                                                               |
| [`default_reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor25default_reception_threadsEv) | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |            | [`default_reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor25default_reception_threadsEv) | Default [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads.           |
| [`reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor17reception_threadsEv)                 | `std::map<uint32_t, ThreadSettings>`                                                  |            | [`reception_threads`](../../api_reference/transport/transport_generic_interfaces/port_based_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28PortBasedTransportDescriptor17reception_threadsEv)                 | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the reception threads on specific ports. |
| [`dump_thread()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor11dump_threadEv)                                          | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) |            | [`dump_thread()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor11dump_threadEv)                                          | [ThreadSettings](../../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos) for the SHM dump thread.                     |

If `rtps_dump_file_` is not empty, all the shared memory traffic on the DomainParticipant
(sent and received) is traced to a file.
The output file format is *tcpdump* hexadecimal text, and can be processed with protocol analyzer applications
such as Wireshark.
Specifically, to open the file using Wireshark, use the “Import from Hex Dump” option using the “Raw IPv4” encapsulation
type.

#### NOTE
The [`kind`](../../api_reference/transport/transport_generic_interfaces/transport_interface.md#_CPPv4NK8eprosima7fastdds4rtps18TransportInterface4kindEv) value for a [`SharedMemTransportDescriptor`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4N8eprosima7fastdds4rtps28SharedMemTransportDescriptorE) is given by the value
[`LOCATOR_KIND_SHM`](../../api_reference/rtps/common/Locator/defines.md#c.LOCATOR_KIND_SHM).

#### WARNING
Setting a [`segment_size()`](../../api_reference/transport/shm_transport/shared_mem_transport_descriptor.md#_CPPv4NK8eprosima7fastdds4rtps28SharedMemTransportDescriptor12segment_sizeEv) close to or smaller than the data size poses a high risk
of data loss, since the write operation will overwrite the buffer during a single send operation.

<a id="transport-sharedmemory-enabling"></a>

<a id="enabling-shared-memory-transport"></a>

## Enabling Shared Memory Transport

*Fast DDS* enables a SHM transport by default.
Nevertheless, the application can enable other SHM transports if needed.
To enable a new SHM transport in a [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), first
create an instance of [SharedMemTransportDescriptor](#transport-sharedmemory-transportdescriptor),
and add it to the user transport list of the [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

The examples below show this procedure in both C++ code and XML file.

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
std::shared_ptr<SharedMemTransportDescriptor> shm_transport =
        std::make_shared<SharedMemTransportDescriptor>();

// [OPTIONAL] ThreadSettings configuration
shm_transport->default_reception_threads(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
shm_transport->set_thread_config_for_port(12345, eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});
shm_transport->dump_thread(eprosima::fastdds::rtps::ThreadSettings{-1, 0, 0, -1});

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(shm_transport);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <transport_descriptors>
            <!-- Create a descriptor for the new transport -->
            <transport_descriptor>
                <transport_id>shm_transport</transport_id>
                <type>SHM</type>
                <default_reception_threads> <!-- OPTIONAL -->
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </default_reception_threads>
                <reception_threads> <!-- OPTIONAL -->
                    <reception_thread port="12345">
                        <scheduling_policy>-1</scheduling_policy>
                        <priority>0</priority>
                        <affinity>0</affinity>
                        <stack_size>-1</stack_size>
                    </reception_thread>
                </reception_threads>
                <dump_thread> <!-- OPTIONAL -->
                    <scheduling_policy>-1</scheduling_policy>
                    <priority>0</priority>
                    <affinity>0</affinity>
                    <stack_size>-1</stack_size>
                </dump_thread>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="SHMParticipant">
            <rtps>
                <!-- Link the Transport Layer to the Participant -->
                <userTransports>
                    <transport_id>shm_transport</transport_id>
                </userTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

#### NOTE
In case that several transports are enabled, the discovery traffic is always performed using the UDP/TCP transport,
even if the SHM transport is enabled in both participants running in the same machine.
This may cause discovery issues if one or several of the participants only has SHM enabled and other participants use
some other transport at the same time.
Also, when two participants on the same machine have SHM transport enabled, the user data communication between them
is automatically performed by SHM transport only.
The rest of the enabled transports are not used between those two participants.

#### HINT
To configure discovery traffic through Shared Memory, the default builtin transports must be disabled.
In that way, communication is performed completely using Shared Memory.
The snippet examples below show this procedure in both C++ code and XML file.
See [Delivery Mechanisms example](#transport-sharedmemory-example) for a complete example.

### C++

```c++
DomainParticipantQos qos;

// Create a descriptor for the new transport.
std::shared_ptr<SharedMemTransportDescriptor> shm_transport =
        std::make_shared<SharedMemTransportDescriptor>();

// Link the Transport Layer to the Participant.
qos.transport().user_transports.push_back(shm_transport);

// Explicit configuration of SharedMem transport
qos.transport().use_builtin_transports = false;
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <transport_descriptors>
        <!-- Create a descriptor for the new transport -->
        <transport_descriptor>
            <transport_id>shm_transport_only</transport_id>
            <type>SHM</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="DisableBuiltinTransportsParticipant">
        <rtps>
            <!-- Link the Transport Layer to the Participant -->
            <userTransports>
                <transport_id>shm_transport_only</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
```

<a id="transport-sharedmemory-example"></a>

<a id="delivery-mechanisms-example"></a>

## Delivery Mechanisms example

A hello world example suitable for supported delivery mechanisms can be found in the
[delivery_mechanisms folder](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/delivery_mechanisms).
It shows a publisher and a subscriber that communicate through the desired delivery mechanism (which can be set to
shared memory only).
