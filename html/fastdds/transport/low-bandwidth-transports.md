<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="low-bandwidth-transports"></a>

<a id="low-bandwidth-transports-pro"></a>

# Low Bandwidth Transports **[Pro]**

The Low Bandwidth Transports feature in *Fast DDS Pro* enables efficient communication over constrained networks by
reducing the size of network packets.
This optimization is particularly beneficial for environments with limited bandwidth, such as satellite links or
tactical radios, allowing Fast DDS applications to operate reliably and efficiently in such scenarios.

This feature provides two specialized transports:

- **Payload Compression Transport**: Compresses the message payload before transmitting it over the network.
  By reducing the amount of data sent, it minimizes bandwidth usage and can improve throughput and reduce latency on
  bandwidth-constrained links.
- **Header Reduction Transport**: Minimizes the size of the RTPS protocol headers in each packet.
  By reducing header overhead, it further optimizes total packet size and ensures efficient communication,
  especially for small and frequent messages.

Both transports are implemented using the feature [Chaining of transports](transport_api.md#transport-transportapi-chaining), therefore both can be
configured independently or together, providing flexibility to adapt Fast DDS to the requirements of low bandwidth
environments.

<a id="payload-compression-transport"></a>

## Payload Compression Transport

This low-bandwidth transport performs a standard compression of data before sending and the corresponding decompression
after receiving.
The compression algorithm may be selected between Zlib and Bzip2, or even let the transport perform both and use the one
which produces the shortest output.

This transport can be configured using the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) [`properties()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`rtps.payload_compression`.

| Property name                        | Property value                                                                              |
|--------------------------------------|---------------------------------------------------------------------------------------------|
| `compression_library`                | Default compression library for all packets. <br/><br/>**Values**: ZLIB, BZIP2 or AUTOMATIC |
| `compression_level`                  | Default compression level for all packets. <br/><br/>**Values**: 1 to 9                     |
| `compression_library.small_packets`  | Compression library for small packets. <br/><br/>**Values**: ZLIB, BZIP2 or AUTOMATIC       |
| `compression_level.small_packets`    | Compression level for small packets. <br/><br/>**Values**: 1 to 9                           |
| `compression_library.medium_packets` | Compression library for medium packets. <br/><br/>**Values**: ZLIB, BZIP2 or AUTOMATIC      |
| `compression_level.medium_packets`   | Compression level for medium packets. <br/><br/>**Values**: 1 to 9                          |
| `compression_library.large_packets`  | Compression library for large packets. <br/><br/>**Values**: ZLIB, BZIP2 or AUTOMATIC       |
| `compression_level.large_packets`    | Compression level for large packets. <br/><br/>**Values**: 1 to 9                           |
| `low_mark`                           | Maximum size for a packet to be considered small.                                           |
| `high_mark`                          | Minimum size for a packet to be considered large.                                           |

<a id="transport-transportapi-payloadcompression-descriptor"></a>

<a id="payloadcompressiontransportdescriptor"></a>

### PayloadCompressionTransportDescriptor

`PayloadCompressionTransportDescriptor` has no additional data members from the common ones described in
[ChainingTransportDescriptor](transport_api.md#transport-transportapi-chaining-descriptor).

<a id="enabling-payload-compression-transport"></a>

### Enabling Payload Compression Transport

To enable a new `PayloadCompressionTransport` in a [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), first
create an instance of [PayloadCompressionTransportDescriptor](#transport-transportapi-payloadcompression-descriptor),
and add it to the user transport list of the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

The examples below show this procedure in both C++ code and XML file.

### C++

```c++
DomainParticipantQos participant_qos;

auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

// Create a descriptor for the new transport.
auto compression_transport =
        std::make_shared<PayloadCompressionTransportDescriptor>(udp_transport);

// [OPTIONAL] Transport configuration
participant_qos.properties().properties().emplace_back(Property(
            "rtps.payload_compression.compression_library",
            "AUTOMATIC"));

// Link the Transport Layer to the Participant.
participant_qos.transport().use_builtin_transports = false;
participant_qos.transport().user_transports.push_back(compression_transport);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>compression_transport</transport_id>
                <type>PAYLOAD_COMPRESSION</type>
                <low_level_transport>udp_transport</low_level_transport>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="PayloadCompressionParticipant">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>rtps.payload_compression.compression_library</name>
                            <value>AUTOMATIC</value>
                        </property>
                    </properties>
                </propertiesPolicy>
                <userTransports>
                    <transport_id>compression_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="header-reduction-transport"></a>

## Header Reduction Transport

This transport performs a specific compression of data before sending and the corresponding decompression after
receiving.
The compression algorithm is specific for the RTPS protocol.
It will remove certain headers while compressing others.

This transport can be configured using the [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) [`properties()`](../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos10propertiesEv)
`rtps.header_reduction`.

| Property name                         | Property value                                                                                                                                                                                                                                                                                                    |
|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `remove_protocol`                     | Removes the ProtocolId, which identifies the message as an RTPS message, from the RTPS header. <br/><br/>**Values**: true or false                                                                                                                                                                                |
| `remove_version`                      | Removes the ProtocolVersion, which identifies the version of the RTPS protocol, from the RTPS header. <br/><br/>**Values**: true or false                                                                                                                                                                         |
| `remove_vendor_id`                    | Removes the VendorId, which indicates the vendor that provides the implementation of the RTPS<br/>protocol, from the RTPS header. <br/><br/>**Values**: true or false                                                                                                                                             |
| `compress_guid_prefix`                | Compresses the GuidPrefix, which is a unique identifier for each participant in the RTPS<br/>protocol, from three 32-bit words to a smaller size. <br/><br/>**Values**: Three numbers between 0 and 32, which tells the number of bits each GuidPrefix world must be<br/>reduced. <br/><br/>**Example**: `8,8,16` |
| `submessage.combine_id_and_flags`     | Combines the SubmessageId and the SubmessageFlags into a single byte in the Submessage header. <br/><br/>**Values**: true or false                                                                                                                                                                                |
| `submessage.remove_extra_flags`       | Removes the ExtraFlags field from the Submessage header. <br/><br/>**Values**: true or false                                                                                                                                                                                                                      |
| `submessage.compress_entitiy_ids`     | Compresses the EntityId fields in the Submessage header from two 32-bit words to a smaller size. <br/><br/>**Values**: Two numbers between 0 and 32, which tells the number of bits each EntityId must be reduced. <br/><br/>**Example**: `16,16`                                                                 |
| `submessage.compress_sequence_number` | Compresses the SequenceNumber field in the Submessage header from a 64-bit word to a smaller size. <br/><br/>**Values**: A number between 0 and 64, which tells the number of bits the SequenceNumber must be reduced. <br/><br/>**Example**: `32`                                                                |

<a id="transport-transportapi-headerreduction-descriptor"></a>

<a id="headerreductiontransportdescriptor"></a>

### HeaderReductionTransportDescriptor

`HeaderReductionTransportDescriptor` has no additional data members from the common ones described in
[ChainingTransportDescriptor](transport_api.md#transport-transportapi-chaining-descriptor).

<a id="enabling-header-reduction-transport"></a>

### Enabling Header Reduction Transport

To enable a new `HeaderReductionTransport` in a [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant), first
create an instance of [HeaderReductionTransportDescriptor](#transport-transportapi-headerreduction-descriptor),
and add it to the user transport list of the [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant).

The examples below show this procedure in both C++ code and XML file.

### C++

```c++
DomainParticipantQos participant_qos;

auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

// Create a descriptor for the new transport.
auto header_reduction_transport = std::make_shared<HeaderReductionTransportDescriptor>(
    udp_transport);

// [OPTIONAL] Transport configuration
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.remove_version", "true"));
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.remove_vendor_id", "true"));
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.submessage.combine_id_and_flags",
            "true"));
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.submessage.compress_entitiy_ids",
            "16,16"));

// Link the Transport Layer to the Participant.
participant_qos.transport().use_builtin_transports = false;
participant_qos.transport().user_transports.push_back(header_reduction_transport);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>reduction_transport</transport_id>
                <type>HEADER_REDUCTION</type>
                <low_level_transport>udp_transport</low_level_transport>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="HeaderReductionParticipant">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>rtps.header_reduction.remove_version</name>
                            <value>true</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.remove_vendor_id</name>
                            <value>true</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.submessage.combine_id_and_flags</name>
                            <value>true</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.submessage.compress_entitiy_ids</name>
                            <value>16,16</value>
                        </property>
                    </properties>
                </propertiesPolicy>
                <userTransports>
                    <transport_id>reduction_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```

<a id="full-example"></a>

## Full example

This example shows how to combine all low-bandwidth transports to ensure optimal performance in constrained networks.
This procedure can be configured in C++ code or XML file.

### C++

```c++
DomainParticipantQos participant_qos;

auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();

auto compress_transport =
        std::make_shared<PayloadCompressionTransportDescriptor>(udp_transport);
participant_qos.properties().properties().emplace_back(Property(
            "rtps.payload_compression.compression_library",
            "AUTOMATIC"));

auto header_reduction_transport = std::make_shared<HeaderReductionTransportDescriptor>(
    compress_transport);
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.remove_version", "true"));
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.remove_vendor_id", "true"));
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.submessage.combine_id_and_flags",
            "true"));
participant_qos.properties().properties().emplace_back(Property(
            "rtps.header_reduction.submessage.compress_entitiy_ids",
            "16,16"));

participant_qos.transport().use_builtin_transports = false;
participant_qos.transport().user_transports.push_back(header_reduction_transport);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com">
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>compression_transport</transport_id>
                <type>PAYLOAD_COMPRESSION</type>
                <low_level_transport>udp_transport</low_level_transport>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>reduction_transport</transport_id>
                <type>HEADER_REDUCTION</type>
                <low_level_transport>compression_transport</low_level_transport>
            </transport_descriptor>
        </transport_descriptors>

        <participant profile_name="LowBandwidthParticipant">
            <rtps>
                <propertiesPolicy>
                    <properties>
                        <property>
                            <name>rtps.payload_compression.compression_library</name>
                            <value>AUTOMATIC</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.remove_version</name>
                            <value>true</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.remove_vendor_id</name>
                            <value>true</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.submessage.combine_id_and_flags</name>
                            <value>true</value>
                        </property>
                        <property>
                            <name>rtps.header_reduction.submessage.compress_entitiy_ids</name>
                            <value>16,16</value>
                        </property>
                    </properties>
                </propertiesPolicy>
                <userTransports>
                    <transport_id>reduction_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
            </rtps>
        </participant>
    </profiles>
</dds>
```
