<a id="use-case-manysubscribers"></a>

<a id="topics-with-many-subscribers"></a>

# Topics with many subscribers

By default, every time a [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) publishes a data change on a
[Topic](../../dds_layer/topic/topic/topic.md#dds-layer-topic-topic), it sends a unicast message for every
[DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) that is subscribed to the Topic.
If there are several DataReaders subscribed, it is recommendable
to use multicast instead of unicast.
By doing so, only one network package will be sent for each sample.
This will improve both CPU and network usage.

This solution can be implemented with [UDP Transport](../../transport/udp/udp.md#transport-udp-udp) or [Shared Memory Transport](../../transport/shared_memory/shared_memory.md#transport-sharedmemory-sharedmemory) (SHM).
SHM transport is multicast by default, but is only available between DataWriters and
DataReaders on the same machine.
UDP transport needs some extra configuration.
The example below shows how to set a [DataReaderQos](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos) to configure
a DataReader to use a multicast transport on UDP.
More information about configuring local and remote locators on endpoints can be found in [RTPSEndpointQos](../../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos).

#### NOTE
Multicast over UDP can be problematic on some scenarios, mainly WiFi and complex networks
with multiple network links.

### C++

```c++
DataReaderQos qos;

// Add new multicast locator with IP 239.255.0.4 and port 7900
eprosima::fastdds::rtps::Locator_t new_multicast_locator;
eprosima::fastdds::rtps::IPLocator::setIPv4(new_multicast_locator, "239.255.0.4");
new_multicast_locator.port = 7900;
qos.endpoint().multicast_locator_list.push_back(new_multicast_locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <data_reader profile_name="reader_xml_conf_multicast_locators_profile">
        <multicastLocatorList>
            <locator>
                <udpv4>
                    <address>239.255.0.4</address>
                    <port>7900</port>
                </udpv4>
            </locator>
        </multicastLocatorList>
    </data_reader>
</profiles>
```
