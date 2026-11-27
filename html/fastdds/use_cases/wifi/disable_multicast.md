<a id="use-case-disabling-multicast-discovery"></a>

<a id="disabling-multicast-discovery"></a>

# Disabling multicast discovery

If all the peers are known and configured on the initial peer list beforehand,
it is possible to disable the multicast meta traffic completely, as all
[DomainParticipants](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) can communicate among them through unicast.

The complete description of the procedure to disable multicast discovery can be found at
[Disabling all Multicast Traffic](../../transport/disabling_multicast.md#transport-disablemulticast).
For convenience, however, this example shows how to disable all multicast traffic configuring one
*metatraffic unicast* locator.
Consideration should be given to the assignment of the ports in the `metatrafficUnicastLocatorList`,
avoiding the assignment of ports that are not available or do not match the address-port
listed in the [intial peers list](../../discovery/simple.md#simple-initial-peers) of the peer participant.

### C++

```c++
DomainParticipantQos qos;

// configure one metatraffic unicast locator on interface 192.168.10.13.
// on participant ID `1` and domain `0`.
Locator_t meta_unicast_locator;
IPLocator::setIPv4(meta_unicast_locator, "192.168.10.13");
meta_unicast_locator.port = 7412;
qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(meta_unicast_locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="initial_peers_multicast_avoidance" is_default_profile="true" >
        <rtps>
            <builtin>
                <!-- Choosing a specific unicast address -->
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>192.168.10.13</address>
                            <port>7412</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```
