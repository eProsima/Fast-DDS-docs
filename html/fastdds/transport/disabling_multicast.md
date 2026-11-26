<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="transport-disablemulticast"></a>

<a id="disabling-all-multicast-traffic"></a>

# Disabling all Multicast Traffic

If all the peers are known beforehand and have been configured on the
[Initial Peers List](../discovery/simple.md#simple-initial-peers), all multicast traffic can be completely disabled.

By defining a custom [Metatraffic Unicast Locators](listening_locators.md#listening-locators-metaunicast), the local [DomainParticipant](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
creates a unicast meta traffic receiving resource for each address-port pair specified in the list,
avoiding the creation of the default metatraffic multicast and unicast locators.
This prevents the DomainParticipant from listening to any discovery data from
multicast sources.

Consideration should be given to the assignment of the ports in the
[`metatrafficUnicastLocatorList`](../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes29metatrafficUnicastLocatorListE), avoiding the assignment of ports that are not available or do
not match the address-port listed in the publisher participant Initial Peers List.

The following is an example of how to disable all multicast traffic configuring one
*metatraffic unicast* locator.

### C++

```c++
DomainParticipantQos qos;

// Metatraffic Multicast Locator List will be empty.
// Metatraffic Unicast Locator List will contain one locator, with null address and null port.
// Then Fast DDS will use all network interfaces to receive network messages using a well-known port.
Locator_t default_unicast_locator;
qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(default_unicast_locator);

// Initial peer will be UDPv4 address 192.168.0.1. The port will be a well-known port.
// Initial discovery network messages will be sent to this UDPv4 address.
Locator_t initial_peer;
IPLocator::setIPv4(initial_peer, 192, 168, 0, 1);
qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="disable_multicast" is_default_profile="true">
        <rtps>
            <builtin>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>192.168.0.1</address>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>192.168.0.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```
