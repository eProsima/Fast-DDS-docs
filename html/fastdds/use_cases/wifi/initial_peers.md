<a id="use-case-initial-peers"></a>

<a id="configuring-initial-peers"></a>

# Configuring Initial Peers

A complete description of the initial peers list and its configuration can be found in
[Initial peers](../../discovery/simple.md#simple-initial-peers).
For convenience, this example shows how to configure an initial peers list with one peer
on host `192.168.10.13` with participant ID `1` in domain `0`.

#### NOTE
Note that the port number used here is not arbitrary, as discovery ports are defined by
the [RTPS v2.2 standard](https://www.omg.org/spec/DDSI-RTPS/2.2/).
Refer to [Well Known Ports](../../transport/listening_locators.md#listening-locators-defaultports) to learn about these standard port numbers.

If the participant ID is not known, setting [TransportDescriptorInterface](../../transport/transport_api.md#transport-transportapi-transportdescriptor)
`maxInitialPeersRange` to at least the maximum expected number of DomainParticipants will ensure discovery and
communication.

### C++

```c++
DomainParticipantQos qos;

// configure an initial peer on host 192.168.10.13.
// The port number corresponds to the well-known port for metatraffic unicast
// on participant ID `1` and domain `0`.
Locator_t initial_peer;
IPLocator::setIPv4(initial_peer, "192.168.10.13");
initial_peer.port = 7412;
qos.wire_protocol().builtin.initialPeersList.push_back(initial_peer);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="initial_peers_example_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>192.168.10.13</address>
                            <port>7412</port>
                        </udpv4>
                    </locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```
