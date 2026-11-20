<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dynamic-network-interfaces"></a>

<a id="id1"></a>

# Dynamic network interfaces

#### WARNING
Starting in Fast DDS v3.4.0, this feature has been deprecated in favor of [IP mobility](../ip_mobility/ip_mobility.md#ip-mobility) **[Pro]**.

DDS [Simple Discovery](../../discovery/simple.md#simple-disc-settings) relies on well-known multicast addresses and ports to relay the
Participant announcement messages (see [Discovery phases](../../discovery/discovery.md#disc-phases)).
Such Participant announcement includes information about the unicast address-port pairs (a.k.a locators) where the
Participant is expecting to receive incoming metatraffic data.
The list with these unicast locators is automatically initialized taking into account the network interfaces that are
available when the *Fast DDS* DomainParticipant is enabled.
Consequently, any network interface that is added after enabling the DomainParticipant should be notified to *Fast DDS*
in order to initialize an unicast locator in said network, so communication can be established over that new interface.

<a id="dynamic-network-interface-addition-at-run-time"></a>

## Dynamic network interface addition at run-time

In case that the user wants to include new network interfaces at run-time, some prerequisites have to be fulfilled.
Then, once the interfaces are available, the user may notify Fast DDS so these interfaces are also used in the
communication.

<a id="prerequisites"></a>

### Prerequisites

This feature is intended to be used when *Fast DDS* automatically sets the listening unicast locators.
Consequently, both [`metatrafficUnicastLocatorList`](../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes29metatrafficUnicastLocatorListE) and
[`metatrafficMulticastLocatorList`](../../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes31metatrafficMulticastLocatorListE) lists must be empty.
These attributes are set within the builtin member of [`wire_protocol()`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4NK8eprosima7fastdds3dds20DomainParticipantQos13wire_protocolEv) contained in the
[`DomainParticipantQos`](../../api_reference/dds_pim/domain/domainparticipantqos.md#_CPPv4N8eprosima7fastdds3dds20DomainParticipantQosE) (please refer to [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos)).

#### NOTE
Be aware of the remote locators’ collections limits set within the [DomainParticipantQos](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) (please refer to
[RemoteLocatorsAllocationAttributes](../../dds_layer/core/policy/eprosimaExtensions.md#remotelocatorsallocationattributes)).
It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.

<a id="notify-fast-dds"></a>

### Notify *Fast DDS*

Once a new network interface has been enabled, *Fast DDS* has to be manually notified.
This is done calling [`DomainParticipant::set_qos()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos).
The DomainParticipantQoS that is passed to the method can either change one of the mutable DomainParticipant QoS or
it can simply be the current DomainParticipant QoS (obtained with [`DomainParticipant::get_qos()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7get_qosER20DomainParticipantQos)).

Using [`DomainParticipant::set_qos()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant7set_qosERK20DomainParticipantQos) is the reason for the previous prerequisites: once the DomainParticipant is
enabled, there are several QoS policies that are immutable and cannot be changed at run-time.
[WireProtocolConfigQos](../../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos) where the aforementioned lists are defined is among these immutable policies.

Find below a brief snippet of how to use this feature:

```c++
// Create the DomainParticipant
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// User application

// Notify Fast DDS a new network interface is available
participant->set_qos(PARTICIPANT_QOS_DEFAULT);
```

#### IMPORTANT
This feature is only officially supported for UDPv4 Transport without interface
filtering (allowlist, blocklist or whitelist).
Interface filters are only re-evaluated upon network interfaces changes by [IP mobility](../ip_mobility/ip_mobility.md#ip-mobility) **[Pro]**
(see [Interface filtering](../ip_mobility/ip_mobility.md#ip-mobility-interface-filtering)).
