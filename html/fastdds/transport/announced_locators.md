<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="announced-locators"></a>

<a id="id1"></a>

# Announced Locators

In order for communication to take place, DDS entities need to exchange the list of addresses and ports where
they can be reached.
Apart from the [default announced locators](#default-announced-locators), which correspond to addresses
of the interfaces in the host where the application is running, the user can configure
[additional locators](#external-locators) with addresses and ports on other networks, when routing rules
have been correspondingly set up.

<a id="default-announced-locators"></a>

<a id="id2"></a>

## Default Announced Locators

The default list of announced locators will be constructed from the [listening locators](listening_locators.md#listening-locators),
as follows:

* If the address field of the locator is a null address (i.e. 0.0.0.0 for UDPv4), a locator of the same kind and port
  will be announced for each of the addresses of the network interfaces of the host.
* If the address field of the locator is not a null address, a single locator with that address will be announced.

<a id="external-locators"></a>

<a id="id3"></a>

## External Locators

The user can configure a set of external locators for each of the lists of unicast locators:

* [`builtin.metatraffic_external_unicast_locators`](../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes37metatraffic_external_unicast_locatorsE) on [WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos)
* [`default_external_unicast_locators`](../api_reference/dds_pim/core/policy/wireprotocolconfigqos.md#_CPPv4N8eprosima7fastdds3dds21WireProtocolConfigQos33default_external_unicast_locatorsE) on [WireProtocolConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#wireprotocolconfigqos)
* [`external_unicast_locators`](../api_reference/dds_pim/core/policy/rtpsendpointqos.md#_CPPv4N8eprosima7fastdds3dds15RTPSEndpointQos25external_unicast_locatorsE) on [RTPSEndpointQos](../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos)

An external locator is made up of the standard locator fields (kind, address, and port), plus the following attributes:

* An *externality* that indicates the number of hops from the host where the application is running to the LAN
  represented by the external locator.
* A *cost* indicating the communication cost relative to other locators on the same externality level.
* A *mask* with the number of significant bits on the LAN represented by the external locator.

<a id="externality-levels"></a>

### Externality levels

The main purpose of the external locators is to enable communication across different levels of interconnected LANs.
Communication will be performed using the locators of the innermost LAN available.

As an example, consider a network topology where the application is running on a host connected to a LAN of an office,
which in turn connects to a LAN for all the offices in the same floor, which in turn connects to a LAN for the
building.

With the default configuration, communication will only occur between hosts on the LAN for the office.
This is considered the externality level 0, which is reserved for the LANs directly connected to the network interfaces
of the host where the application is running.
This is the externality level that will be used on the [matching algorithm](#external-locators-algorithm) for the
[default announced locators](#default-announced-locators).
The floor LAN will be configured as externality level 1, whereas the building LAN will be configured as externality
level 2.

Note that in order for the communication to be successful, routing rules should most probably need to be added to the
different network routers.

#### IMPORTANT
Externality level 0 is automatically populated by Fast DDS and cannot be configured by the application.

<a id="external-locators-algorithm"></a>

<a id="matching-algorithm"></a>

### Matching algorithm

When a remote entity is discovered, its list of announced locators is processed to select the ones on the innermost
externality level where the communication can be established.
The highest externality level is checked first.

If the discovered addresses for one level are equal to the ones announced by the local entity, it means they are on
the same host at that level, and the algorithm proceeds to an inner level.
If the discovered addresses are not equal to the ones announced by the local entity, processing stops at the current
level.

When the externality level on which the communication will be established has been decided, the algorithm will:

* Remove locators that match with addresses on any other externality level.
* Keep locators that match with the selected externality level.
* For the locators with an address that does not match with any of the locators announced by the local entity:
  * Keep them when `ignore_non_matching_locators` is `false` (default behavior)
  * Remove them when `ignore_non_matching_locators` is `true`

<a id="additional-considerations"></a>

### Additional considerations

Since using external locators increases the number of locators announced, the
[allocation limits for locators discovery](../dds_layer/core/policy/eprosimaExtensions.md#remotelocatorsallocationattributes) would need to be adjusted for
your application.

Participants running on the same host, but using different addresses on their
[`builtin.metatraffic_external_unicast_locators`](../api_reference/rtps/attributes/BuiltinAttributes.md#_CPPv4N8eprosima7fastdds4rtps17BuiltinAttributes37metatraffic_external_unicast_locatorsE) will discard shared memory transport locators.
Data sharing communication is not affected by this limitation.
