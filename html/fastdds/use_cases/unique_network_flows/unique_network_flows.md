<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="use-case-unique-flows"></a>

<a id="unique-network-flows"></a>

# Unique network flows

This section explains which APIs should be used on Fast DDS in order to have unique network flows on specific topics.

<a id="background"></a>

## Background

IP networking is the pre-dominant inter-networking technology used nowadays.
Ethernet, WiFi, 4G/5G telecommunication, all of them rely on IP networking.

Streams of IP packets from a given source to destination are called *packet flows* or simply *flows*.
The network QoS of a flow can be configured when using certain networking equipment (routers, switches).
Such pieces of equipment typically support 3GPP/5QI protocols to assign certain Network QoS parameters to
specific flows.
Requesting a specific Network QoS is usually done on the endpoint sending the data, as it is the one
that usually haves complete information about the network flow.

Applications may need to use specific Network QoS parameters on different topics.

This means an application should be able to:

1. Identify the flows being used in the communications, so they can correctly configure the networking
   equipment.
2. Use specific flows on selected topics.

<a id="identifying-a-flow"></a>

## Identifying a flow

The *5-tuple* is a traditional unique identifier for flows on 3GPP enabled equipment.
The 5-tuple consists of five parameters: source IP address, source port, destination IP address, destination port,
and the transport protocol (example, TCP/UDP).

<a id="definitions"></a>

### Definitions

**Network flow**: A tuple of networking resources selected by the middleware for transmission of messages from a
DataWriter to a DataReader, namely:

- Transport protocol: UDP or TCP
- Transport port
- Internet protocol: IPv4 or IPv6
- IP address

**Network Flow Endpoint (NFE)**: The portion of a network flow specific to the DataWriter or the DataReader.
In other words, each network flow has two NFEs; one for the DataWriter, and the other for the DataReader.

<a id="apis"></a>

### APIs

Fast DDS provides the APIs needed to get the list of NFEs used by a given DataWriter or a DataReader.

* On the DataWriter, [`get_sending_locators()`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4NK8eprosima7fastdds3dds10DataWriter20get_sending_locatorsERN4rtps11LocatorListE) allows the application to obtain the list of locators
  from which the writer may send data.
* On the DataReader, [`get_listening_locators()`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4NK8eprosima7fastdds3dds10DataReader22get_listening_locatorsERN4rtps11LocatorListE) allows the application to obtain the list of locators
  on which the reader is listening.

<a id="requesting-unique-flows"></a>

## Requesting unique flows

A unique flow can be created by ensuring that at least one of the two NFEs are unique.
On Fast DDS, there are two ways to select unique listening locators on the DataReader:

* The application can specify on which locators the DataReader should be listening.
  This is done using [RTPSEndpointQos](../../dds_layer/core/policy/eprosimaExtensions.md#rtpsendpointqos) on the [DataReaderQos](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos).
  In this case it is the responsibility of the application to ensure the uniqueness of the locators used.
* The application can request the reader to be created with unique listening locators.
  This is done using a [PropertyPolicyQos](../../dds_layer/core/policy/eprosimaExtensions.md#propertypolicyqos) including the property `"fastdds.unique_network_flows"`.
  In this case, the reader will listen on a unique port outside the range of ports typically used by RTPS.

<a id="example"></a>

## Example

The following snippet demonstrates all the APIs described on this page:

```c++
// Create the DataWriter
DataWriter* writer = publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == writer)
{
    // Error
    return;
}

// Create DataReader with unique flows
DataReaderQos drqos = DATAREADER_QOS_DEFAULT;
drqos.properties().properties().emplace_back("fastdds.unique_network_flows", "");
DataReader* reader = subscriber->create_datareader(topic, drqos);

// Print locators information
eprosima::fastdds::rtps::LocatorList locators;
writer->get_sending_locators(locators);
std::cout << "Writer is sending from the following locators:" << std::endl;
for (const auto& locator : locators)
{
    std::cout << "  " << locator << std::endl;
}

reader->get_listening_locators(locators);
std::cout << "Reader is listening on the following locators:" << std::endl;
for (const Locator_t& locator : locators)
{
    std::cout << "  " << locator << std::endl;
}
```
