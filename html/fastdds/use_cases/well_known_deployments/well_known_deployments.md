<a id="well-known-deployments"></a>

<a id="well-known-network-deployments"></a>

# Well Known Network Deployments

It is often the case in industrial deployments, such as productions lines, that the entire network topology (hosts, IP
addresses, etc.) is known beforehand.
Such scenarios are perfect candidates for *Fast DDS* STATIC [Discovery](../../discovery/discovery.md#discovery) mechanism, which drastically reduces
the middleware setup time (time until all the entities are ready for information exchange),
while at the same time limits the connections to those strictly necessary.

Knowing the complete network topology allows to:

* Minimize the PDP meta-traffic and avoid multicast communication with [Peer-to-Peer Participant Discovery Phase](#wide-deployments-static-pdp).
* Completely avoid the EDP with [STATIC Endpoint Discovery Phase](#wide-deployments-static-edp).

<a id="wide-deployments-static-pdp"></a>

<a id="peer-to-peer-participant-discovery-phase"></a>

## Peer-to-Peer Participant Discovery Phase

The SIMPLE PDP discovery phase entails the [DomainParticipants](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) sending periodic PDP
announcements over multicast, and answering to the announcements received from remote
DomainParticipants.
As a result, the number of PDP connections grows quadratically with the number of
DomainParticipants, resulting in a large amount of meta traffic on the network.

However, if all DomainParticipants are known beforehand,
they can be configured to send their announcements only to the unicast addresses of their peers.
This is done by specifying a list of peer addresses, and by disabling the participant multicast
announcements.
As an additional advantage, with this method only the peers configured on the list are known to the
DomainParticipant, allowing to arrange which participant will communicate with which.
This reduces the amount of meta traffic if not all the DomainParticipants
need to be aware of all the rest of the remote participants present in the network.

Use-case [Fast DDS over WIFI](../wifi/wifi.md#use-case-fast-rtps-over-wifi) provides a detailed explanation on how to configure *Fast DDS* for such
cases.

<a id="wide-deployments-static-edp"></a>

<a id="static-endpoint-discovery-phase"></a>

## STATIC Endpoint Discovery Phase

Users can manually configure which [Publisher](../../dds_layer/publisher/publisher.md#dds-layer-publisher) and [Subscriber](../../dds_layer/subscriber/subscriber.md#dds-layer-subscriber) match with
each other, so they can start sharing user data right away, avoiding the EDP phase.

A complete description of the feature can be found at [STATIC Discovery](../../discovery/static.md#discovery-static).
There is also a fully functional hello world example implementing STATIC EDP in the
[examples/cpp/static_edp_discovery](https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/static_edp_discovery)
folder.

The following subsections present an example configuration where a Publisher in
Topic `HelloWorldTopic` from DomainParticipant `HelloWorldPublisher`
is matched with a Subscriber from DomainParticipant `HelloWorldSubscriber`.

<a id="create-static-discovery-xml-files"></a>

### Create STATIC discovery XML files

| **HelloWorldPublisher.xml**                                                                                                                                                                                                                                                                                                                                                                 |
|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ```xml<br/><staticdiscovery><br/>    <participant><br/>        <name>HelloWorldPublisher</name><br/>        <writer><br/>            <userId>1</userId><br/>            <entityID>2</entityID><br/>            <topicName>HelloWorldTopic</topicName><br/>            <topicDataType>HelloWorld</topicDataType><br/>        </writer><br/>    </participant><br/></staticdiscovery><br/>``` |

| **HelloWorldSubscriber.xml**                                                                                                                                                                                                                                                                                                                                                                 |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ```xml<br/><staticdiscovery><br/>    <participant><br/>        <name>HelloWorldSubscriber</name><br/>        <reader><br/>            <userId>3</userId><br/>            <entityID>4</entityID><br/>            <topicName>HelloWorldTopic</topicName><br/>            <topicDataType>HelloWorld</topicDataType><br/>        </reader><br/>    </participant><br/></staticdiscovery><br/>``` |

<a id="create-entities-and-load-static-discovery-xml-files"></a>

### Create entities and load STATIC discovery XML files

When creating the entities, the local writer/reader attributes must match those defined in the STATIC discovery
XML file loaded by the remote entity.

### PUBLISHER

### C++

```c++
// Participant configuration
DomainParticipantQos participant_qos;
participant_qos.name("HelloWorldPublisher");
participant_qos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
participant_qos.wire_protocol().builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
participant_qos.wire_protocol().builtin.discovery_config.static_edp_xml_config("HelloWorldSubscriber.xml");

// DataWriter configuration
DataWriterQos writer_qos;
writer_qos.endpoint().user_defined_id = 1;
writer_qos.endpoint().entity_id = 2;

// Create the DomainParticipant
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
if (nullptr == participant)
{
    // Error
    return;
}

// Create the Publisher
Publisher* publisher =
        participant->create_publisher(PUBLISHER_QOS_DEFAULT);
if (nullptr == publisher)
{
    // Error
    return;
}

// Create the Topic with the appropriate name and data type
std::string topic_name = "HelloWorldTopic";
std::string data_type = "HelloWorld";
Topic* topic =
        participant->create_topic(topic_name, data_type, TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}

// Create the DataWriter
DataWriter* writer =
        publisher->create_datawriter(topic, DATAWRITER_QOS_DEFAULT);
if (nullptr == writer)
{
    // Error
    return;
}
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_profile_static_pub">
        <rtps>
            <name>HelloWorldPublisher</name>
            <builtin>
                <discovery_config>
                    <EDP>STATIC</EDP>
                    <static_edp_xml_config>file://HelloWorldSubscriber.xml</static_edp_xml_config>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>

    <data_writer profile_name="uc_publisher_xml_conf_static_discovery">
        <userDefinedID>1</userDefinedID>
        <entityID>2</entityID>
    </data_writer>
</profiles>
```

### SUBSCRIBER

### C++

```c++
// Participant configuration
DomainParticipantQos participant_qos;
participant_qos.name("HelloWorldSubscriber");
participant_qos.wire_protocol().builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = false;
participant_qos.wire_protocol().builtin.discovery_config.use_STATIC_EndpointDiscoveryProtocol = true;
participant_qos.wire_protocol().builtin.discovery_config.static_edp_xml_config("HelloWorldPublisher.xml");

// DataWriter configuration
DataWriterQos writer_qos;
writer_qos.endpoint().user_defined_id = 3;
writer_qos.endpoint().entity_id = 4;

// Create the DomainParticipant
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, participant_qos);
if (nullptr == participant)
{
    // Error
    return;
}

// Create the Subscriber
Subscriber* subscriber =
        participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
if (nullptr == subscriber)
{
    // Error
    return;
}

// Create the Topic with the appropriate name and data type
std::string topic_name = "HelloWorldTopic";
std::string data_type = "HelloWorld";
Topic* topic =
        participant->create_topic(topic_name, data_type, TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}

// Create the DataReader
DataReader* reader =
        subscriber->create_datareader(topic, DATAREADER_QOS_DEFAULT);
if (nullptr == reader)
{
    // Error
    return;
}
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="participant_profile_static_sub">
        <rtps>
            <name>HelloWorldSubscriber</name>
            <builtin>
                <discovery_config>
                    <static_edp_xml_config>file://HelloWorldPublisher.xml</static_edp_xml_config>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>

    <data_reader profile_name="uc_subscriber_xml_conf_static_discovery">
        <userDefinedID>3</userDefinedID>
        <entityID>4</entityID>
    </data_reader>
</profiles>
```
