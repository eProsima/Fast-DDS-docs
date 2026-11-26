<a id="discovery-server-use-case"></a>

<a id="discovery-server"></a>

# Discovery Server

During [Discovery](../../discovery/discovery.md#discovery), the Participant Discovery Phase (PDP) relies on meta traffic
announcements sent to multicast addresses so that all the [DomainParticipants](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
in the network can acknowledge each other.
This phase is followed by a Endpoint Discovery Phase (EDP) where all the
DomainParticipants use discovered unicast addresses to exchange information about
their [Publisher](../../dds_layer/publisher/publisher.md#dds-layer-publisher) and [Subscriber](../../dds_layer/subscriber/subscriber.md#dds-layer-subscriber) entities with the rest of the
DomainParticipants, so that matching between entities of the same topic can occur.

*Fast DDS* provides a client-server discovery mechanism, in which a server DomainParticipant operates
as the central point of communication.
It collects and processes the metatraffic sent by the client DomainParticipants,
and then distributes it among the rest of the clients.

A complete description of the feature can be found at [Discovery Server](../../discovery/discovery_server.md#discovery-server).
The following subsections present configurations for different discovery server use cases.

<a id="discovery-server-major-scenario-setup"></a>

<a id="udpv4-basic-example-setup"></a>

## UDPv4 basic example setup

To configure the Discovery Server scenario, two types of participants are created: the server participant and
the client participant.
Only one parameter needs be configured in this type of implementation:

+ **Server Address-port pair**: Specifies the IP address and port of the machine that implements the server.
  Any free random port can be used.
  However, using [RTPS standard ports](../../transport/listening_locators.md#listening-locators-defaultports) is discouraged.

| **SERVER**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ### C++<br/><br/>```c++<br/>DomainParticipantQos qos;<br/><br/>// Configure the current participant as SERVER<br/>qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;<br/><br/>// Define the listening locator to be on interface 192.168.10.57 and port 56542<br/>Locator_t server_locator;<br/>IPLocator::setIPv4(server_locator, "192.168.10.57");<br/>server_locator.port = 56542;<br/>qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);<br/>```<br/><br/>### XML<br/><br/>```xml<br/><?xml version="1.0" encoding="UTF-8" ?><br/><profiles xmlns="http://www.eprosima.com"><br/>    <participant profile_name="UDP SERVER" is_default_profile="true"><br/>        <rtps><br/>            <builtin><br/>                <discovery_config><br/>                    <discoveryProtocol>SERVER</discoveryProtocol><br/>                </discovery_config><br/>                <metatrafficUnicastLocatorList><br/>                    <locator><br/>                        <udpv4><br/>                            <address>192.168.10.57</address><br/>                            <port>56542</port><br/>                        </udpv4><br/>                    </locator><br/>                </metatrafficUnicastLocatorList><br/>            </builtin><br/>        </rtps><br/>    </participant><br/></profiles><br/>``` |

| **CLIENT**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        |
|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ### C++<br/><br/>```c++<br/>DomainParticipantQos qos;<br/><br/>// Configure the current participant as CLIENT<br/>qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::CLIENT;<br/><br/>// Define a locator for the SERVER Participant on address 192.168.10.57 and port 56542<br/>Locator_t remote_server_locator;<br/>IPLocator::setIPv4(remote_server_locator, "192.168.10.57");<br/>remote_server_locator.port = 56542;<br/><br/>// Connect to the SERVER at the previous locator<br/>qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator);<br/>```<br/><br/>### XML<br/><br/>```xml<br/><?xml version="1.0" encoding="UTF-8" ?><br/><profiles xmlns="http://www.eprosima.com"><br/>    <participant profile_name="UDP CLIENT" is_default_profile="true"><br/>        <rtps><br/>            <builtin><br/>                <discovery_config><br/>                    <discoveryProtocol>CLIENT</discoveryProtocol><br/>                    <discoveryServersList><br/>                        <locator><br/>                            <udpv4><br/>                                <address>192.168.10.57</address><br/>                                <port>56542</port><br/>                            </udpv4><br/>                        </locator><br/>                    </discoveryServersList><br/>                </discovery_config><br/>            </builtin><br/>        </rtps><br/>    </participant><br/></profiles><br/>``` |

<a id="discovery-server-redundancy-scenario-setup"></a>

<a id="udpv4-redundancy-example"></a>

## UDPv4 redundancy example

The [basic setup example](#discovery-server-major-scenario-setup) presents a *single point of failure*.
That is, if the server fails the clients are not able to perform the discovery.
To prevent this, several servers could be linked to each client.
Then, a discovery failure only takes place if *all servers* fail, which is a more unlikely event.

In the example below, the values have been chosen to ensure each server has a unique
*unicast address-port pair*.
Note that several servers can share the same IP address but their port numbers should be different.
Likewise, several servers can share the same port if their IP addresses are different.

| Prefix                              | UDPv4 address-port   |
|-------------------------------------|----------------------|
| 75.63.2D.73.76.72.63.6C.6E.74.2D.31 | 192.168.10.57:56542  |
| 75.63.2D.73.76.72.63.6C.6E.74.2D.32 | 192.168.10.60:56543  |
![image](01-figures/ds_redundancy.svg)
<!-- | @startuml -->
<!-- | -->
<!-- | package "Servers" { -->
<!-- | -->
<!-- | interface "\n192.168.10.57\n56542" as P1 -->
<!-- | interface "\n192.168.10.60\n56543" as P2 -->
<!-- | -->
<!-- | P1 -left- [75.63.2D.73.76.72.63.6C.6E.74.2D.31] -->
<!-- | P2 -left- [75.63.2D.73.76.72.63.6C.6E.74.2D.32] -->
<!-- | -->
<!-- | [75.63.2D.73.76.72.63.6C.6E.74.2D.31] -[hidden]up- [75.63.2D.73.76.72.63.6C.6E.74.2D.32] -->
<!-- | P1 -[hidden]up- P2 -->
<!-- | } -->
<!-- | -->
<!-- | node "Clients" { -->
<!-- | (client\n1) as ps1 -->
<!-- | (client\n2) as ps2 -->
<!-- | (client\n3) as ps3 -->
<!-- | (client\nX) as psX -->
<!-- | } -->
<!-- | -->
<!-- | ps1 -> P1 -->
<!-- | ps1 .> P2 -->
<!-- | -->
<!-- | ps2 -> P1 -->
<!-- | ps2 .left.> P2 -->
<!-- | -->
<!-- | ps3 -> P1 -->
<!-- | ps3 .> P2 -->
<!-- | -->
<!-- | psX -> P1 -->
<!-- | psX .left.> P2 -->
<!-- | -->
<!-- | ps1 -[hidden]down- ps2 -->
<!-- | ps2 -[hidden]right- psX -->
<!-- | ps3 -[hidden]down- psX -->
<!-- | -->
<!-- | @enduml -->

| **SERVER**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ### C++<br/><br/>```c++<br/>// Configure first server's locator on interface 192.168.10.57 and port 56542<br/>Locator_t server_locator_1;<br/>IPLocator::setIPv4(server_locator_1, "192.168.10.57");<br/>server_locator_1.port = 56542;<br/><br/>// Configure participant_1 as SERVER listening on the previous locator<br/>DomainParticipantQos server_1_qos;<br/>server_1_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;<br/>// Optional GUID<br/>std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.31") >> server_1_qos.wire_protocol().prefix;<br/>server_1_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator_1);<br/><br/>// Configure second server's locator on interface 192.168.10.60 and port 56543<br/>Locator_t server_locator_2;<br/>IPLocator::setIPv4(server_locator_2, "192.168.10.60");<br/>server_locator_2.port = 56543;<br/><br/>// Configure participant_2 as SERVER listening on the previous locator<br/>DomainParticipantQos server_2_qos;<br/>server_2_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;<br/>// Optional GUID<br/>std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.32") >> server_2_qos.wire_protocol().prefix;<br/>server_2_qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator_2);<br/>```<br/><br/>### XML<br/><br/>```xml<br/><?xml version="1.0" encoding="UTF-8" ?><br/><profiles xmlns="http://www.eprosima.com"><br/>    <participant profile_name="UDP SERVER 1"><br/>        <rtps><br/>            <!-- Optional GUID --><br/>            <prefix>75.63.2D.73.76.72.63.6C.6E.74.2D.31</prefix><br/>            <builtin><br/>                <discovery_config><br/>                    <discoveryProtocol>SERVER</discoveryProtocol><br/>                </discovery_config><br/>                <metatrafficUnicastLocatorList><br/>                    <locator><br/>                        <udpv4><br/>                            <address>192.168.10.57</address><br/>                            <port>56542</port><br/>                        </udpv4><br/>                    </locator><br/>                </metatrafficUnicastLocatorList><br/>            </builtin><br/>        </rtps><br/>    </participant><br/><br/>    <participant profile_name="UDP SERVER 2"><br/>        <rtps><br/>            <!-- Optional GUID --><br/>            <prefix>75.63.2D.73.76.72.63.6C.6E.74.2D.32</prefix><br/>            <builtin><br/>                <discovery_config><br/>                    <discoveryProtocol>SERVER</discoveryProtocol><br/>                </discovery_config><br/>                <metatrafficUnicastLocatorList><br/>                    <locator><br/>                        <udpv4><br/>                            <address>192.168.10.60</address><br/>                            <port>56543</port><br/>                        </udpv4><br/>                    </locator><br/>                </metatrafficUnicastLocatorList><br/>            </builtin><br/>        </rtps><br/>    </participant><br/></profiles><br/>``` |

| **CLIENT**                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |
|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| ### C++<br/><br/>```c++<br/>// Define a locator for the first SERVER Participant<br/>Locator_t remote_server_locator_1;<br/>IPLocator::setIPv4(remote_server_locator_1, "192.168.10.57");<br/>remote_server_locator_1.port = 56542;<br/><br/>// Define a locator for the second SERVER Participant<br/>Locator_t remote_server_locator_2;<br/>IPLocator::setIPv4(remote_server_locator_2, "192.168.10.60");<br/>remote_server_locator_2.port = 56543;<br/><br/>// Configure the current participant as CLIENT connecting to the SERVERS at the previous locators<br/>DomainParticipantQos client_qos;<br/>client_qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::CLIENT;<br/>client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_1);<br/>client_qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_2);<br/>```<br/><br/>### XML<br/><br/>```xml<br/><?xml version="1.0" encoding="UTF-8" ?><br/><profiles xmlns="http://www.eprosima.com"><br/>    <participant profile_name="UDP CLIENT REDUNDANCY"><br/>        <rtps><br/>            <builtin><br/>                <discovery_config><br/>                    <discoveryProtocol>CLIENT</discoveryProtocol><br/>                    <discoveryServersList><br/>                        <locator><br/>                            <udpv4><br/>                                <address>192.168.10.57</address><br/>                                <port>56542</port><br/>                            </udpv4><br/>                        </locator><br/>                        <locator><br/>                            <udpv4><br/>                                <address>192.168.10.60</address><br/>                                <port>56543</port><br/>                            </udpv4><br/>                        </locator><br/>                    </discoveryServersList><br/>                </discovery_config><br/>            </builtin><br/>        </rtps><br/>    </participant><br/></profiles><br/>``` |

<a id="discovery-server-persistency-scenario-setup"></a>

<a id="udpv4-persistency-example"></a>

## UDPv4 persistency example

On Discovery Server, servers gather and maintain the information of all connected endpoints,
and distribute it to the clients.
In case of a server failure, all this information is lost and the server needs to recover it on restart.
In the [basic setup](#discovery-server-major-scenario-setup) this is done
starting over the [Discovery](../../discovery/discovery.md#discovery) process.
Given that servers usually have lots of clients associated, this is very time consuming.

Alternatively, *Fast DDS* allows to synchronize the server’s discovery record to a file, so that the information can be
loaded back into memory during the restart.
This feature is enabled specifying the [Discovery Protocol](../../discovery/general_disc_settings.md#discovery-protocol) as **BACKUP**.

The record file is located on the server’s process working directory, and named following the pattern
*server-<GUIDPREFIX>.db* (for example: *server-73-65-72-76-65-72-63-6C-69-65-6E-74.db*).
Once the server is created, it automatically looks for this file.
If it already exists, its contents are loaded, avoiding the need of re-discovering the clients.
To make a fresh restart, any such backup file must be removed or renamed before launching the server.

<a id="discovery-server-partitioning-setup"></a>

<a id="udpv4-partitioning-using-servers"></a>

## UDPv4 partitioning using servers

Server association can be seen as another isolation mechanism besides [Domains](../../dds_layer/domain/domain.md#dds-layer-domain) and
[Partitions](../../dds_layer/domain/domainParticipant/partition.md#partitions).
Clients that do not share a server cannot see each other and belong to isolated server networks.
For example, in the following figure, *client 1* and *client 2* cannot communicate even if they are on the
same physical network and Domain.

![image](01-figures/ds_partition.svg)

*Clients cannot see each other due to server isolation*

<!-- | @startuml -->
<!-- | -->
<!-- | package "Option 1 | Static" { -->
<!-- | -->
<!-- | component [Server 1] as 1_s1 -->
<!-- | component [Server 2] as 1_s2 -->
<!-- | (client 1) as 1_c1 -->
<!-- | (client 2) as 1_c2 -->
<!-- | -->
<!-- | 1_s2 -[hidden]up- 1_s1 -->
<!-- | 1_c2 -[hidden]up- 1_c1 -->
<!-- | -->
<!-- | } -->
<!-- | -->
<!-- | 1_s1 <- 1_c1 -->
<!-- | 1_s2 <- 1_c2 -->
<!-- | -->
<!-- | 1_s1 <- 1_c2 -->
<!-- | 1_s2 <-left- 1_c1 -->
<!-- | -->
<!-- | @enduml -->

However, it is possible to connect server isolated networks very much as physical networks
can be connected through routers:

> * [Option 1](#discovery-server-partitioning-option1):
>   Connecting the clients to several servers, so that the clients belong to several networks.
> * [Option 2](#discovery-server-partitioning-option2):
>   Connecting one server to another, so that the networks are linked together.
> * [Option 3](#discovery-server-partitioning-option3):
>   Create a new server linked to the servers to which the clients are connected.

Options 1 and 2 can only be implemented by modifying QoS values or XML configuration files beforehand.
In this regard they match the domain and partition strategy.
Option 3, however, can be implemented at runtime, when the isolated networks are already up and running.

![image](01-figures/ds_partition_link.svg)
<!-- | @startuml -->
<!-- | -->
<!-- | package "Option 1 | Static" { -->
<!-- | -->
<!-- | component [Server 1] as 1_s1 -->
<!-- | component [Server 2] as 1_s2 -->
<!-- | (client 1) as 1_c1 -->
<!-- | (client 2) as 1_c2 -->
<!-- | -->
<!-- | 1_s2 -[hidden]up- 1_s1 -->
<!-- | 1_c2 -[hidden]up- 1_c1 -->
<!-- | -->
<!-- | } -->
<!-- | -->
<!-- | 1_s1 <- 1_c1 -->
<!-- | 1_s2 <- 1_c2 -->
<!-- | -->
<!-- | 1_s1 <- 1_c2 -->
<!-- | 1_s2 <-left- 1_c1 -->
<!-- | -->
<!-- | package "Option 2 | Static" { -->
<!-- | -->
<!-- | component [Server 1] as 2_s1 -->
<!-- | component [Server 2] as 2_s2 -->
<!-- | (client 1) as 2_c1 -->
<!-- | (client 2) as 2_c2 -->
<!-- | -->
<!-- | 2_s2 -up- 2_s1 -->
<!-- | 2_c2 -[hidden]up- 2_c1 -->
<!-- | -->
<!-- | } -->
<!-- | -->
<!-- | 2_s1 <- 2_c1 -->
<!-- | -->
<!-- | 2_s2 <- 2_c2 -->
<!-- | -->
<!-- | package "Option 3 | Dynamic" { -->
<!-- | -->
<!-- | component [Server 1] as 3_s1 -->
<!-- | component [Server 2] as 3_s2 -->
<!-- | component [Aux Server] as aux -->
<!-- | -->
<!-- | (client 1) as 3_c1 -->
<!-- | (client 2) as 3_c2 -->
<!-- | -->
<!-- | 3_s2 <-up- aux -->
<!-- | aux -up-> 3_s1 -->
<!-- | 3_c2 -[hidden]up- aux -->
<!-- | aux -[hidden]up- 3_c1 -->
<!-- | } -->
<!-- | -->
<!-- | 3_s1 <-right- 3_c1 -->
<!-- | -->
<!-- | 3_s2 <-right- 3_c2 -->
<!-- | -->
<!-- | @enduml -->

<a id="discovery-server-partitioning-option1"></a>

<a id="option-1"></a>

### Option 1

Connect each client to both servers.
This case matches the [redundancy use case](#discovery-server-redundancy-scenario-setup) already introduced.

<a id="discovery-server-partitioning-option2"></a>

<a id="option-2"></a>

### Option 2

Connect one server to the other.
This means configuring one of the servers to act as a client of the other.

Consider two servers, each one managing an isolated network:

| Network   | Prefix                              | UDPv4 address       |
|-----------|-------------------------------------|---------------------|
| A         | 75.63.2D.73.76.72.63.6C.6E.74.2D.31 | 192.168.10.60:56543 |
| B         | 75.63.2D.73.76.72.63.6C.6E.74.2D.32 | 192.168.10.57:56542 |

In order to communicate both networks we can set server A to act as  a client of server B:

### C++

```c++
DomainParticipantQos qos;

// Configure current Participant as SERVER on address 192.168.10.60
Locator_t server_locator;
IPLocator::setIPv4(server_locator, "192.168.10.60");
server_locator.port = 56543;

qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;
// Optional GUID
std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.31") >> qos.wire_protocol().prefix;
qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

// Add the connection attributes to the remote server.
Locator_t remote_server_locator;
IPLocator::setIPv4(remote_server_locator, "192.168.10.57");
remote_server_locator.port = 56542;

qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="UDP SERVER A">
        <rtps>
            <!-- Optional GUID -->
            <prefix>75.63.2D.73.76.72.63.6C.6E.74.2D.31</prefix>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SERVER</discoveryProtocol>
                    <discoveryServersList>
                        <locator>
                            <udpv4>
                                <address>192.168.10.57</address>
                                <port>56542</port>
                            </udpv4>
                        </locator>
                    </discoveryServersList>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>192.168.10.60</address>
                            <port>56543</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

<a id="discovery-server-partitioning-option3"></a>

<a id="option-3"></a>

### Option 3

Create a new server linked to the servers to which the clients are connected.

Consider two servers (A and B), each one managing an isolated network, and a third
server (C) that will be used to connect the first two:

| Server   | Prefix                              | UDPv4 address       |
|----------|-------------------------------------|---------------------|
| A        | 75.63.2D.73.76.72.63.6C.6E.74.2D.31 | 192.168.10.60:56543 |
| B        | 75.63.2D.73.76.72.63.6C.6E.74.2D.32 | 192.168.10.57:56542 |
| C        | 75.63.2D.73.76.72.63.6C.6E.74.2D.33 | 192.168.10.54:56541 |

In order to communicate both networks we can setup server C to act as client of servers A and B as follows:

### C++

```c++
DomainParticipantQos qos;

// Configure current Participant as SERVER on address 192.168.10.60
Locator_t server_locator;
IPLocator::setIPv4(server_locator, "192.168.10.54");
server_locator.port = 56541;

qos.wire_protocol().builtin.discovery_config.discoveryProtocol = DiscoveryProtocol::SERVER;
// Optional GUID
std::istringstream("75.63.2D.73.76.72.63.6C.6E.74.2D.33") >> qos.wire_protocol().prefix;
qos.wire_protocol().builtin.metatrafficUnicastLocatorList.push_back(server_locator);

// Add the connection attributes to the remote server A.
Locator_t remote_server_locator_A;
IPLocator::setIPv4(remote_server_locator_A, "192.168.10.60");
remote_server_locator_A.port = 56543;

qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_A);

// Add the connection attributes to the remote server B.
Locator_t remote_server_locator_B;
IPLocator::setIPv4(remote_server_locator_B, "192.168.10.57");
remote_server_locator_B.port = 56542;

qos.wire_protocol().builtin.discovery_config.m_DiscoveryServers.push_back(remote_server_locator_B);
```

### XML

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com">
    <participant profile_name="UDP SERVER C">
        <rtps>
            <!-- Optional GUID -->
            <prefix>75.63.2D.73.76.72.63.6C.6E.74.2D.33</prefix>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SERVER</discoveryProtocol>
                    <discoveryServersList>
                        <locator>
                            <udpv4>
                                <address>192.168.10.57</address>
                                <port>56542</port>
                            </udpv4>
                        </locator>
                        <locator>
                            <udpv4>
                                <address>192.168.10.60</address>
                                <port>56543</port>
                            </udpv4>
                        </locator>
                    </discoveryServersList>
                </discovery_config>
                <metatrafficUnicastLocatorList>
                    <locator>
                        <udpv4>
                            <address>192.168.10.54</address>
                            <port>56541</port>
                        </udpv4>
                    </locator>
                </metatrafficUnicastLocatorList>
            </builtin>
        </rtps>
    </participant>
</profiles>
```

#### NOTE
GUID Prefixes are used in these examples to identify the *servers* and help to understand each scenario.
However, they are not mandatory and can be omitted.
Note that in the *clients* configuration, the GUID Prefix is always missing, as it is not needed in
order to connect to the *servers*.
