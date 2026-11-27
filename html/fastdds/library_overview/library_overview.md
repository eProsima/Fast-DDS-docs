<a id="library-overview"></a>

<a id="id1"></a>

# Library Overview

Fast DDS (formerly Fast RTPS) is an efficient and high-performance implementation of the
[DDS specification](https://www.omg.org/spec/DDS/1.4), a data-centric communications
middleware (DCPS) for distributed application software.
This section reviews the architecture, operation and key features of Fast DDS.

<a id="architecture"></a>

## Architecture

The architecture of *Fast DDS* is shown in the figure below, where a layer model with the following different
environments can be seen.

* **Application layer**.
  The user application that makes use of the *Fast DDS* API for the implementation of communications in
  distributed systems.
* **Fast DDS layer**.
  Robust implementation of the DDS communications middleware.
  It allows the deployment of one or more DDS domains in which DomainParticipants within the same domain
  exchange messages by publishing/subscribing under a domain topic.
* **RTPS layer**.
  Implementation of the [Real-Time Publish-Subscribe (RTPS) protocol](https://www.omg.org/spec/DDSI-RTPS/2.2)
  for interoperability with DDS applications.
  This layer acts as an abstraction layer of the transport layer.
* **Transport Layer**.
  *Fast DDS* can be used over various transport protocols such as unreliable transport protocols (UDP), reliable
  transport protocols (TCP), or shared memory transport protocols (SHM).

![image](01-figures/fast_dds/library_overview/library_overview.svg)

*Fast DDS layer model architecture*

<a id="dds-layer"></a>

### DDS Layer

Several key elements for communication are defined in the DDS layer of *Fast DDS*.
The user will create these elements in their application, thus incorporating DDS application elements and creating a
data-centric communication system.
*Fast DDS*, following the DDS specification, defines these elements involved in communication as **Entities**.
A DDS **Entity** is any object that supports Quality of Service configuration (QoS), and that implements a listener.

* **QoS**.
  The mechanism by which the behavior of each of the entities is defined.
* **Listener**.
  The mechanism by which the entities are notified of the possible events that arise during the application’s execution.

Below are listed the DDS Entities together with their description and functionality.
For a more detailed explanation of each entity, their QoS, and their listeners, please refer to [DDS Layer](../dds_layer/dds_layer.md#dds-layer)
section.

* **Domain**.
  A positive integer which identifies the DDS domain.
  Each DomainParticipant will have an assigned DDS domain, so that DomainParticipants in the same domain can
  communicate, as well as isolate communications between DDS domains.
  This value must be given by the application developer when creating the DomainParticipants.
* **DomainParticipant**.
  Object containing other DDS entities such as publishers, subscribers, topics and multitopics.
  It is the entity that allows the creation of the previous entities it contains, as well as the configuration of their
  behavior.
* **Publisher**.
  The Publisher publishes data under a topic using a DataWriter, which writes the data to the transport.
  It is the entity that creates and configures the DataWriter entities it contains, and may contain one or more
  of them.
* **DataWriter**.
  It is the entity in charge of publishing messages.
  The user must provide a Topic when creating this entity which will be the Topic under which the data will be
  published.
  Publication is done by writing the data-objects as a change in the DataWriterHistory.
* **DataWriterHistory**.
  This is a list of changes to the data-objects.
  When the DataWriter proceeds to publish data under a specific Topic, it actually creates a change in this data.
  It is this change that is registered in the History.
  These changes are then sent to the DataReader that subscribes to that specific topic.
* **Subscriber**.
  The Subscriber subscribes to a topic using a DataReader, which reads the data from the transport.
  It is the entity that creates and configures the DataReader entities it contains, and may contain one or more
  DataReader entities.
* **DataReader**.
  It is the entity that subscribes to the topics for the reception of publications.
  The user must provide a subscription Topic when creating this entity.
  A DataReader receives the messages as changes in its DataReaderHistory.
* **DataReaderHistory**.
  It contains the changes in the data-objects that the DataReader receives as a result of subscribing to a certain
  Topic.
* **Topic**. Entity that binds Publishers’ DataWriters with Subscribers’ DataReaders.

<a id="rtps-layer"></a>

### RTPS layer

As mentioned above, the RTPS protocol in *Fast DDS* allows the abstraction of DDS application entities from the
transport layer.
According to the graph shown above, the RTPS layer has four main **Entities**.

* **RTPSDomain**.
  It is the extension of the DDS domain to the RTPS protocol.
* **RTPSParticipant**.
  Entity containing other RTPS entities. It allows the configuration and creation of the entities it contains.
* **RTPSWriter**.
  The source of the messages. It reads the changes written in the DataWriterHistory and transmits them to all
  the RTPSReaders to which it has previously matched.
* **RTPSReader**.
  Receiving entity of the messages. It writes the changes reported by the RTPSWriter into the DataReaderHistory.

For a more detailed explanation of each entity, their attributes, and their listeners, please refer to [RTPS Layer](../rtps_layer/rtps_layer.md#rtps-layer)
section.

<a id="transport-layer"></a>

### Transport layer

*Fast DDS* supports the implementation of applications over various transport protocols.
Those are UDPv4, UDPv6, TCPv4, TCPv6 and Shared Memory Transport (SHM).
By default, a DomainParticipant implements a UDPv4 and a SHM transport protocol.
The configuration of all supported transport protocols is detailed in the [Transport Layer](../transport/transport.md#comm-transports-configuration) section.

<a id="programming-and-execution-model"></a>

## Programming and execution model

*Fast DDS* is concurrent and event-based.
The following explains the multithreading model that governs the operation of *Fast DDS* as well as the possible events.

<a id="concurrency-multithreading"></a>

<a id="concurrency-and-multithreading"></a>

### Concurrency and multithreading

*Fast DDS* implements a concurrent multithreading system.
Each DomainParticipant spawns a set of threads to take care of background tasks such as logging, message reception, and
asynchronous communication.
This should not impact the way you use the library, i.e. the *Fast DDS* API is thread safe, so you can fearlessly call
any methods on the same DomainParticipant from different threads.
However, this multithreading implementation must be taken into account when external functions access to resources that
are modified by threads running internally in the library.
An example of this is the modified resources in the entity listener callbacks.

The complete set of threads spawned by Fast DDS is shown below.
Transport related threads (marked as UDP, TCP and SHM types) are only created when the appropriate Transport is used.

| Name                   | Type               | Cardinality                                                   | OS thread name                                                             | Description                                                                                                                                                                                                                                                                                                                                              |
|------------------------|--------------------|---------------------------------------------------------------|----------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Event                  | General            | One per DomainParticipant                                     | `dds.ev.<participant_id>`                                                  | Processes periodic and triggered time events.<br/>See [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos).                                                                                                                                                                                 |
| Discovery Server Event | General            | One per DomainParticipant                                     | `dds.ds_ev.<participant_id>`                                               | Synchronizes access to the Discovery Server  Database.<br/>See [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos).                                                                                                                                                                        |
| Asynchronous Writer    | General            | One per enabled asynchronous  flow controller.<br/>Minimum 1. | `dds.asyn.<participant_id>.`<br/>`<async_flow_controller_index>`           | Manages asynchronous writes.<br/>Even for synchronous writers, some forms of  communication must be initiated in the  background.<br/>See [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos) and [FlowControllersQos](../dds_layer/core/policy/eprosimaExtensions.md#flowcontrollersqos). |
| Datasharing Listener   | General            | One per  DataReader                                           | `dds.dsha.<reader_id>`                                                     | Listener thread that processes messages  received via Datasharing.<br/>See [DataReaderQos](../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareaderqos).                                                                                                                                                                         |
| Reception              | UDP                | One per port                                                  | `dds.udp.<port>`                                                           | Listener thread that processes incoming  UDP messages.<br/>See [TransportConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#transportconfigqos) and [UDPTransportDescriptor](../transport/udp/udp.md#transport-udp-transportdescriptor).                                                                                                          |
| Reception              | TCP                | One per TCP connection                                        | `dds.tcp.<port>`                                                           | Listener thread that processes incoming  TCP messages.<br/>See [TCPTransportDescriptor](../transport/tcp/tcp.md#transport-tcp-transportdescriptor).                                                                                                                                                                                                      |
| Accept                 | TCP                | One per TCP transport                                         | `dds.tcp_accept`                                                           | Thread that processes incoming TCP connection requests.<br/>See [TCPTransportDescriptor](../transport/tcp/tcp.md#transport-tcp-transportdescriptor).                                                                                                                                                                                                     |
| Keep Alive             | TCP                | One per TCP transport                                         | `dds.tcp_keep`                                                             | Keep alive thread for TCP connections.<br/>See [TCPTransportDescriptor](../transport/tcp/tcp.md#transport-tcp-transportdescriptor).                                                                                                                                                                                                                      |
| Reception              | SHM                | One per port                                                  | `dds.shm.<port>`                                                           | Listener thread that processes incoming  messages via SHM segments.<br/>See [TransportConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#transportconfigqos) and [SharedMemTransportDescriptor](../transport/shared_memory/shared_memory.md#transport-sharedmemory-transportdescriptor).                                                          |
| Logging                | SHM                | One per port                                                  | `dds.shmd.<port>`                                                          | Stores and dumps transferred packets to a file.<br/>See [TransportConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#transportconfigqos) and [SharedMemTransportDescriptor](../transport/shared_memory/shared_memory.md#transport-sharedmemory-transportdescriptor).                                                                              |
| Watchdog               | SHM                | One                                                           | `dds.shm.wdog`                                                             | Monitors health of open shared memory  segments.<br/>See [TransportConfigQos](../dds_layer/core/policy/eprosimaExtensions.md#transportconfigqos) and [SharedMemTransportDescriptor](../transport/shared_memory/shared_memory.md#transport-sharedmemory-transportdescriptor).                                                                             |
| General Logging        | Log                | One                                                           | `dds.log`                                                                  | Accumulates and writes to the appropriate  consumer log entries.<br/>See [Logging Thread](../logging/logging_thread.md#dds-layer-log-thread).                                                                                                                                                                                                            |
| Security Logging       | Log                | One per  DomainParticipant                                    | `dds.slog.<participant_id>`                                                | Accumulates and writes security log entries.<br/>See [DomainParticipantQos](../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipantqos).                                                                                                                                                                                  |
| Watchdog               | Filewatch          | One                                                           | `dds.fwatch`                                                               | Tracks the status of the watched file for  modifications.<br/>See [DomainParticipantFactoryQos](../dds_layer/domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactoryqos).                                                                                                                                         |
| Callback               | Filewatch          | One                                                           | `dds.fwatch.cb`                                                            | Runs the registered callback when the  watched file changes.<br/>See [DomainParticipantFactoryQos](../dds_layer/domain/domainParticipantFactory/domainParticipantFactory.md#dds-layer-domainparticipantfactoryqos).                                                                                                                                      |
| Reception              | TypeLookup Service | Two per DomainParticipant                                     | `dds.tls.replies.<participant_id>`<br/>`dds.tls.requests.<participant_id>` | Runs when remote endpoint discovery information has been received<br/>with unknown data type.                                                                                                                                                                                                                                                            |

Some of these threads are only spawned when certain conditions are met:

* Datasharing listener thread is created only when Datasharing is in use.
* Discovery Server Event thread is only created when the DomainParticipant is configured as a Discovery Server SERVER.
* TCP keep alive thread requires the keep alive period to be configured to a value greater than zero.
* Security logging and Shared Memory packet logging threads both require certain configuration options to be enabled.
* Filewatch threads are only spawned if the [FASTDDS_ENVIRONMENT_FILE](../env_vars/env_vars.md#env-vars-fastdds-environment-file) is in use.

Regarding transport threads, Fast DDS by default uses both a UDP and a Shared Memory transport.
Port configuration can be configured to suit the specific needs of the deployment,
but the default configuration is to always use a metatraffic port and a unicast user traffic port.
This applies both to UDP and Shared Memory since TCP does not support multicast.
More information can be found at the [Default Listening Locators](../transport/listening_locators.md#listening-locators-default) page.

*Fast DDS* offers the possibility of configuring certain attributes of the threads it creates by means of the
[ThreadSettings](../dds_layer/core/policy/eprosimaExtensions.md#threadsettingsqos).

<a id="event-driven-architecture"></a>

### Event-driven architecture

There is a time-event system that enables *Fast DDS* to respond to certain conditions, as well as schedule periodic
operations.
Few of them are visible to the user since most are related to DDS and RTPS metadata.
However, the user can define in their application periodic time-events by inheriting from the `TimedEvent`
class.

<a id="functionalities"></a>

## Functionalities

*Fast DDS* has some added features that can be implemented and configured by the user in their application.
These are outlined below.

<a id="discovery-protocols"></a>

### Discovery Protocols

The discovery protocols define the mechanisms by which DataWriters publishing under a given Topic, and DataReaders
subscribing to that same Topic are matched, so that they can start sharing data.
This applies at any point in the communication process.
*Fast DDS* provides the following discovery mechanisms:

* **Simple Discovery**.
  This is the default discovery mechanism, which is defined in the
  [RTPS standard](https://www.omg.org/spec/DDSI-RTPS/2.2) and provides compatibility with other DDS implementations.
  Here the DomainParticipants are discovered individually at an early stage to subsequently match the DataWriter and
  DataReader they implement.
* **Discovery Server**.
  This discovery mechanism uses a centralized discovery architecture, where servers act as hubs for meta traffic
  discovery.
* **Static Discovery**.
  This implements the discovery of DomainParticipants to each other but it is possible to skip the discovery of the
  entities contained in each DomainParticipant (DataReader/DataWriter) if these entities are known in advance by the
  remote DomainParticipants.
* **Manual Discovery**.
  This mechanism is only compatible with the RTPS layer.
  It allows the user to manually match and unmatch RTPSParticipants, RTPSWriters, and RTPSReaders using whatever
  external meta-information channel of its choice.

The detailed explanation and configuration of all the discovery protocols implemented in *Fast DDS* can be seen in
the [Discovery](../discovery/discovery.md#discovery) section.

<a id="security"></a>

### Security

*Fast DDS* can be configured to provide secure communications by implementing pluggable security at three levels:

* Authentication of remote DomainParticipants.
  The **DDS:Auth:PKI-DH** plugin provides authentication using a trusted Certificate
  Authority (CA) and ECDSA Digital Signature Algorithms to perform the mutual authentication.
  It also establishes a shared secret using either Elliptic Curve Diffie-Hellman (ECDH) or MODP-2048 Diffie-Hellman (DH)
  as Key Agreement protocol.
* Access control of entities.
  The **DDS:Access:Permissions** plugin provides access control to DomainParticipants at the DDS Domain and Topic level.
* Encryption of data.
  The **DDS:Crypto:AES-GCM-GMAC** plugin provides authenticated encryption using Advanced Encryption Standard (AES) in
  Galois Counter Mode (AES-GCM).

More information about security configuration in *Fast DDS* is available in the [Security](../security/security.md#security) section.

<a id="logging"></a>

### Logging

*Fast DDS* provides an extensible Logging system.
`Log` class is the entry point of the Logging system.
It exposes three macro definitions to ease its usage:
`EPROSIMA_LOG_INFO`, `EPROSIMA_LOG_WARNING` and `EPROSIMA_LOG_ERROR`.
Moreover, it allows the definition of new categories, in addition to those already available
(`INFO_MSG`, `WARN_MSG` and `ERROR_MSG`).
It provides filtering by category using regular expressions, as well as control of the verbosity of the Logging system.
Details of the possible Logging system configurations can be found in the [Logging](../logging/intro.md#dds-layer-log-intro) section.

<a id="xml-profiles-configuration"></a>

### XML profiles configuration

*Fast DDS* offers the possibility to make changes in its default settings by using XML profile configuration files.
Thus, the behavior of the DDS Entities can be modified without the need for the user to implement any program source
code or re-build an existing application.

The user has XML tags for each of the API functionalities.
Therefore, it is possible to build and configure DomainParticipant profiles through the `<participant>` tag, or
the DataWriter and DataReader profiles with the `<data_writer>` and `<data_reader>` tags respectively.

For a better understanding of how to write and use these XML profiles configuration files you can continue reading
the [XML profiles](../xml_configuration/xml_configuration.md#xml-profiles) section.

<a id="environment-variables"></a>

### Environment variables

Environment variables are those variables that are defined outside the scope of the program, through operating system
functionalities.
*Fast DDS* relies on environment variables so that the user can easily customize the default settings of DDS
applications.
Please, refer to the [Environment variables](../env_vars/env_vars.md#env-vars) section for a complete list and description of the environment variables
affecting *Fast DDS*.
