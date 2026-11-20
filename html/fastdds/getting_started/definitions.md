<a id="what-is-dds"></a>

<a id="id1"></a>

# What is DDS?

The [Data Distribution Service (DDS)](https://www.omg.org/spec/DDS/About-DDS/)
is a data-centric communication protocol used for distributed software
application communications.
It describes the communications Application Programming Interfaces (APIs) and Communication Semantics that enable
communication between data providers and data consumers.

Since it is a Data-Centric Publish Subscribe (DCPS) model, three key application entities are defined in its
implementation: publication entities, which define the information-generating objects and their properties;
subscription entities, which define the information-consuming objects and their properties; and configuration entities
that define the types of information that are transmitted as topics, and create the publisher and subscriber with
its Quality of Service (QoS) properties, ensuring the correct performance of the above entities.

DDS uses QoS to define the behavioral characteristics of DDS Entities. QoS are comprised of individual QoS policies
(objects of type deriving from QoSPolicy). These are described in [Policy](../dds_layer/core/policy/policy.md#dds-layer-core-policy).

<a id="the-dcps-conceptual-model"></a>

## The DCPS conceptual model

In the DCPS model, four basic elements are defined for the development of a system of communicating applications.

* **Publisher**.
  It is the DCPS entity in charge of the creation and configuration of the **DataWriters** it implements.
  The **DataWriter** is the entity in charge of the actual publication of the messages.
  Each one will have an assigned **Topic** under which the messages are published.
  See [Publisher](../dds_layer/publisher/publisher.md#dds-layer-publisher) for further details.
* **Subscriber**.
  It is the DCPS Entity in charge of receiving the data published under the topics to which it subscribes.
  It serves one or more **DataReader** objects, which are responsible for communicating the availability of new data
  to the application.
  See [Subscriber](../dds_layer/subscriber/subscriber.md#dds-layer-subscriber) for further details.
* **Topic**.
  It is the entity that binds publications and subscriptions.
  It is unique within a DDS domain.
  Through the **TopicDescription**, it allows the uniformity of data types of publications and subscriptions.
  See [Topic](../dds_layer/topic/topic.md#dds-layer-topic) for further details.
* **Domain**.
  This is the concept used to link all publishers and subscribers, belonging to one or more applications,
  which exchange data under different topics.
  These individual applications that participate in a domain are called **DomainParticipant**.
  The DDS Domain is identified by a domain ID.
  The DomainParticipant defines the domain ID to specify the DDS domain to which it belongs.
  Two DomainParticipants with different IDs are not aware of each other’s presence in the network.
  Hence, several communication channels can be created.
  This is applied in scenarios where several DDS applications are involved, with their respective DomainParticipants
  communicating with each other, but these applications must not interfere.
  The **DomainParticipant** acts as a container for other DCPS Entities, acts as a factory for
  **Publisher**, **Subscriber** and **Topic** Entities, and provides administrative services in the domain.
  See [Domain](../dds_layer/domain/domain.md#dds-layer-domain) for further details.

These elements are shown in the figure below.

![image](01-figures/fast_dds/getting_started/dds_domain.svg)

*DCPS model entities in the DDS Domain.*

<a id="what-is-rtps"></a>

<a id="id2"></a>

# What is RTPS?

The [Real-Time Publish Subscribe (RTPS)](https://www.omg.org/spec/DDSI-RTPS/2.2/PDF) protocol, developed to
support DDS applications, is a publication-subscription communication middleware
over best-effort transports such as UDP/IP. Furthermore, Fast DDS provides support for TCP and
Shared Memory (SHM) transports.

It is designed to support both unicast and multicast communications.

At the top of RTPS, inherited from DDS, the **Domain** can be found, which defines a separate plane of communication.
Several domains can coexist at the same time independently.
A domain contains any number of **RTPSParticipants**, that is, elements capable of sending and receiving data.
To do this, the RTPSParticipants use their **Endpoints**:

* **RTPSWriter**: Endpoint able to send data.
* **RTPSReader**: Endpoint able to receive data.

A RTPSParticipant can have any number of writer and reader endpoints.

![image](01-figures/fast_dds/getting_started/rtps_domain.svg)

*RTPS high-level architecture*

Communication revolves around **Topics**, which define and label the data being exchanged.
The topics do not belong to a specific participant.
The participant, through the RTPSWriters, makes changes in the data published under a topic, and through the RTPSReaders
receives the data associated with the topics to which it subscribes.
The communication unit is called **Change**, which represents an update in the data that is written under a Topic.
**RTPSReaders/RTPSWriters** register these changes on their **History**, a data structure that serves as a cache for
recent changes.

In the default configuration of *eProsima Fast DDS*, when you publish a change through a RTPSWriter endpoint, the
following steps happen behind the scenes:

1. The change is added to the RTPSWriter’s history cache.
2. The RTPSWriter sends the change to any RTPSReaders it knows about.
3. After receiving data, RTPSReaders update their history cache with the new change.

However, Fast DDS supports numerous configurations that allow you to change the behavior of RTPSWriters/RTPSReaders.
A modification in the default configuration of the RTPS entities implies a change in the data exchange flow between
RTPSWriters and RTPSReaders.
Moreover, by choosing Quality of Service (QoS) policies, you can affect how these history caches are managed in several
ways, but the communication loop remains the same.
You can continue reading section [RTPS Layer](../rtps_layer/rtps_layer.md#rtps-layer) to learn more about the implementation of the RTPS protocol in Fast
DDS.
