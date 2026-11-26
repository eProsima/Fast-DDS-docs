<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-getting-started-questions"></a>

<a id="frequently-asked-getting-started-questions"></a>

# Frequently Asked Getting Started Questions

**What is DDS?**

<br/> The Data Distribution Service (DDS) is a data-centric communication protocol used for distributed software application communications.
For further information, refer to [What is DDS?](../../getting_started/definitions.md#what-is-dds).

---

**What are the key entities of a DDS?**

<br/> These are the basic entities of a DDS: [Publisher](../../dds_layer/publisher/publisher/publisher.md#dds-layer-publisher-publisher), [Subscriber](../../dds_layer/subscriber/subscriber/subscriber.md#dds-layer-subscriber-subscriber), [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader), [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), [Topic](../../dds_layer/topic/topic.md#dds-layer-topic), [domain](../../dds_layer/domain/domain.md#dds-layer-domain).
For further information, refer to [What is DDS?](../../getting_started/definitions.md#what-is-dds).

---

**What is the primary purpose of using Quality of Service (QoS) in DDS?**

<br/> DDS uses QoS to define the behavioral characteristics of DDS Entities.
QoS are comprised of individual QoS policies (objects of type deriving from QoSPolicy).
For further information, refer to [What is DDS?](../../getting_started/definitions.md#what-is-dds).

---

**What is the primary responsibility of a DDS Publisher entity?**

<br/> [`Publisher`](../../api_reference/dds_pim/publisher/publisher_class.md#_CPPv4N8eprosima7fastdds3dds9PublisherE). It is the Data-Centric Publish Subscribe entity in charge of the creation and configuration of the DataWriters it implements.
The  [`DataWriter`](../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE) is the entity in charge of the actual publication of the messages.
Each one will have an assigned Topic under which the messages are published.
For further information, refer to [Publisher](../../dds_layer/publisher/publisher.md#dds-layer-publisher).

---

**What is the primary responsibility of a DDS Subscriber entity?**

<br/> [`Subscriber`](../../api_reference/dds_pim/subscriber/subscriber_class.md#_CPPv4N8eprosima7fastdds3dds10SubscriberE). It is the DCPS [Entity](../../dds_layer/core/entity/entity.md#dds-layer-core-entity) in charge of receiving the data published under the topics to which it subscribes.
It serves one or more [`DataReader`](../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) objects, which are responsible for communicating the availability of new data to the application.
For further information, refer to [Subscriber](../../dds_layer/subscriber/subscriber.md#dds-layer-subscriber).

---

**What is the primary function of a DDS Domain?**

<br/> [domain](../../dds_layer/domain/domain.md#dds-layer-domain). This is the concept used to link all publishers and subscribers, belonging to one or more applications, which exchange data under different topics.
These individual applications that participate in a domain are called [`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE).
The DDS Domain is identified by a domain ID.
Domains create logical separations among the entities that share a common communication infrastructure.
They isolate applications running in the same domain from applications running on different domains.
For further information, refer to [Domain](../../dds_layer/domain/domain.md#dds-layer-domain).

---

**What is RTPS?**

<br/> The Real-Time Publish Subscribe (RTPS) protocol, developed to support DDS applications, is a publication-subscription communication middleware over transports such as UDP/IP.
RTPS supports unicast and multicast communications, organizing communication within independent domains containing RTPSParticipants, which use RTPSWriters to send data and RTPSReaders to receive data, all centered around Topics that label exchanged data.
For further information, refer to [What is RTPS?](../../getting_started/definitions.md#what-is-rtps).

---

**What type of communication does RTPS support?**

<br/> It is designed to support both unicast and multicast communications.
For further information, refer to [What is RTPS?](../../getting_started/definitions.md#what-is-rtps).

---

**What is the maximum number of endpoints that a single RTPSParticipant can have?**

<br/> A `RTPSParticipant` can have any number of writer and reader endpoints.
For further information, refer to [What is RTPS?](../../getting_started/definitions.md#what-is-rtps).

---

**What is the primary mechanism by which RTPS participants exchange data, and what role do topics play in this process?**

<br/> Communication revolves around Topics, which define and label the data being exchanged.
The topics do not belong to a specific participant.
The participant, through the RTPSWriters, makes changes in the data published under a topic, and through the RTPSReaders receives the data associated with the topics to which it subscribes.
The communication unit is called Change, which represents an update in the data that is written under a Topic.
RTPSReaders/RTPSWriters register these changes on their History, a data structure that serves as a cache for recent changes.
For further information, refer to [What is RTPS?](../../getting_started/definitions.md#what-is-rtps).

<br/>
