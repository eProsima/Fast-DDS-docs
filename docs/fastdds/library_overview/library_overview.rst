.. _library_overview:

Library Overview
================

Fast DDS is an efficient and high-performance implementation of the DDS specification, a data-centric communications
midddleware (DCPS) for distributed application software.
This section reviews the architecture, operation and key concepts of Fast DDS.

Architecture
------------

The architecture of Fast DDS is shown in the figure below. A layer model with the following different environments can
be seen.

* **Application layer**.
  The user application that makes use of the Fast DDS API for the implementation of communications in
  distributed systems.
* **Fast DDS layer**.
  Robust implementation of the DDS communications middleware.
  It allows the development of one or more DDS domains in which DomainParticipants within the same domain
  exchange messages by publishing under a domain topic and subscribing to a it.
* **RTPS layer**.
  Implementation of the Real-Time Publish-Subscribe (RTPS) protocol for interoperability with DDS applications.
  This layer acts as the DDS abstraction layer of the transport layer.
* **Transport Layer**.
  Fast DDS can be implemented over various transport protocols such as unreliable transport protocols (UDP), reliable
  transport protocols (TCP), and shared memory transport protocols (SHM).

.. figure:: /01-figures/fast_dds/library_overview/library_overview.svg
  :align: center

  Fast DDS model layer architecture

The elements that comprise each of the layers shown in the figure above are explained below.

DDS Layer
^^^^^^^^^

Several key elements for communication are defined in the DDS layer of Fast DDS.
The user will create these elements in his/her application, thus incorporating DDS application elements and creating a
data-centric communication system.
Fast DDS, following the DDS specification, defines these elements involved in communication as **Entities**.

* **DomainId_t**.
  uint32_t data type which identifies the DDS domain.
  Each DomainParticipant will have an assigned DDS domain, so that DomainParticipants in the same domain can communicate, as well as isolate communications between DDS domains.
  This value must be given by the application developer when creating the DomainParticipants.
* **DomainParticipants**.
  Object containing other DDS entities such as publishers, subscribers, topics and multitopics.
  It is the entity that allows the creation of the previous entities it contains, as well as the configuration of their behavior.
* **Publisher**.
  It is the entity that creates and configures the DataWriter entities it contains.
  It may contain one or more DataWriter entities.
* **DataWriter**.
  It is the entity in charge of publishing messages.
  The user must provide a Topic when creating this entity which will be the Topic under which the data will be
  published.
  Publication is done by writing the data-objects as a change in the DataWriterHistory.
* **DataWriterHistory**.
  It contains the data objects that are changed by the DataWriter and that are going to be served to the DataReader
  that subscribes to the topic under which the DataWriter publishes.
* **Subscriber**.
  It is the entity that creates and configures the DataReader entities it contains.
  It may contain one or more DataReader entities.
* **DataReader**.
  It is the entity that subscribes to the topics for the reception of publications.
  The user must provide a subscription Topic when creating this entity.
  A DataReader receives the messages as changes in its HistoryDataReader.
* **DataReaderHistory**.
  It contains the changes in the data-objects made by the DataWriter that publishes under the topic to which the
  DataReader subscribes.
* **Topic**. Entity that binds Publishers' DataWriters with Subscribers' DataReaders.


The aforementioned **Entities** represent any object that supports QoS, and a listener.

* **QoS**.
  The mechanism by which the behavior of each of the entities is defined.
* **Listener**.
  The mechanism by which the entities are notified of the possible events that arise during the application's execution.

RTPS layer
^^^^^^^^^^

As mentioned above, the RTPS protocol in Fast DDS allows the abstraction of DDS application entities from the transport
layer.
According to the graph shown above, the RTPS layer has four main **Entities**.

* **RTPSDomain**.
  It is the extension of the DDS domain to the RTPS protocol.
* **RTPSParticipant**.
  Entity containing other RTPS entities. It allows the configuration and creation of the entities it contains.
* **RTPSWriter**.
  The source of the messages. It reads the changes written in the DataWriterHistory and transmits these changes to the
  RTPSReader it has previously matched.
* **RTPSReader**.
  Receiving entity of the messages. It writes the changes reported by the RTPSWriter in the DataReaderHistory.

Transport layer
^^^^^^^^^^^^^^^

Fast DDS supports the implementation of applications over various transport protocols: UDPv4, UDPv6, TCPv4, TCPv6 and
Shared Memory Transport (SHM). By default, a DomainParticipant implements two transport protocols:

* **SHM**: for communications between DomainParticipants in the same machine.
* **UDPv4**: for inter machine communications.

