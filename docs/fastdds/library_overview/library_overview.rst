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

Programming and execution model
-------------------------------

Fast DDS is concurrent and event-based.
The following explains the multithreading model that governs the operation of Fast DDS as well as the possible events.

Concurrency and multithreading
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS implements a concurrent multithreading system.
Each DomainParticipant spawns a set of threads to take care of background tasks such as logging, message reception, and
asynchronous communication.
This should not impact the way you use the library, i.e. the Fast DDS API is thread safe, so you can fearlessly call any
methods on the same DomainParticipant from different threads.
However, this multithreading implementation must be taken into account when external functions access to resources that
are modified by threads running internally in the library.
An example of this is the modified resources in the entity listener callbacks.
The following is a brief overview of how Fast DDS multithreading schedule work:

* Main thread: Managed by the application.
* Event thread: Each DomainParticipant owns one of these. It processes periodic and triggered time events.
* Asynchronous writer thread: This thread manages asynchronous writes for all DomainParticipants.
  Even for synchronous writers, some forms of communication must be initiated in the background.
* Reception threads: DomainParticipants spawn a thread for each reception channel, where the concept of a channel
  depends on the transport layer (e.g. a UDP port).

Event-driven architecture
^^^^^^^^^^^^^^^^^^^^^^^^^

There is a time-event system that enables Fast DDS to respond to certain conditions, as well as schedule periodic
operations.
Few of them are visible to the user since most are related to DDS and RTPS metadata.
However, the user can define in his/her application periodic time-events by inheriting from the :class:TimedEvent class.

