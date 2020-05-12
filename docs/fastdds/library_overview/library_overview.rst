.. _library_overview:

Library Overview
================

Fast DDS is an efficient and high-performance implementation of the DDS specification, a data-centric communications
middleware (DCPS) for distributed application software.
This section reviews the architecture, operation and key features of Fast DDS.

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

  Fast DDS layer model architecture

The elements that comprise each of the layers shown in the figure above are explained below.

DDS Layer
^^^^^^^^^

Several key elements for communication are defined in the DDS layer of Fast DDS.
The user will create these elements in his/her application, thus incorporating DDS application elements and creating a
data-centric communication system.
Fast DDS, following the DDS specification, defines these elements involved in communication as **Entities**.

* **DomainId_t**.
  uint32_t data type which identifies the DDS domain.
  Each DomainParticipant will have an assigned DDS domain, so that DomainParticipants in the same domain can
  communicate, as well as isolate communications between DDS domains.
  This value must be given by the application developer when creating the DomainParticipants.
* **DomainParticipants**.
  Object containing other DDS entities such as publishers, subscribers, topics and multitopics.
  It is the entity that allows the creation of the previous entities it contains, as well as the configuration of their
  behavior.
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

This is all covered in more detail in the :ref:`dds_layer` section.

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

This is all covered in more detail in the :ref:`rtps_layer` section.

Transport layer
^^^^^^^^^^^^^^^

Fast DDS supports the implementation of applications over various transport protocols: UDPv4, UDPv6, TCPv4, TCPv6 and
Shared Memory Transport (SHM). By default, a DomainParticipant implements two transport protocols:

* **SHM**: for communications between DomainParticipants in the same machine.
* **UDPv4**: for inter machine communications.

The configuration of all supported transport protocols is detailed in the :ref:`comm-transports-configuration` section.

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
However, the user can define in his/her application periodic time-events by inheriting from the :class:`TimedEvent`
class.


Functionalities
---------------

Fast DDS has some added features that can be implemented and configured by the user in his/her application.
These are outlined below.

Discovery Protocols
^^^^^^^^^^^^^^^^^^^

The discovery protocols define the mechanisms by which DataWriters publishing under a given Topic, and DataReaders
subscribing to that same Topic are matched, so that they can start sharing data.
This applies at any point in the communication process.
Fast DDS provides the following discovery mechanisms:

* **Simple Discovery**.
  This is the default mechanism.
  Here the DomainParticipants are discovered individually at an early stage to subsequently match the DataWriter and
  DataReader they implement.
* **Static Discovery**.
  This implements the discovery of DomainParticipants to each other but it is possible to skip the discovery of the
  entities contained in each DomainParticipant if they are known in advance.
* **Server-Client Discovery**.
  This discovery mechanism uses a centralized discovery architecture, where servers act as a hubs for discovery meta
  traffic.
* **Manual Discovery**.
  This mechanism is only compatible with the RTPS layer.
  It allows the user to manually match and unmatch DomainParticipants, DataWriters, and DataReaders using whatever,
  external meta-information channel of its choice.

The detailed explanation and configuration of all the discovery protocols implemented in Fast DDS can be seen in
the :ref:`discovery` section.

Security
^^^^^^^^

Fast DDS can be configured to provide secure communications by implementing pluggable security at three levels:

* Authentication of remote DomainParticipants.
  The **DDS:Auth:PKI-DH** plugin provides authentication using a trusted Certificate
  Authority (CA) and ECDSA Digital Signature Algorithms to perform the mutual authentication.
  It also establishes a shared secret using Elliptic Curve Diffie-Hellman (ECDH) Key Agreement protocol.
* Access control of entities.
  The **DDS:Access:Permissions** plugin provides access control to DomainParticipants at the DDS Domain and Topic level.
* Encryption of data.
  The **DDS:Crypto:AES-GCM-GMAC** plugin provides authenticated encryption using Advanced Encryption Standard (AES) in
  Galois Counter Mode (AES-GCM).

You can continue reading the :ref:`security` section for more information.

Logging
^^^^^^^

Fast DDS provides an extensible Logging system.
:class:`Log` class is the entry point of the Logging system.
It exposes three macro definitions to ease its usage: ``logInfo``, ``logWarning`` and ``logError``.
Moreover, it allows the definition of new categories, in addition to those already available
(:class:`INFO_MSG`, :class:`WARN_MSG` and :class:`ERROR_MSG`).
It provides filtering by category using regular expressions, as well as control of the verbosity of the Logging system.
Details of the possible Logging system configurations can be found in the :ref:`here <dds_layer_core_logging>` section.


XML profiles configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^

Fast DDS offers the possibility to make changes in its default settings by using XML profile configuration files.
Thus, the behavior of the DDS Entities can be modified without the need for the user to implement any program source
code.

The user has XML tags for each of the API functionalities.
Therefore, it is possible to build and configure DomainParticipant profiles through the ``<participant>`` tag, or
the DataWriter and DataReader profiles associated to the ``<data_writer>`` and ``<data_reader>`` tags respectively.

For a better understanding of how to develop these XML profiles configuration files you can continue reading
the :ref:`xml_profiles` section.

Environment variables
^^^^^^^^^^^^^^^^^^^^^

Environment variables are those variables that are defined outside the scope of the program, through operating system
functionalities.
Fast DDS relies on two environment variables so that the user can easily customize the default settings of DDS
applications.
These two environment variables are as follows:

* ``FASTRTPS_DEFAULT_PROFILES_FILE``.
  Defines the location of the profile configuration XML files.

* ``ROS_DISCOVERY_SERVER``.
  Sets as the default discovery protocol the Server-Client Discovery.
  It lists the UDP addresses of the DomainParticipant that perform the server function.

You can see more information about Fast DDS environment variables in the :ref:`env_vars` section.
