Architecture
------------

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
  Implementation of the `Real-Time Publish-Subscribe (RTPS) protocol <https://www.omg.org/spec/DDSI-RTPS/2.2>`_
  for interoperability with DDS applications.
  This layer acts as an abstraction layer of the transport layer.
* **Transport Layer**.
  *Fast DDS* can be used over various transport protocols such as unreliable transport protocols (UDP), reliable
  transport protocols (TCP), or shared memory transport protocols (SHM).

.. figure:: /01-figures/fast_dds/library_overview/library_overview.svg
  :align: center

  *Fast DDS* layer model architecture

DDS Layer
^^^^^^^^^

Several key elements for communication are defined in the DDS layer of *Fast DDS*.
The user will create these elements in their application, thus incorporating DDS application elements and creating a
data-centric communication system.
*Fast DDS*, following the DDS specification, defines these elements involved in communication as **Entities**.
A DDS **Entity** is any object that supports Quality of Service configuration (QoS), and that implements a listener.

* **QoS**.
  The mechanism by which the behavior of each of the entities is defined.
* **Listener**.
  The mechanism by which the entities are notified of the possible events that arise during the application's execution.

Below are listed the DDS Entities together with their description and functionality.
For a more detailed explanation of each entity, their QoS, and their listeners, please refer to :ref:`dds_layer`
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
  When the DataWriter proceeds to publish data under a specific Topic, it actually creates a `change` in this data.
  It is this `change` that is registered in the History.
  These `changes` are then sent to the DataReader that subscribes to that specific topic.
* **Subscriber**.
  The Subscriber subscribes to a topic using a DataReader, which reads the data from the transport.
  It is the entity that creates and configures the DataReader entities it contains, and may contain one or more
  DataReader entities.
* **DataReader**.
  It is the entity that subscribes to the topics for the reception of publications.
  The user must provide a subscription Topic when creating this entity.
  A DataReader receives the messages as changes in its DataReaderHistory.
* **DataReaderHistory**.
  It contains the `changes` in the data-objects that the DataReader receives as a result of subscribing to a certain
  Topic.
* **Topic**. Entity that binds Publishers' DataWriters with Subscribers' DataReaders.

RTPS layer
^^^^^^^^^^

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

For a more detailed explanation of each entity, their attributes, and their listeners, please refer to :ref:`rtps_layer`
section.

Transport layer
^^^^^^^^^^^^^^^

*Fast DDS* supports the implementation of applications over various transport protocols.
Those are UDPv4, UDPv6, TCPv4, TCPv6 and Shared Memory Transport (SHM).
By default, a DomainParticipant implements a UDPv4 and a SHM transport protocol.
The configuration of all supported transport protocols is detailed in the :ref:`comm-transports-configuration` section.

