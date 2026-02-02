.. _what_is_rtps:

What is RTPS?
-------------

The `Real-Time Publish Subscribe (RTPS) <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ protocol, developed to
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

.. figure:: /01-figures/fast_dds/getting_started/rtps_domain.svg
    :align: center

    RTPS high-level architecture

Communication revolves around **Topics**, which define and label the data being exchanged.
The topics do not belong to a specific participant.
The participant, through the RTPSWriters, makes changes in the data published under a topic, and through the RTPSReaders
receives the data associated with the topics to which it subscribes.
The communication unit is called **Change**, which represents an update in the data that is written under a Topic.
**RTPSReaders/RTPSWriters** register these changes on their **History**, a data structure that serves as a cache for
recent changes.

In the default configuration of *eProsima Fast DDS*, when you publish a `change` through a RTPSWriter endpoint, the
following steps happen behind the scenes:

1. The `change` is added to the RTPSWriter’s history cache.
2. The RTPSWriter sends the change to any RTPSReaders it knows about.
3. After receiving data, RTPSReaders update their history cache with the new change.

However, Fast DDS supports numerous configurations that allow you to change the behavior of RTPSWriters/RTPSReaders.
A modification in the default configuration of the RTPS entities implies a change in the data exchange flow between
RTPSWriters and RTPSReaders.
Moreover, by choosing Quality of Service (QoS) policies, you can affect how these history caches are managed in several
ways, but the communication loop remains the same.
You can continue reading section :ref:`rtps_layer` to learn more about the implementation of the RTPS protocol in Fast
DDS.



