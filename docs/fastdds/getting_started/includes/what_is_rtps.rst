.. _what_is_rtps:

What is RTPS?
-------------

The `Real-Time Publish Subscribe (RTPS) <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ protocol, developed to
support DDS applications, is a publication-subscription communications protocol
over best-effort transports such as UDP/IP. Furthermore, Fast RTPS provides support for TCP and
Shared Memory (SHM) transports.

It is designed to support both unicast and multicast communications.

At the top of RTPS, inherited from DDS, the **Domain** can be found, which defines a separate plane of communication.
Several domains can coexist at the same time independently.
A domain contains any number of **RTPSParticipants**, that is, elements capable of sending and receiving data.
To do this, the participants use their **Endpoints**:

* **RTPSWriter**: Endpoint able to send data.
* **RTPSReader**: Endpoint able to receive data.

A RTPSParticipant can have any number of writer and reader endpoints.

.. figure:: /01-figures/fast_dds/getting_started/rtps_domain.svg
    :scale: 100 %
    :align: center

    RTPS high-level architecture

Communication revolves around **Topics**, which define the data being exchanged.
The topics do not belong to any particular participant; all participants follow the changes in the data *written* under
a topic, that is, the data published under a topic, so that they keep the day of the published data.
The communication unit is called **Change**, which represents an update in the data that is written under a Topic.
**RTPSReaders/RTPSWriters** register these changes on their **History**, a data structure that serves as a cache for
recent changes.

When you publish a change through a writer endpoint, the following steps happen behind the scenes:

* The change is added to the writer’s history cache.
* The writer informs any readers it knows about.
* Any interested (subscribed) readers request the change.
* After receiving data, readers update their history cache with the new change.

By choosing Quality of Service (QoS) policies, you can affect how these history caches are managed in several ways,
but the communication loop remains the same. You can read :ref:`configuration` for further details.


