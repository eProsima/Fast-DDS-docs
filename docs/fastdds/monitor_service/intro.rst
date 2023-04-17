.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service:

Monitor Service
===============

The *Fast DDS* Monitor Service is an extension of Fast DDS that
grants user the ability to collect data about the entities existing within
a particular |domain| (i.e |DomainParticipants|, |DataReaders|, |DataWriters|)
as well as the capability of predicting possible misconfigurations among them.

The :ref:`monitor_service` can be particularly useful in the following scenarios:

*  To retrieve an entity graph of a |domain|, overcoming situations
   in which a sub-set of |DomainParticipants| are not directly visible, for instance, because they
   have multicast traffic disabled, discovering each other using an initial peers list.
   By means of the :ref:`monitor_service` it is possible to recover the information
   of those |DomainParticipants| and their entities (given that there is at least one
   |DomainParticipant| bridging the unicast and multicast groups).
*  Troubleshooting issues regarding discovery or entity-matching, leveraging the information
   of the current locators in use, for example.
*  Checking whether the Quality of Service *QoS* attributes of a pair of known entities (Reader, Writer)
   are compatible, or not, using the service in conjunction with the |DomainParticipant::check-compatible-qos-api|.

Enabling the service makes each |DomainParticipant| publish the collection of
entities known through discovery.
In addition, a Remote Procedural Call *RPC* Server is provided so that information of a requested entity
can be retrieved.
An RPC server works in a kind of client/server model where the RPC server provides
a service to remotely connected RPC clients.
The RPC Client sends a request, the RPC Server processes it, returning a response back to the client.

The :ref:`monitor_service` is disabled by default, as it may entail a performance cost.
Further information on the :ref:`monitor_service` topics and how to configure it, is described
in the following sections:

.. toctree::
    :titlesonly:

    /fastdds/monitor_service/monitor_service_topics
    /fastdds/monitor_service/monitor_service_configuration
