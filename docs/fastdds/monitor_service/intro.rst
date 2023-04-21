.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include

.. _monitor_service:

Monitor Service
===============

The *Fast DDS* Monitor Service is an extension of Fast DDS that
grants user the ability to collect data about the entities existing within
a particular |domain| (i.e |DomainParticipants|, |DataReaders|, |DataWriters|)
as well as the capability of detecting possible misconfigurations among them.

Keywords
^^^^^^^^
* **Request/Response service or RPC**: An RPC is initiated by the client, which sends a request message to a
  known remote server to execute a specified procedure with supplied parameters.
  Then, the remote server sends a response back to the client, and the application continues its process.

* **Proxy**: An entity that acts on behalf of another entity.

* **Proxy Data**: The way in which a Proxy can be described.

* **Monitoring Information**: The collection of different sources of information and statuses of an entity,
  including: the Proxy Data, incompatible_qos, connections, liveliness, deadlines missed, inconsistent
  topics and sample lost status.

Description
^^^^^^^^^^^

Enabling the service makes each |DomainParticipant| publish the collection of its local entities.
In addition, a Remote Procedural Call *RPC* Server is provided so that information of a requested local entity
can be retrieved.

The :ref:`monitor_service` is disabled by default, as it may entail a performance cost.
Further information on the :ref:`monitor_service` topics and how to configure it, is described
in the following sections.

The :ref:`monitor_service` is available in both the :ref:`DDS Layer <dds_layer>` and :ref:`RTPS Layer <rtps_layer>`.

.. note::

   If the service is activated within a :ref:`RTPS <rtps_layer>` context, not all the information requested could
   be returned by the service.

Use Cases
^^^^^^^^^^^

The :ref:`monitor_service` can be particularly useful in the following scenarios:

*  Querying a remote |DomainParticipant| for its ``Monitoring Information`` targetting any of its local
   entities in order to extend the default discovery information (see :ref:`discovery`) about it.
*  Troubleshooting issues regarding discovery or entity-matching, leveraging the information
   of the current locators in use, for example.

.. toctree::
    :titlesonly:

    /fastdds/monitor_service/monitor_service_topics
    /fastdds/monitor_service/monitor_service_configuration
