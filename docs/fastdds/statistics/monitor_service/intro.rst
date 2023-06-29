.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

Introduction
============

The ``Monitor Service`` targets any application implementing the subscription side of
the :ref:`Monitor Service Status Topic <monitor_service_topics>`,
giving the possibility of retrieving the :ref:`Monitoring Information <monitor_service_keywords>`
of the local entities (incompatible QoS, deadlines missed,
active connections,...).

.. _monitor_service_keywords:

Keywords
^^^^^^^^

* **Proxy**: An entity that acts on behalf of another entity.

* **Proxy Data**: The way in which a Proxy can be described.

* **Monitoring Information**: The collection of different sources of information and statuses of an entity,
  including: the Proxy Data, incompatible qos, connections, liveliness, deadlines missed, inconsistent
  topics and lost sample status.

Description
^^^^^^^^^^^

Enabling the service makes each |DomainParticipant| publish its local entities, each one with its related
``Monitoring Information``.

The :ref:`monitor_service` is disabled by default, as it may entail a performance cost.
Further information on the :ref:`monitor_service` topics and how to configure it is described
in the following sections.

The :ref:`monitor_service` is available in both the :ref:`DDS Layer <dds_layer>` and :ref:`RTPS Layer <rtps_layer>`.


.. _monitor_service_in_rtps_note:

.. note::

   If the service is activated within a :ref:`RTPS <rtps_layer>` context, not all the ``Monitoring Information``
   may be published by the service.

Use Cases
^^^^^^^^^

The :ref:`monitor_service` can be particularly useful in the following scenarios:

*  Collecting the ``Monitoring Information`` of any local entity of a remote |DomainParticipant|
   in order to extend the default discovery information (see :ref:`discovery`) about it.
*  Troubleshooting issues regarding discovery or entity-matching, leveraging the information
   of the current locators in use, for example.
*  Recreating an entity graph of a certain domain given that all participants are able to discover each other.
