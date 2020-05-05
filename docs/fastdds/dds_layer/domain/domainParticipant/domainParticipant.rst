.. _dds_layer_domainParticiant:

DomainParticipant
=================

A DomainParticipant is the entry point of the application to a domain. Every :ref:`dds_layer_domainParticiant`
is linked to a single domain from its creation, and contains all the Entities related to that domain. It also acts
as a factory for Publishers, Subscribers and Topics on that domain.

The behavior of the DomainParticipant can be modified with the QoS values
specified on :ref:`dds_layer_domainParticiantQos`. The QoS values can be set
at the creation of the DomainParticipant, or modified later with :func:`set_qos()` method.

Like other Entities, DomainParticipant accepts a Listener that will be notified of status changes on the DomainParticipant instance.


.. _dds_layer_domainParticiant_creation:

Creating a DomainParticipant
----------------------------

Creation of a DomainParticipant is done with the :func:`create_participant()` method on the
:ref:`dds_layer_domainParticiantFactory` singleton.

The only mandatory parameter is the DomainId that identifies the domain where the DomainParticipant
will be created.

Optional parameters include:

 * :ref:`dds_layer_domainParticiantQos` describing the behavior of the DomainParticipant. If not provided,
   the values of the :ref:`dds_layer_defaultDomainParticiantQos` are used.

 * A Listener derived from :ref:`dds_layer_domainParticiantListener`, implementing the callbacks that will be triggered
   in response to events and state changes on the DomainParticipant. By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the Listener. By default
   all events are enabled.

.. Maybe add documentation for create_participant_with_profile?

.. _dds_layer_defaultDomainParticiantQos:

Default DomainParticipantQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default DomainParticipantQos is the QoS definition that will be used when no :ref:`dds_layer_defaultDomainParticiantQos`
is specified at the moment of DomainParticipant creation. This default QoS can be modified at any time using the
:func:`set_default_participant_qos()` method on the :ref:`dds_layer_domainParticiantFactory` singleton. Once modified, all
DomainParticipant instances created without specifying a  :ref:`dds_layer_defaultDomainParticiantQos` value will be created
using the new default value. DomainParticipant instances created before the modification will not be affected.


.. _dds_layer_domainParticiant_deletion:

Deleting a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A DomainParticipant can be deleted with the :func:`delete_participant()` method on the
:ref:`_dds_layer_domainParticiantFactory` singleton.

A DomainParticipant can only be deleted if all domain Entities belonging to the participant
(Publisher, Subscriber or Entity) have already been deleted. Otherwise, the method will issue
an error and the DomainParticipant will not be deleted.


.. _dds_layer_domainParticiantQos:

DomainParticipantQos
--------------------

DomainParticipantQos controls the behavior of the :ref:`dds_layer_domainParticiant`.
Internally it contains the following QosPolicy objects:

+--------------------------+------------------------------+
| Name                     | QosPolicy class              |
+===============================+=========================+
| ``user_data_``           | UserDataQosPolicy            |
+--------------------------+------------------------------+
| ``entity_factory_``      | EntityFactoryQosPolicy       |
+--------------------------+------------------------------+
| ``allocation_``          | ParticipantResourceLimitsQos |
+--------------------------+------------------------------+
| ``properties_``          | PropertyPolicyQos            |
+--------------------------+------------------------------+
| ``wire_protocol_``       | WireProtocolConfigQos        |
+--------------------------+------------------------------+
| ``transport_``           | TransportConfigQos           |
+--------------------------+------------------------------+






            
            
            
            