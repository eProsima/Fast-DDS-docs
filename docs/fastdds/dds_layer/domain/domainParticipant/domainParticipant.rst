.. _dds_layer_domainParticiant:

DomainParticipant
=================

A DomainParticipant is the entry point of the application to a domain.
Every :ref:`dds_layer_domainParticiant` is linked to a single domain from its creation,
and contains all the Entities related to that domain.
It also acts as a factory for Publishers, Subscribers and Topics on that domain.

The behavior of the DomainParticipant can be modified with the QoS values
specified on :ref:`dds_layer_domainParticiantQos`.
The QoS values can be set at the creation of the DomainParticipant,
or modified later with :func:`set_qos()` method.

Like other Entities, DomainParticipant accepts a Listener that will be notified of
status changes on the DomainParticipant instance.


.. _dds_layer_domainParticiant_creation:

Creating a DomainParticipant
----------------------------

.. _dds_layer_domainParticiant_creation_qos:

QoS based creation of a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Creation of a DomainParticipant is done with the :func:`create_participant()` method on the
:ref:`dds_layer_domainParticiantFactory` singleton.

The only mandatory parameter is the DomainId that identifies the domain where the DomainParticipant
will be created.

Optional parameters include:

 * :ref:`dds_layer_domainParticiantQos` describing the behavior of the DomainParticipant.
   If no value is provided, or if the provided value is :class:`PARTICIPANT_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDomainParticiantQos` is used.

 * A Listener derived from :ref:`dds_layer_domainParticiantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DomainParticipant.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the Listener.
   By default all events are enabled.


.. _dds_layer_domainParticiant_creation_profile:

Profile based creation of a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Creation of a DomainParticipant based on a profile is done with the :func:`create_participant_with_profile()`
method on the :ref:`dds_layer_domainParticiantFactory` singleton.

This method takes two mandatory parameters:

 * The DomainId that identifies the domain where the DomainParticipant will be created.
 
 * The name of the profile to be applied to the DomainParticipant.

Optional parameters include a Listener and a StatusMask (see :ref:`dds_layer_domainParticiant_creation_qos`).


.. _dds_layer_domainParticiant_deletion:

Deleting a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A DomainParticipant can be deleted with the :func:`delete_participant()` method on the
:ref:`_dds_layer_domainParticiantFactory` singleton.

A DomainParticipant can only be deleted if all domain Entities belonging to the participant
(Publisher, Subscriber or Entity) have already been deleted.
Otherwise, the method will issue an error and the DomainParticipant will not be deleted.


.. _dds_layer_defaultDomainParticiantQos:

Default DomainParticipantQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default DomainParticipantQos is the QoS definition that will be used when no other
:ref:`dds_layer_defaultDomainParticiantQos` is provided to the DomainParticipant, either during
creation of through the :func:`set_participant_qos()` method.

The current default DomainParticipantQos can be retrieved using the :func:`get_default_participant_qos()` method
on the :ref:`dds_layer_domainParticiantFactory` singleton.
It can also be modified at any time using the :func:`set_default_participant_qos()` method
on the :ref:`dds_layer_domainParticiantFactory` singleton.
Once modified, all new instances of DomainParticipant created without specifying a
:ref:`dds_layer_defaultDomainParticiantQos` value will use the new default value.
DomainParticipant instances created before the modification will not be affected.

The special symbol :class:`PARTICIPANT_QOS_DEFAULT` can be used as QoS parameter on :func:`create_participant()`
or :func:`set_participant_qos()` methods to indicate that the current default DomainParticipantQos should be used.

:func:`set_default_participant_qos()` method also accepts the symbol :class:`PARTICIPANT_QOS_DEFAULT`
as input parameter.
This will reset the current default DomainParticipantQos to default constructed values.


.. _dds_layer_domainParticiantQos:

DomainParticipantQos
--------------------

DomainParticipantQos controls the behavior of the :ref:`dds_layer_domainParticiant`.
Internally it contains the following QosPolicy objects:

+--------------------------+------------------------------+----------+
| Name                     | QosPolicy class              | Mutable  |
+===============================+=========================+==========+
| ``user_data_``           | UserDataQosPolicy            | yes      |
+--------------------------+------------------------------+----------+
| ``entity_factory_``      | EntityFactoryQosPolicy       | yes      |
+--------------------------+------------------------------+----------+
| ``allocation_``          | ParticipantResourceLimitsQos | no       |
+--------------------------+------------------------------+----------+
| ``properties_``          | PropertyPolicyQos            | no       |
+--------------------------+------------------------------+----------+
| ``wire_protocol_``       | WireProtocolConfigQos        | no       |
+--------------------------+------------------------------+----------+
| ``transport_``           | TransportConfigQos           | no       |
+--------------------------+------------------------------+----------+

The symbol :class:`PARTICIPANT_QOS_DEFAULT` represents a default constructed DomainParticipantQos,
where all its QosPolicy values have their default value.

The QoS value of a previously created DomainParticipant can be modified using the :func:`set_participant_qos()` method.
Trying to modify an immutable QosPolicy on an already enabled DomainParticipant will result on an error.
In such case, no changes will be applied and the DomainParticipant will keep its previous DomainParticipantQos.



