.. _dds_layer_domainParticipant:

DomainParticipant
=================

A DomainParticipant is the entry point of the application to a domain.
Every :ref:`dds_layer_domainParticipant` is linked to a single domain from its creation,
and contains all the Entities related to that domain.
It also acts as a factory for Publishers, Subscribers and Topics on that domain.

The behavior of the DomainParticipant can be modified with the QoS values
specified on :ref:`dds_layer_domainParticipantQos`.
The QoS values can be set at the creation of the DomainParticipant,
or modified later with :func:`set_qos()` method.

Like other Entities, DomainParticipant accepts a Listener that will be notified of
status changes on the DomainParticipant instance.


.. _dds_layer_domainParticipant_creation:

Creating a DomainParticipant
----------------------------

.. _dds_layer_domainParticipant_creation_qos:

QoS based creation of a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Creation of a DomainParticipant is done with the :func:`create_participant()` method on the
:ref:`dds_layer_domainParticipantFactory` singleton.

The only mandatory parameter is the DomainId that identifies the domain where the DomainParticipant
will be created.

Optional parameters include:

 * :ref:`dds_layer_domainParticipantQos` describing the behavior of the DomainParticipant.
   If no value is provided, or if the provided value is :class:`PARTICIPANT_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultDomainParticipantQos` is used.

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DomainParticipant.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the Listener.
   By default all events are enabled.

:func:`create_participant()` will return a null pointer if there was an error during the operation.
For example, if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DOMAINPARTICIPANT
   :end-before: //!


.. _dds_layer_domainParticipant_creation_profile:

Profile based creation of a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Creation of a DomainParticipant based on a profile is done with the :func:`create_participant_with_profile()`
method on the :ref:`dds_layer_domainParticipantFactory` singleton.

This method takes two mandatory parameters:

 * The DomainId that identifies the domain where the DomainParticipant will be created.
 
 * The name of the profile to be applied to the DomainParticipant.

Optional parameters include a Listener and a StatusMask (see :ref:`dds_layer_domainParticipant_creation_qos`).

:func:`create_participant_with_profile()` will return a null pointer if there was an error during the operation.
For example, if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   CML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.


.. _dds_layer_domainParticipant_deletion:

Deleting a DomainParticipant
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A DomainParticipant can be deleted with the :func:`delete_participant()` method on the
:ref:`dds_layer_domainParticipantFactory` singleton.

A DomainParticipant can only be deleted if all domain Entities belonging to the participant
(Publisher, Subscriber or Entity) have already been deleted.
Otherwise, the method will issue an error and the DomainParticipant will not be deleted.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DOMAINPARTICIPANT
   :end-before: //!

.. _dds_layer_domainParticipantQos:

DomainParticipantQos
--------------------

DomainParticipantQos controls the behavior of the :ref:`dds_layer_domainParticipant`.
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

The QoS value of a previously created DomainParticipant can be modified using the :func:`set_participant_qos()` method.
Trying to modify an immutable QosPolicy on an already enabled DomainParticipant will result on an error.
In such case, no changes will be applied and the DomainParticipant will keep its previous DomainParticipantQos.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS
   :end-before: //!


.. _dds_layer_defaultDomainParticipantQos:

Default DomainParticipantQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default DomainParticipantQos is the QoS definition that will be used when no other
:ref:`dds_layer_defaultDomainParticipantQos` is provided to the DomainParticipant, either during
creation of through the :func:`set_participant_qos()` method.

The current default DomainParticipantQos can be retrieved using the :func:`get_default_participant_qos()` method
on the :ref:`dds_layer_domainParticipantFactory` singleton.
It can also be modified at any time using the :func:`set_default_participant_qos()` method
on the :ref:`dds_layer_domainParticipantFactory` singleton.
Once modified, all new instances of DomainParticipant created without specifying a
:ref:`dds_layer_defaultDomainParticipantQos` value will use the new default value.
DomainParticipant instances created before the modification will not be affected.

The special symbol :class:`PARTICIPANT_QOS_DEFAULT` can be used as QoS parameter on :func:`create_participant()`
or :func:`set_participant_qos()` methods to indicate that the current default DomainParticipantQos should be used.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DOMAINPARTICIPANTQOS
   :end-before: //!

:func:`set_default_participant_qos()` method also accepts the symbol :class:`PARTICIPANT_QOS_DEFAULT`
as input parameter.
This will reset the current default DomainParticipantQos to default constructed values.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS_TO_DEFAULT
   :end-before: //!

