.. _dds_layer_domainParticipant:

DomainParticipant
=================

A DomainParticipant is the entry point of the application to a domain.
Every DomainParticipant is linked to a single domain from its creation,
and contains all the Entities related to that domain.
It also acts as a factory for :ref:`dds_layer_publisher_publisher`, :ref:`dds_layer_subscriber_subscriber`
and :ref:`dds_layer_topic_topic`.

The behavior of the DomainParticipant can be modified with the QoS values
specified on :ref:`dds_layer_domainParticipantQos`.
The QoS values can be set at the creation of the DomainParticipant,
or modified later with :func:`set_qos()` method.

Like other Entities, DomainParticipant accepts a Listener (:ref:`dds_layer_domainParticipantListener`)
that will be notified of status changes on the DomainParticipant instance.


.. _dds_layer_domainParticipantQos:

DomainParticipantQos
--------------------

DomainParticipantQos controls the behavior of the :ref:`dds_layer_domainParticipant`.
Internally it contains the following QosPolicy objects:

+--------------------------+------------------------------+----------+
| Name                     | QosPolicy class              | Mutable  |
+==========================+==============================+==========+
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

Refer to the detailed description of each QosPolicy class for more information about their usage and
default values.

The QoS value of a previously created :ref:`dds_layer_domainParticipant` can be modified using the
:func:`set_qos()` method.
Trying to modify an immutable QosPolicy on an already enabled :ref:`dds_layer_domainParticipant`
will result on an error.
In such case, no changes will be applied and the :ref:`dds_layer_domainParticipant` will keep its
previous DomainParticipantQos.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS
   :end-before: //!


.. _dds_layer_defaultDomainParticipantQos:

Default DomainParticipantQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_domainParticipantQos` refers to the value returned by the
:func:`get_default_participant_qos()` method on the :ref:`dds_layer_domainParticipantFactory` singleton.
The special symbol :class:`PARTICIPANT_QOS_DEFAULT` can be used as QoS parameter on :func:`create_participant()`
or :func:`set_qos()` methods to indicate that the current default :ref:`dds_layer_domainParticipantQos` should be used.

When the system starts, the default :ref:`dds_layer_domainParticipantQos` is equivalent to the default constructed
value :func:`DomainParticipantQos()`.
The default :ref:`dds_layer_domainParticipantQos` can be modified at any time using the
:func:`set_default_participant_qos()` method on the :ref:`dds_layer_domainParticipantFactory` singleton.
Modifying the default :ref:`dds_layer_domainParticipantQos` will not affect already existing
:ref:`dds_layer_domainParticipant` instances unless their QoS is modified with
:func:`set_qos(PARTICIPANT_QOS_DEFAULT)`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DOMAINPARTICIPANTQOS
   :end-before: //!

:func:`set_default_participant_qos()` method also accepts the symbol :class:`PARTICIPANT_QOS_DEFAULT`
as input parameter.
This will reset the current default :ref:`dds_layer_domainParticipantQos` to the default constructed value
:func:`DomainParticipantQos()`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS_TO_DEFAULT
   :end-before: //!


.. _dds_layer_domainParticipant_creation:

Creating a DomainParticipant
============================

Creation of a :ref:`dds_layer_domainParticipant` is done with the :func:`create_participant()` method on the
:ref:`dds_layer_domainParticipantFactory` singleton, that acts as a factory for the :ref:`dds_layer_domainParticipant`.

Mandatory parameters are:

 * The domainId that identifies the domain where the :ref:`dds_layer_domainParticipant` will be created.

 * The :ref:`dds_layer_domainParticipantQos` describing the behavior of the :ref:`dds_layer_domainParticipant`.
   If the provided value is :class:`TOPIC_QOS_DEFAULT`, the value of the :ref:`dds_layer_domainParticipantQos` is used.

Optional parameters are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_domainParticipant`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

:func:`create_participant()` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DOMAINPARTICIPANT
   :end-before: //!


.. _dds_layer_domainParticipant_creation_profile:

Profile based creation of a DomainParticipant
---------------------------------------------

Instead of using a :ref:`dds_layer_domainParticipantQos`, the name of a profile
can be used to create a :ref:`dds_layer_domainParticipant` with the :func:`create_participant_with_profile()`
method on the :ref:`dds_layer_domainParticipantFactory` singleton.

Mandatory parameters are:

 * The DomainId that identifies the domain where the :ref:`dds_layer_domainParticipant` will be created.

 * The name of the profile to be applied to the :ref:`dds_layer_domainParticipant`.

Optional parameters are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_domainParticipant`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

:func:`create_participant_with_profile()` will return a null pointer if there was an error during the operation, e.g
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_DOMAINPARTICIPANT
   :end-before: //!

.. _dds_layer_domainParticipant_deletion:

Deleting a DomainParticipant
----------------------------

A :ref:`dds_layer_domainParticipant` can be deleted with the :func:`delete_participant()` method on the
:ref:`dds_layer_domainParticipantFactory` singleton.

.. note::

   A DomainParticipant can only be deleted if all domain Entities belonging to the participant
   (Publisher, Subscriber or Topic) have already been deleted.
   Otherwise, the method will issue an error and the DomainParticipant will not be deleted.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DOMAINPARTICIPANT
   :end-before: //!

