.. _dds_layer_domainParticipant:

DomainParticipant
=================

A :class:`DomainParticipant` is the entry point of the application to a domain.
Every :class:`DomainParticipant` is linked to a single domain from its creation,
and contains all the Entities related to that domain.
It also acts as a factory for :ref:`dds_layer_publisher_publisher`, :ref:`dds_layer_subscriber_subscriber`
and :ref:`dds_layer_topic_topic`.

The behavior of the :class:`DomainParticipant` can be modified with the QoS values
specified on :ref:`dds_layer_domainParticipantQos`.
The QoS values can be set at the creation of the :class:`DomainParticipant`,
or modified later with :func:`set_qos()` member function.

As an Entity, :class:`DomainParticipant` accepts a :ref:`dds_layer_domainParticipantListener`
that will be notified of status changes on the :class:`DomainParticipant` instance.


.. _dds_layer_domainParticipantQos:

DomainParticipantQos
--------------------

:class:`DomainParticipantQos` controls the behavior of the :ref:`dds_layer_domainParticipant`.
Internally it contains the following :class:`QosPolicy` objects:

+------------------------------+------------------------+----------+
| QosPolicy class              | Accessor/Mutator       | Mutable  |
+==============================+========================+==========+
| UserDataQosPolicy            | :func:`user_data`      | Yes      |
+------------------------------+------------------------+----------+
| EntityFactoryQosPolicy       | :func:`entity_factory` | Yes      |
+------------------------------+------------------------+----------+
| ParticipantResourceLimitsQos | :func:`allocation`     | No       |
+------------------------------+------------------------+----------+
| PropertyPolicyQos            | :func:`properties`     | No       |
+------------------------------+------------------------+----------+
| WireProtocolConfigQos        | :func:`wire_protocol`  | No       |
+------------------------------+------------------------+----------+
| TransportConfigQos           | :func:`transport`      | No       |
+------------------------------+------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

The QoS value of a previously created :ref:`dds_layer_domainParticipant` can be modified using the
:func:`set_qos()` member function.
Trying to modify an immutable :class:`QosPolicy` on an already enabled :ref:`dds_layer_domainParticipant`
will result on an error.
In such case, no changes will be applied and the :ref:`dds_layer_domainParticipant` will keep its
previous :class:`DomainParticipantQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultDomainParticipantQos:

Default DomainParticipantQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_domainParticipantQos` refers to the value returned by the
:func:`get_default_participant_qos()` member function on the :ref:`dds_layer_domainParticipantFactory` singleton.
The special value :class:`PARTICIPANT_QOS_DEFAULT` can be used as QoS argument on :func:`create_participant()`
or :func:`set_qos()` member functions to indicate that the current default :ref:`dds_layer_domainParticipantQos`
should be used.

When the system starts, the default :ref:`dds_layer_domainParticipantQos` is equivalent to the default constructed
value :func:`DomainParticipantQos()`.
The default :ref:`dds_layer_domainParticipantQos` can be modified at any time using the
:func:`set_default_participant_qos()` member function on the :ref:`dds_layer_domainParticipantFactory` singleton.
Modifying the default :ref:`dds_layer_domainParticipantQos` will not affect already existing
:ref:`dds_layer_domainParticipant` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DOMAINPARTICIPANTQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_participant_qos()` member function also accepts the value :class:`PARTICIPANT_QOS_DEFAULT`
as input argument.
This will reset the current default :ref:`dds_layer_domainParticipantQos` to the default constructed value
:func:`DomainParticipantQos()`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8


