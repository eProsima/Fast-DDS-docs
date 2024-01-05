.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_domainParticipant:

DomainParticipant
=================

A |DomainParticipant| is the entry point of the application to a domain.
Every DomainParticipant is linked to a single domain from its creation,
and contains all the Entities related to that domain.
It also acts as a factory for :ref:`dds_layer_publisher_publisher`, :ref:`dds_layer_subscriber_subscriber`
and :ref:`dds_layer_topic_topic`.

The behavior of the DomainParticipant can be modified with the QoS values
specified on DomainParticipantQos.
The QoS values can be set at the creation of the DomainParticipant,
or modified later with |DomainParticipant::set_qos-api| member function.

As an Entity, DomainParticipant accepts a :ref:`dds_layer_domainParticipantListener`
that will be notified of status changes on the DomainParticipant instance.


.. _dds_layer_domainParticipantQos:

DomainParticipantQos
--------------------

|DomainParticipantQos-api| controls the behavior of the DomainParticipant.
Internally it contains the following |QosPolicy-api| objects:

.. list-table::
   :header-rows: 1

   * - QosPolicy class
     - Accessor/Mutator
     - Mutable
   * - |UserDataQosPolicy|
     - |DomainParticipantQos::user_data-api|
     - Yes
   * - |EntityFactoryQosPolicy|
     - |DomainParticipantQos::entity_factory-api|
     - Yes
   * - |ParticipantResourceLimitsQos|
     - |DomainParticipantQos::allocation-api|
     - No
   * - |PropertyPolicyQos|
     - |DomainParticipantQos::properties-api|
     - No
   * - |WireProtocolConfigQos|
     - |DomainParticipantQos::wire_protocol-api|
     - No*
   * - |TransportConfigQos|
     - |DomainParticipantQos::transport-api| and
       |DomainParticipantQos::setup_transports-api|
     - No
   * - |FlowControllersQos|
     - |DomainParticipantQos::flow_controllers-api|
     - No

.. Important::
    The only mutable field in |WireProtocolConfigQos| is |m_DiscoveryServers|, which is contained in
    |BuiltinAttributes::discovery_config-api| within |WireProtocolConfigQos::builtin-api| (see
    :ref:`DS_modify_server_list`).

Refer to the detailed description of each QosPolicy class for more information about their usage and
default values.

The QoS value of a previously created DomainParticipant can be modified using the
|DomainParticipant::set_qos-api| member function.
Trying to modify an immutable QosPolicy on an already enabled DomainParticipant
will result on an error.
In such case, no changes will be applied and the DomainParticipant will keep its
previous DomainParticipantQos.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultDomainParticipantQos:

Default DomainParticipantQos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The default DomainParticipantQos refers to the value returned by the
|DomainParticipantFactory::get_default_participant_qos-api| member function on the
:ref:`dds_layer_domainParticipantFactory` singleton.
The special value ``PARTICIPANT_QOS_DEFAULT`` can be used as QoS argument on
|DomainParticipantFactory::create_participant-api|
or |DomainParticipant::set_qos-api| member functions to indicate that the current default
DomainParticipantQos should be used.

When the system starts, the default DomainParticipantQos is equivalent to the default constructed
value |DomainParticipantQos::DomainParticipantQos-api|.
The default DomainParticipantQos can be modified at any time using the
|DomainParticipantFactory::set_default_participant_qos-api|
member function on the DomainParticipantFactory singleton.
Modifying the default DomainParticipantQos will not affect already existing
DomainParticipant instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_DOMAINPARTICIPANTQOS
   :end-before: //!
   :dedent: 8

|DomainParticipantFactory::set_default_participant_qos-api|
member function also accepts the value ``PARTICIPANT_QOS_DEFAULT``
as input argument.
This will reset the current default DomainParticipantQos to the default constructed value
|DomainParticipantQos::DomainParticipantQos-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DOMAINPARTICIPANTQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``PARTICIPANT_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On |DomainParticipantFactory::create_participant-api| and |DomainParticipant::set_qos-api| it refers to the
     default DomainParticipantQos as returned by |DomainParticipantFactory::get_default_participant_qos-api|.
   * On |DomainParticipantFactory::set_default_participant_qos-api| it refers to the default constructed
     |DomainParticipantQoS::DomainParticipantQoS-api|.

