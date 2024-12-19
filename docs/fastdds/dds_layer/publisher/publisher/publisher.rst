.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_publisher:

Publisher
=========

The |Publisher| acts on behalf of one or several |DataWriter| objects
that belong to it.
It serves as a container that allows grouping different DataWriter objects under
a common configuration given by the |PublisherQos| of the Publisher.

DataWriter objects that belong to the same Publisher do not have any other
relation among each other beyond the PublisherQos of the Publisher and act independently
otherwise.
Specifically, a Publisher can host DataWriter objects for different |Topics|
and data types.


.. _dds_layer_publisher_publisherQos:

PublisherQos
------------

|PublisherQos-api| controls the behavior of the |Publisher-api|.
Internally it contains the following |QosPolicy-api| objects:

+--------------------------------+------------------------------------+----------+
| QosPolicy class                | Accessor/Mutator                   | Mutable  |
+================================+====================================+==========+
| |PresentationQosPolicy|        | |PublisherQos::presentation-api|   | Yes      |
+--------------------------------+------------------------------------+----------+
| |PartitionQosPolicy|           | |PublisherQos::partition-api|      | Yes      |
+--------------------------------+------------------------------------+----------+
| |GroupDataQosPolicy|           | |PublisherQos::group_data-api|     | Yes      |
+--------------------------------+------------------------------------+----------+
| |EntityFactoryQosPolicy|       | |PublisherQos::entity_factory-api| | Yes      |
+--------------------------------+------------------------------------+----------+

Refer to the detailed description of each |QosPolicy-api| class for more information about their usage and
default values.

The QoS value of a previously created Publisher can be modified using the
|Publisher::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_PUBLISHERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultPublisherQos:

Default PublisherQos
^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_publisher_publisherQos` refers to the value returned by the
|DomainParticipant::get_default_publisher_qos-api| member function on the DomainParticipant instance.
The special value :code:`PUBLISHER_QOS_DEFAULT` can be used as QoS argument on
|DomainParticipant::create_publisher-api| or |Publisher::set_qos-api| member functions to indicate that the current
default PublisherQos should be used.

When the system starts, the default PublisherQos is equivalent to the default constructed
value |PublisherQos::PublisherQos-api|.
The default PublisherQos can be modified at any time using the
|DomainParticipant::set_default_publisher_qos-api| member function on the :ref:`dds_layer_domainParticipant` instance.
Modifying the default PublisherQos will not affect already existing
:ref:`dds_layer_publisher_publisher` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_PUBLISHERQOS
   :end-before: //!
   :dedent: 8

|DomainParticipant::set_default_publisher_qos-api| member function also accepts the special value
:code:`PUBLISHER_QOS_DEFAULT` as input argument.
This will reset the current default PublisherQos to default constructed
value |PublisherQos::PublisherQos-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_PUBLISHERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value :code:`PUBLISHER_QOS_DEFAULT` has different meaning depending on where it is used:

   * On |DomainParticipant::create_publisher-api| and |Publisher::set_qos-api| it refers to the default
     :ref:`dds_layer_publisher_publisherQos`.
     as returned by |DomainParticipant::get_default_publisher_qos-api|.
   * On |DomainParticipant::set_default_publisher_qos-api| it refers to the default constructed
     |PublisherQos::PublisherQos-api|.

