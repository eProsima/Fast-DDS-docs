.. include:: ../../exports/aliases.include
.. include:: ../../../api_reference/dds_pim/publisher/exports/aliases.include
.. include:: ../../../api_reference/dds_pim/domain/exports/aliases.include

.. _dds_layer_publisher_publisher:

Publisher
=========

The **Publisher** acts on behalf of one or several **DataWriter** objects
that belong to it.
It serves as a container that allows grouping different **DataWriter** objects under
a common configuration given by the **PublisherQos** of the **Publisher**.

**DataWriter** objects that belong to the same **Publisher** do not have any other
relation among each other beyond the **PublisherQos** of the **Publisher** and act independently
otherwise.
Specifically, a **Publisher** can host **DataWriter** objects for different **Topics**
and data types.


.. _dds_layer_publisher_publisherQos:

PublisherQos
------------

**PublisherQos** controls the behavior of the **Publisher**.
Internally it contains the following :class:`QosPolicy` objects:

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

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

The QoS value of a previously created **Publisher** can be modified using the
|Publisher::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_PUBLISHERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultPublisherQos:

Default PublisherQos
^^^^^^^^^^^^^^^^^^^^

The default **PublisherQos** refers to the value returned by the
|DomainParticipant::get_default_publisher_qos-api| member function on the DomainParticipant instance.
The special value ``PUBLISHER_QOS_DEFAULT`` can be used as QoS argument on |DomainParticipant::create_publisher-api|
or |Publisher::set_qos-api| member functions to indicate that the current default :ref:`dds_layer_publisher_publisherQos`
should be used.

When the system starts, the default :ref:`dds_layer_publisher_publisherQos` is equivalent to the default constructed
value :func:`PublisherQos`.
The default :ref:`dds_layer_publisher_publisherQos` can be modified at any time using the
:func:`set_default_publisher_qos` member function on the :ref:`dds_layer_domainParticipant` instance.
Modifying the default :ref:`dds_layer_publisher_publisherQos` will not affect already existing
:ref:`dds_layer_publisher_publisher` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_PUBLISHERQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_publisher_qos` member function also accepts the special value ``PUBLISHER_QOS_DEFAULT``
as input argument.
This will reset the current default :ref:`dds_layer_publisher_publisherQos` to default constructed
value :func:`PublisherQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_PUBLISHERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``PUBLISHER_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On :func:`create_publisher` and :func:`set_qos` it refers to the default :ref:`dds_layer_publisher_publisherQos`
     as returned by :func:`get_default_publisher_qos`.
   * On :func:`set_default_publisher_qos` it refers to the default constructed :func:`PublisherQos`.

