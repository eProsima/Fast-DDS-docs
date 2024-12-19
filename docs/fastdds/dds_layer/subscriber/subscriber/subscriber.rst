.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_subscriber:

Subscriber
==========

The |Subscriber-api| acts on behalf of one or several :ref:`dds_layer_subscriber_dataReader` objects
that belong to it.
It serves as a container that allows grouping different DataReader objects under
a common configuration given by the :ref:`dds_layer_subscriber_subscriberQos` of the Subscriber.

DataReader objects that belong to the same Subscriber
do not have any other relation among each other beyond the |SubscriberQos-api| of the Subscriber
and act independently otherwise.
Specifically, a Subscriber can host DataReader objects
for different topics and data types.


.. _dds_layer_subscriber_subscriberQos:

SubscriberQos
-------------

|SubscriberQos-api| controls the behavior of the :ref:`dds_layer_subscriber_subscriber`.
Internally it contains the following |QosPolicy-api| objects:

+----------------------------------------+------------------------------------------------------------------+----------+
| QosPolicy class                        | Accessor/Mutator                                                 | Mutable  |
+========================================+==================================================================+==========+
| |presentationqospolicy|                | |SubscriberQos::presentation-api|                                | Yes      |
+----------------------------------------+------------------------------------------------------------------+----------+
| |partitionqospolicy|                   | |SubscriberQos::partition-api|                                   | Yes      |
+----------------------------------------+------------------------------------------------------------------+----------+
| |groupdataqospolicy|                   | |SubscriberQos::group_data-api|                                  | Yes      |
+----------------------------------------+------------------------------------------------------------------+----------+
| |entityfactoryqospolicy|               | |SubscriberQos::entity_factory-api|                              | Yes      |
+----------------------------------------+------------------------------------------------------------------+----------+

Refer to the detailed description of each |QosPolicy-api| class for more information about their usage and
default values.

The QoS value of a previously created Subscriber can be modified using the
|Subscriber::set_qos-api| member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_SUBSCRIBERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultSubscriberQos:

Default SubscriberQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_subscriber_subscriberQos` refers to the value returned by the
|DomainParticipant::get_default_subscriber_qos-api| member function
on the :ref:`dds_layer_domainParticipant` instance.
The special value :code:`SUBSCRIBER_QOS_DEFAULT` can be used as QoS argument on
|DomainParticipant::create_subscriber-api| or |Subscriber::set_qos-api|
member functions to indicate that the current default SubscriberQos
should be used.

When the system starts, the default SubscriberQos is equivalent to the default constructed
value |SubscriberQos::SubscriberQos-api|.
The default SubscriberQos can be modified at any time using the
|DomainParticipant::set_default_subscriber_qos-api| member function on the
DomainParticipant instance.
Modifying the default SubscriberQos will not affect already existing
Subscriber instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_SUBSCRIBERQOS
   :end-before: //!
   :dedent: 8

|DomainParticipant::set_default_subscriber_qos-api| member function also accepts
the special value :code:`SUBSCRIBER_QOS_DEFAULT` as input argument.
This will reset the current default SubscriberQos to default constructed
value |SubscriberQos::SubscriberQos-api|.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_SUBSCRIBERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value :code:`SUBSCRIBER_QOS_DEFAULT` has different meaning depending on where it is used:

   * On |DomainParticipant::create_subscriber-api| and |Subscriber::set_qos-api| it refers to the default
     SubscriberQos as returned by |DomainParticipant::get_default_subscriber_qos-api|.
   * On |DomainParticipant::set_default_subscriber_qos-api| it refers to the default constructed
     |SubscriberQos::SubscriberQos-api|.

