.. include:: ../../exports/aliases.include
.. include:: ../../../api_reference/dds_pim/subscriber/exports/aliases.include

.. _dds_layer_subscriber_subscriber:

Subscriber
==========

The :ref:`api_pim_subscriber_class` acts on behalf of one or several :ref:`dds_layer_subscriber_dataReader` objects
that belong to it.
It serves as a container that allows grouping different :ref:`dds_layer_subscriber_dataReader` objects under
a common configuration given by the :ref:`dds_layer_subscriber_subscriberQos` of the :ref:`api_pim_subscriber_class`.

:ref:`dds_layer_subscriber_dataReader` objects that belong to the same :ref:`api_pim_subscriber_class`
do not have any other relation among each other beyond the :class:`SubscriberQos` of the :class:`Subscriber`
and act independently otherwise.
Specifically, a :ref:`api_pim_subscriber_class` can host :ref:`dds_layer_subscriber_dataReader` objects
for different topics and data types.


.. _dds_layer_subscriber_subscriberQos:

SubscriberQos
-------------

:ref:`api_pim_subscriberqos` controls the behavior of the :ref:`dds_layer_subscriber_subscriber`.
Internally it contains the following :class:`QosPolicy` objects:

+----------------------------------------+-------------------------------------+----------+
| QosPolicy class                        | Accessor/Mutator                    | Mutable  |
+========================================+=====================================+==========+
| |presentationqospolicy|                | |SubscriberQos::presentation-api|   | Yes      |
+----------------------------------------+-------------------------------------+----------+
| |partitionqospolicy|                   | |SubscriberQos::partition-api|      | Yes      |
+----------------------------------------+-------------------------------------+----------+
| |groupdataqospolicy|                   | |SubscriberQos::group_data-api|     | Yes      |
+----------------------------------------+-------------------------------------+----------+
| |entityfactoryqospolicy|               | |SubscriberQos::entity_factory-api| | Yes      |
+----------------------------------------+-------------------------------------+----------+

Refer to the detailed description of each :class:`QosPolicy` class for more information about their usage and
default values.

The QoS value of a previously created :ref:`dds_layer_subscriber_subscriber` can be modified using the
:func:`set_qos` member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_SUBSCRIBERQOS
   :end-before: //!
   :dedent: 8


.. _dds_layer_defaultSubscriberQos:

Default SubscriberQos
^^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_subscriber_subscriberQos` refers to the value returned by the
:func:`get_default_subscriber_qos` member function
on the :ref:`dds_layer_domainParticipant` instance.
The special value ``SUBSCRIBER_QOS_DEFAULT`` can be used as QoS argument on
:func:`create_subscriber` or :func:`set_qos`
member functions to indicate that the current default :ref:`dds_layer_subscriber_subscriberQos`
should be used.

When the system starts, the default :ref:`dds_layer_subscriber_subscriberQos` is equivalent to the default constructed
value :func:`SubscriberQos`.
The default :ref:`dds_layer_subscriber_subscriberQos` can be modified at any time using the
:func:`set_default_subscriber_qos` member function on the
:ref:`dds_layer_domainParticipant` instance.
Modifying the default :ref:`dds_layer_subscriber_subscriberQos` will not affect already existing
:ref:`dds_layer_subscriber_subscriber` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_SUBSCRIBERQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_subscriber_qos` member function also accepts
the special value ``SUBSCRIBER_QOS_DEFAULT`` as input argument.
This will reset the current default :ref:`dds_layer_subscriber_subscriberQos` to default constructed
value :func:`SubscriberQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_SUBSCRIBERQOS_TO_DEFAULT
   :end-before: //!
   :dedent: 8

.. note::
   The value ``SUBSCRIBER_QOS_DEFAULT`` has different meaning depending on where it is used:

   * On :func:`create_subscriber` and :func:`set_qos` it refers to the default
     :ref:`dds_layer_subscriber_subscriberQos` as returned by :func:`get_default_subscriber_qos`.
   * On :func:`set_default_subscriber_qos` it refers to the default constructed :func:`SubscriberQos`.

