.. _dds_layer_subscriber_subscriber:

Subscriber
==========

The :class:`Subscriber` acts on behalf of one or several :ref:`dds_layer_subscriber_dataReader` objects
that belong to it.
It serves as a container that allows grouping different :ref:`dds_layer_subscriber_dataReader` objects under
a common configuration given by the :class:`SubscriberQos` of the :class:`Subscriber`.

:ref:`dds_layer_subscriber_dataReader` objects that belong to the same :class:`Subscriber` do not have any other
relation among each other beyond the :class:`SubscriberQos` of the :class:`Subscriber` and act independently
otherwise.
Specifically, a :class:`Subscriber` can host :ref:`dds_layer_subscriber_dataReader` objects for different topics
and data types.


.. _dds_layer_subscriber_subscriberQos:

SubscriberQos
-------------

:class:`SubscriberQos` controls the behavior of the :ref:`dds_layer_subscriber_subscriber`.
Internally it contains the following :class:`QosPolicy` objects:

+--------------------------------+------------------------------------+----------+
| QosPolicy class                | Accessor/Mutator                   | Mutable  |
+================================+====================================+==========+
| PresentationQosPolicy          | :func:`presentation`               | Yes      |
+--------------------------------+------------------------------------+----------+
| PartitionQosPolicy             | :func:`partition`                  | Yes      |
+--------------------------------+------------------------------------+----------+
| GroupDataQosPolicy             | :func:`group_data`                 | Yes      |
+--------------------------------+------------------------------------+----------+
| EntityFactoryQosPolicy         | :func:`entity_factory`             | Yes      |
+--------------------------------+------------------------------------+----------+

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
:func:`get_default_subscriber_qos` member function on the :ref:`dds_layer_domainParticipant` instance.
The special value ``SUBSCRIBER_QOS_DEFAULT`` can be used as QoS argument on :func:`create_subscriber`
or :func:`set_qos` member functions to indicate that the current default :ref:`dds_layer_subscriber_subscriberQos`
should be used.

When the system starts, the default :ref:`dds_layer_subscriber_subscriberQos` is equivalent to the default constructed
value :func:`SubscriberQos`.
The default :ref:`dds_layer_subscriber_subscriberQos` can be modified at any time using the
:func:`set_default_subscriber_qos` member function on the :ref:`dds_layer_domainParticipant` instance.
Modifying the default :ref:`dds_layer_subscriber_subscriberQos` will not affect already existing
:ref:`dds_layer_subscriber_subscriber` instances.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_DEFAULT_SUBSCRIBERQOS
   :end-before: //!
   :dedent: 8

:func:`set_default_subscriber_qos` member function also accepts the special value ``SUBSCRIBER_QOS_DEFAULT``
as input argument.
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

