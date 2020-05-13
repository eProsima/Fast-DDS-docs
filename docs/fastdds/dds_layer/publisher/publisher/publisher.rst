.. _dds_layer_publisher_publisher:

Publisher
=========

The :class:`Publisher` acts on behalf of one or several :ref:`dds_layer_publisher_dataWriter` objects
that belong to it.
When it is informed of a change to the data associated with one of its :ref:`dds_layer_publisher_dataWriter` objects,
it decides when it is appropriate to actually send the data update message.
In making this decision, it considers any extra information that goes with the data (e.g. the data timestamp),
as well as the QoS of the :class:`Publisher` and the QoS of the :ref:`dds_layer_publisher_dataWriter`.

.. _dds_layer_publisher_publisherQos:

PublisherQos
------------

:class:`PublisherQos` controls the behavior of the :ref:`dds_layer_publisher_publisher`.
Internally it contains the following :class:`QosPolicy` objects:

+--------------------------------+------------------------------------+----------+
| QosPolicy class                | Accessor                           | Mutable  |
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

The QoS value of a previously created :ref:`dds_layer_publisher_publisher` can be modified using the
:func:`set_qos` member function.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_PUBLISHERQOS
   :end-before: //!


.. _dds_layer_defaultPublisherQos:

Default PublisherQos
^^^^^^^^^^^^^^^^^^^^

The default :ref:`dds_layer_publisher_publisherQos` refers to the value returned by the
:func:`get_default_publisher_qos` member function on the :ref:`dds_layer_domainParticipant` instance.
The special value ``PUBLISHER_QOS_DEFAULT`` can be used as QoS argument on :func:`create_publisher`
or :func:`set_qos` member functions to indicate that the current default :ref:`dds_layer_publisher_publisherQos`
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

:func:`set_default_publisher_qos` member function also accepts the special value ``PUBLISHER_QOS_DEFAULT``
as input argument.
This will reset the current default :ref:`dds_layer_publisher_publisherQos` to default constructed
value :func:`PublisherQos`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CHANGE_PUBLISHERQOS_TO_DEFAULT
   :end-before: //!


.. _dds_layer_publisher_creation:

Creating a Publisher
====================

A :ref:`dds_layer_publisher_publisher` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a :ref:`dds_layer_publisher_publisher` is done with the :func:`create_publisher` member function on the
:ref:`dds_layer_domainParticipant` instance, that acts as a factory for the :ref:`dds_layer_publisher_publisher`.

Mandatory arguments are:

 * The :ref:`dds_layer_publisher_publisherQos` describing the behavior of the :ref:`dds_layer_publisher_publisher`.
   If the provided value is :class:`PUBLISHER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultPublisherQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_publisher_publisherListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_publisher_publisher`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_publisher_publisherListener`.
   By default all events are enabled.

:func:`create_publisher` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PUBLISHER
   :end-before: //!


.. _dds_layer_publisher_creation_profile:

Profile based creation of a Publisher
-------------------------------------

Instead of using a :ref:`dds_layer_publisher_publisherQos`, the name of a profile
can be used to create a :ref:`dds_layer_publisher_publisher` with the :func:`create_publisher_with_profile`
member function on the :ref:`dds_layer_domainParticipant` instance.

Mandatory arguments are:

 * A string with the name that identifies the :ref:`dds_layer_publisher_publisher`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_publisher_publisherListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_publisher_publisher`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_publisher_publisherListener`.
   By default all events are enabled.

:func:`create_publisher_with_profile` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_PUBLISHER
   :end-before: //!


.. _dds_layer_publisher_deletion:

Deleting a Publisher
--------------------

A :ref:`dds_layer_publisher_publisher` can be deleted with the :func:`delete_publisher` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_publisher_publisher` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_PUBLISHER
   :end-before: //!
