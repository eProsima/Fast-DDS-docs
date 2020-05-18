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
   :dedent: 8


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
   :dedent: 8


.. _dds_layer_publisher_deletion:

Deleting a Publisher
--------------------

A :ref:`dds_layer_publisher_publisher` can be deleted with the :func:`delete_publisher` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_publisher_publisher` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_PUBLISHER
   :end-before: //!
   :dedent: 8

