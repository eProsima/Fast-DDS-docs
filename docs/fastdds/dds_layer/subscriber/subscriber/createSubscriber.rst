.. _dds_layer_subscriber_creation:

Creating a Subscriber
=====================

A :ref:`dds_layer_subscriber_subscriber` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a :ref:`dds_layer_subscriber_subscriber` is done with the :func:`create_subscriber` member function on the
:ref:`dds_layer_domainParticipant` instance, that acts as a factory for the :ref:`dds_layer_subscriber_subscriber`.

Mandatory arguments are:

 * The :ref:`dds_layer_subscriber_subscriberQos` describing the behavior of the :ref:`dds_layer_subscriber_subscriber`.
   If the provided value is :class:`SUBSCRIBER_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultSubscriberQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_subscriber_subscriberListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_subscriber_subscriber`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_subscriber_subscriberListener`.
   By default all events are enabled.

:func:`create_subscriber` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_SUBSCRIBER
   :end-before: //!
   :dedent: 8


.. _dds_layer_subscriber_creation_profile:

Profile based creation of a Subscriber
--------------------------------------

Instead of using a :ref:`dds_layer_subscriber_subscriberQos`, the name of a profile
can be used to create a :ref:`dds_layer_subscriber_subscriber` with the :func:`create_subscriber_with_profile`
member function on the :ref:`dds_layer_domainParticipant` instance.

Mandatory arguments are:

 * A string with the name that identifies the :ref:`dds_layer_subscriber_subscriber`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_subscriber_subscriberListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_subscriber_subscriber`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_subscriber_subscriberListener`.
   By default all events are enabled.

:func:`create_subscriber_with_profile` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_SUBSCRIBER
   :end-before: //!
   :dedent: 8


.. _dds_layer_subscriber_deletion:

Deleting a Subscriber
---------------------

A :ref:`dds_layer_subscriber_subscriber` can be deleted with the :func:`delete_subscriber` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_subscriber_subscriber` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_SUBSCRIBER
   :end-before: //!
   :dedent: 8

