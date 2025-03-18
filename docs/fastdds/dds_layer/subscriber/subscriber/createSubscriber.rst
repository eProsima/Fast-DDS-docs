.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_subscriber_creation:

Creating a Subscriber
=====================

A :ref:`dds_layer_subscriber_subscriber` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a Subscriber is done with the |DomainParticipant::create_subscriber-api| member function on the
DomainParticipant instance, that acts as a factory for the Subscriber.

Mandatory arguments are:

* The :ref:`dds_layer_subscriber_subscriberQos` describing the behavior of the Subscriber.
  If the provided value is :code:`SUBSCRIBER_QOS_DEFAULT`,
  the value of the :ref:`dds_layer_defaultSubscriberQos` is used.

Optional arguments are:

* A Listener derived from :ref:`dds_layer_subscriber_subscriberListener`, implementing the callbacks
  that will be triggered in response to events and state changes on the Subscriber.
  By default empty callbacks are used.

* A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
  SubscriberListener.
  By default all events are enabled.

|DomainParticipant::create_subscriber-api| will return a null pointer if there was an error during the operation, e.g.
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

Instead of using a SubscriberQos, the name of a profile
can be used to create a Subscriber with the |DomainParticipant::create_subscriber_with_profile-api|
member function on the DomainParticipant instance.

Mandatory arguments are:

* A string with the name that identifies the Subscriber.

Optional arguments are:

* A Listener derived from SubscriberListener, implementing the callbacks
  that will be triggered in response to events and state changes on the Subscriber.
  By default empty callbacks are used.

* A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
  SubscriberListener.
  By default all events are enabled.

|DomainParticipant::create_subscriber_with_profile-api| will return a null pointer if there was an error during
the operation, e.g. if the provided QoS is not compatible or is not supported.
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

A Subscriber can be deleted with the |DomainParticipant::delete_subscriber-api| member function on the
DomainParticipant instance where the Subscriber was created.

.. note::

    A Subscriber can only be deleted if all Entities belonging to the Subscriber
    (DataReaders) have already been deleted.
    Otherwise, the function will issue an error and the Subscriber will not be deleted.
    This can be performed by using the |Subscriber::delete_contained_entities-api| member function of the
    :ref:`dds_layer_subscriber_subscriber`.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_DELETE_SUBSCRIBER
    :end-before: //!
    :dedent: 8
