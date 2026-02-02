.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_publisher_creation:

Creating a Publisher
====================

A :ref:`dds_layer_publisher_publisher` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a Publisher is done with the |DomainParticipant::create_publisher-api| member function on the
DomainParticipant instance, that acts as a factory for the Publisher.

Mandatory arguments are:

* The :ref:`dds_layer_publisher_publisherQos` describing the behavior of the Publisher.
  If the provided value is :code:`PUBLISHER_QOS_DEFAULT`,
  the value of the :ref:`dds_layer_defaultPublisherQos` is used.

Optional arguments are:

* A Listener derived from :ref:`dds_layer_publisher_publisherListener`, implementing the callbacks
  that will be triggered in response to events and state changes on the Publisher.
  By default empty callbacks are used.

* A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
  PublisherListener.
  By default all events are enabled.

|DomainParticipant::create_publisher-api| will return a null pointer if there was an error during the operation, e.g.
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
can be used to create a Publisher with the |DomainParticipant::create_publisher_with_profile-api|
member function on the DomainParticipant instance.

Mandatory arguments are:

* A string with the name that identifies the Publisher.

Optional arguments are:

* A Listener derived from :ref:`dds_layer_publisher_publisherListener`, implementing the callbacks
  that will be triggered in response to events and state changes on the Publisher.
  By default empty callbacks are used.

* A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
  PublisherListener.
  By default all events are enabled.

|DomainParticipant::create_publisher_with_profile-api| will return a null pointer if there was an error during the
operation, e.g. if the provided QoS is not compatible or is not supported.
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

A Publisher can be deleted with the |DomainParticipant::delete_publisher-api| member function on the
DomainParticipant instance where the Publisher was created.

.. note::

    A Publisher can only be deleted if all Entities belonging to the Publisher
    (DataWriters) have already been deleted.
    Otherwise, the function will issue an error and the Publisher will not be deleted.
    This can be performed by using the |Publisher::delete_contained_entities-api| member function of the
    :ref:`dds_layer_publisher_publisher`.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //DDS_DELETE_PUBLISHER
    :end-before: //!
    :dedent: 8
