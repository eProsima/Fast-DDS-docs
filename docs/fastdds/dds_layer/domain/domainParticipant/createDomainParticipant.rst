.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_domainParticipant_creation:

Creating a DomainParticipant
============================

Creation of a :ref:`dds_layer_domainParticipant` is done with the |DomainParticipantFactory::create_participant-api|
member function on the
:ref:`dds_layer_domainParticipantFactory` singleton, that acts as a factory for the DomainParticipant.

Mandatory arguments are:

 * The domainId that identifies the domain where the DomainParticipant will be created.

 * The :ref:`dds_layer_domainParticipantQos` describing the behavior of the DomainParticipant.
   If the provided value is :class:`TOPIC_QOS_DEFAULT`, the value of the DomainParticipantQos is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DomainParticipant.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

|DomainParticipantFactory::create_participant-api|
will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DOMAINPARTICIPANT
   :end-before: //!
   :dedent: 8


.. _dds_layer_domainParticipant_creation_profile:

Profile based creation of a DomainParticipant
---------------------------------------------

Instead of using a DomainParticipantQos, the name of a profile
can be used to create a DomainParticipant with the |DomainParticipantFactory::create_participant_with_profile-api|
member function on the :ref:`dds_layer_domainParticipantFactory` singleton.

Mandatory arguments are:

 * The domainId that identifies the domain where the DomainParticipant will be created.

 * The name of the profile to be applied to the DomainParticipant.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DomainParticipant.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

|DomainParticipantFactory::create_participant_with_profile-api| will return a null pointer if there was an error during
the operation, e.g if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_DOMAINPARTICIPANT
   :end-before: //!
   :dedent: 8

.. _dds_layer_domainParticipant_deletion:

Deleting a DomainParticipant
----------------------------

A DomainParticipant can be deleted with the |DomainParticipantFactory::delete_participant-api| member function on the
:ref:`dds_layer_domainParticipantFactory` singleton.

.. note::

   A DomainParticipant can only be deleted if all domain Entities belonging to the participant
   (Publisher, Subscriber or Topic) have already been deleted.
   Otherwise, the function will issue an error and the DomainParticipant will not be deleted.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DOMAINPARTICIPANT
   :end-before: //!
   :dedent: 8

