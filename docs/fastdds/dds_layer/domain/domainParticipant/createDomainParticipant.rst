.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_domainParticipant_creation:

Creating a DomainParticipant
============================

Creation of a :ref:`dds_layer_domainParticipant` is done with the |DomainParticipantFactory::create_participant-api|
member function on the
:ref:`dds_layer_domainParticipantFactory` singleton, that acts as a factory for the DomainParticipant.
Then, when the lifecycle of the participant finishes, every participant must be deleted with
|DomainParticipantFactory::delete_participant-api|.
Please refer to :ref:`dds_layer_domainParticipant_deletion` for further details on the deletion of a participant.

Mandatory arguments are:

 * The |DomainId-api| that identifies the domain where the DomainParticipant will be created.

 * The :ref:`dds_layer_domainParticipantQos` describing the behavior of the DomainParticipant.
   If the provided value is :class:`PARTICIPANT_QOS_DEFAULT`, the value of the DomainParticipantQos is used.

Alternatively, instead of the two mandatory arguments above, you can use:

 * The :ref:`dds_layer_domainParticipantExtendedQos` which encompasses both the |DomainId-api| and
   the :ref:`dds_layer_domainParticipantQos`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DomainParticipant.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

.. _DDSI-RTPS V2.2: https://www.omg.org/spec/DDSI-RTPS/2.2/PDF

.. warning::
   Following the `DDSI-RTPS V2.2`_ standard (Section 9.6.1.1), the default ports are calculated depending on the
   |DomainId-api|, as it is explained in section :ref:`listening_locators_defaultPorts`.
   Thus, it is encouraged to use |DomainId-api| lower than 200
   (over |DomainId-api| 233 default port assign will fail consistently).

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

 * The |DomainId-api| that identifies the domain where the DomainParticipant will be created.
   Do not use |DomainId-api| higher than 200 (see :ref:`dds_layer_domainParticipant_creation`).


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

.. _dds_layer_domainParticipant_creation_default_profile:

Default profile DomainParticipant creation
------------------------------------------

If there is a profile already exported in the environment (please refer to :ref:`xml_profiles` for related
information), creating a DomainParticipant with the
|DomainParticipantFactory::create_participant_with_default_profile-api| member function on the
:ref:`dds_layer_domainParticipantFactory` singleton would use that settings to configure the participant.
If the profile has not been exported, the DomainParticipant will be created with the default values per
:ref:`dds_layer_domainParticipantQos`, and ``0`` as |DomainId-api|.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the DomainParticipant.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

|DomainParticipantFactory::create_participant_with_default_profile-api| will return a null pointer if there was an
error during the operation.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`xml_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_DOMAINPARTICIPANT_DEFAULT_PROFILE
   :end-before: //!
   :dedent: 8

.. _dds_layer_domainParticipant_deletion:

Deleting a DomainParticipant
----------------------------

A DomainParticipant shall be deleted with the |DomainParticipantFactory::delete_participant-api| member function on the
:ref:`dds_layer_domainParticipantFactory` singleton.

.. important::

   A DomainParticipant can only be deleted if all Entities belonging to the participant
   (Publisher, Subscriber or Topic) have already been deleted.
   Otherwise, the function will issue an error and the DomainParticipant will not be deleted.
   This can be performed either by using the |DomainParticipant::delete_contained_entities-api| member function of the
   :ref:`dds_layer_domainParticipant` or manually deleting each entity with the corresponding *delete_* method from the
   DomainParticipant e.g |DomainParticipant::delete_publisher-api|, |DomainParticipant::delete_subscriber-api|,
   |DomainParticipant::delete_topic-api|, etc.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_DOMAINPARTICIPANT
   :end-before: //!
   :dedent: 8
