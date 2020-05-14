.. _dds_layer_domainParticipant_creation:

Creating a DomainParticipant
============================

Creation of a :ref:`dds_layer_domainParticipant` is done with the :func:`create_participant()` member function on the
:ref:`dds_layer_domainParticipantFactory` singleton, that acts as a factory for the :ref:`dds_layer_domainParticipant`.

Mandatory arguments are:

 * The domainId that identifies the domain where the :ref:`dds_layer_domainParticipant` will be created.

 * The :ref:`dds_layer_domainParticipantQos` describing the behavior of the :ref:`dds_layer_domainParticipant`.
   If the provided value is :class:`TOPIC_QOS_DEFAULT`, the value of the :ref:`dds_layer_domainParticipantQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_domainParticipant`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

:func:`create_participant()` will return a null pointer if there was an error during the operation, e.g.
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

Instead of using a :ref:`dds_layer_domainParticipantQos`, the name of a profile
can be used to create a :ref:`dds_layer_domainParticipant` with the :func:`create_participant_with_profile()`
member function on the :ref:`dds_layer_domainParticipantFactory` singleton.

Mandatory arguments are:

 * The domainId that identifies the domain where the :ref:`dds_layer_domainParticipant` will be created.

 * The name of the profile to be applied to the :ref:`dds_layer_domainParticipant`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_domainParticipantListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_domainParticipant`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_domainParticipantListener`.
   By default all events are enabled.

:func:`create_participant_with_profile()` will return a null pointer if there was an error during the operation, e.g
if the provided QoS is not compatible or is not supported.
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

A :ref:`dds_layer_domainParticipant` can be deleted with the :func:`delete_participant()` member function on the
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

