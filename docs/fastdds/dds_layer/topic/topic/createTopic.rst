.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_topic_creation:

Creating a Topic
================

A :ref:`dds_layer_topic_topic` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a Topic is done with the |DomainParticipant::create_topic-api| member function on the
|DomainParticipant-api| instance, that acts as a factory for the |Topic-api|.

Mandatory arguments are:

 * A string with the name that identifies the Topic.

 * The name of the registered :ref:`data type<dds_layer_definition_data_types>` that will be transmitted.

 * The :ref:`dds_layer_topic_topicQos` describing the behavior of the Topic.
   If the provided value is :class:`TOPIC_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultTopicQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_topic_topicListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the Topic.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   TopicListener.
   By default all events are enabled.

|DomainParticipant::create_topic-api| will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_TOPIC
   :end-before: //!
   :dedent: 8

.. _dds_layer_topic_creation_profile:

Profile based creation of a Topic
---------------------------------

Instead of using a TopicQos, the name of a profile
can be used to create a Topic with the |DomainParticipant::create_topic_with_profile-api|
member function on the DomainParticipant instance.

Mandatory arguments are:

 * A string with the name that identifies the Topic.

 * The name of the registered :ref:`data type<dds_layer_definition_data_types>` that will be transmitted.

 * The name of the profile to be applied to the Topic.

Optional arguments are:

 * A Listener derived from TopicListener, implementing the callbacks
   that will be triggered in response to events and state changes on the Topic.
   By default empty callbacks are used.

 * A |StatusMask-api| that activates or deactivates triggering of individual callbacks on the
   TopicListener.
   By default all events are enabled.

:func:`create_topic_with_profile` will return a null pointer if there was an error during the operation, e.g.
if the provided QoS is not compatible or is not supported.
It is advisable to check that the returned value is a valid pointer.

.. note::

   XML profiles must have been loaded previously. See :ref:`dds_layer_domainParticipantFactory_load_profiles`.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_CREATE_PROFILE_TOPIC
   :end-before: //!
   :dedent: 8


.. _dds_layer_topic_deletion:

Deleting a Topic
----------------

A Topic can be deleted with the |DomainParticipant::delete_topic-api| member function on the
DomainParticipant instance where the Topic was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_TOPIC
   :end-before: //!
   :dedent: 8

