.. _dds_layer_topic_creation:

Creating a Topic
================

A :ref:`dds_layer_topic_topic` always belongs to a :ref:`dds_layer_domainParticipant`.
Creation of a :ref:`dds_layer_topic_topic` is done with the :func:`create_topic` member function on the
:ref:`dds_layer_domainParticipant` instance, that acts as a factory for the :class:`Topic`.

Mandatory arguments are:

 * A string with the name that identifies the :ref:`dds_layer_topic_topic`.

 * The name of the registered :ref:`data type<dds_layer_definition_data_types>` that will be transmitted.

 * The :ref:`dds_layer_topic_topicQos` describing the behavior of the :ref:`dds_layer_topic_topic`.
   If the provided value is :class:`TOPIC_QOS_DEFAULT`,
   the value of the :ref:`dds_layer_defaultTopicQos` is used.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_topic_topicListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_topic_topic`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_topic_topicListener`.
   By default all events are enabled.

:func:`create_topic` will return a null pointer if there was an error during the operation, e.g.
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

Instead of using a :ref:`dds_layer_topic_topicQos`, the name of a profile
can be used to create a :ref:`dds_layer_topic_topic` with the :func:`create_topic_with_profile()`
member function on the :ref:`dds_layer_domainParticipant` instance.

Mandatory arguments are:

 * A string with the name that identifies the :ref:`dds_layer_topic_topic`.

 * The name of the registered :ref:`data type<dds_layer_definition_data_types>` that will be transmitted.

 * The name of the profile to be applied to the :ref:`dds_layer_topic_topic`.

Optional arguments are:

 * A Listener derived from :ref:`dds_layer_topic_topicListener`, implementing the callbacks
   that will be triggered in response to events and state changes on the :ref:`dds_layer_topic_topic`.
   By default empty callbacks are used.

 * A :class:`StatusMask` that activates or deactivates triggering of individual callbacks on the
   :ref:`dds_layer_topic_topicListener`.
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

A :ref:`dds_layer_topic_topic` can be deleted with the :func:`delete_topic` member function on the
:ref:`dds_layer_domainParticipant` instance where the :ref:`dds_layer_topic_topic` was created.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DELETE_TOPIC
   :end-before: //!
   :dedent: 8

