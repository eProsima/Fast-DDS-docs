.. _dds_layer_domainParticipantListener:

DomainParticipantListener
=========================

:class:`DomainParticipantListener` is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_domainParticipant`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

:class:`DomainParticipantListener` inherits from :ref:`dds_layer_topic_topicListener`,
:ref:`dds_layer_publisher_publisherListener`, and :ref:`dds_layer_subscriber_subscriberListener`.
Therefore, it has the ability to react to every kind of event that is
reported to any of its attached Entities.
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that DomainParticipantListener inherits from other Listeners will only be called
if no other Listener was able to handle the event.

Additionally, :class:`DomainParticipantListener` adds the following callbacks:

 * **on_participant_discovery**: A new :ref:`dds_layer_domainParticipant` is discovered in the same domain,
   a previously known :ref:`dds_layer_domainParticipant` has been removed, or some :ref:`dds_layer_domainParticipant`
   has changed its QoS.

 * **on_subscriber_discovery**: A new :ref:`dds_layer_subscriber_subscriber` is discovered in the same domain,
   a previously known :ref:`dds_layer_subscriber_subscriber` has been removed,
   or some :ref:`dds_layer_subscriber_subscriber` has changed its QoS.

 * **on_publisher_discovery**: A new :ref:`dds_layer_publisher_publisher` is discovered in the same domain,
   a previously known :ref:`dds_layer_publisher_publisher` has been removed,
   or some :ref:`dds_layer_publisher_publisher` has changed its QoS.

 * **on_type_discovery**: A new data Type is discovered in the same domain.

 * **on_type_dependencies_reply**: The Type lookup client received a replay to a :func:`getTypeDependencies` request.
   This callback can be used to retrieve the new type using the :func:`getTypes` request and create a new
   dynamic type using the retrieved type object.

 * **on_type_information_received**: A new :class:`TypeInformation` has been received from a newly discovered
   :ref:`dds_layer_domainParticipant`.

 * **onParticipantAuthentication**: Informs about the result of the authentication process
   of a remote :ref:`dds_layer_domainParticipant` (either on failure or success).

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DOMAINPARTICIPANT_LISTENER_SPECIALIZATION
   :end-before: //!


