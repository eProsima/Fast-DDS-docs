.. include:: ../../../../03-exports/aliases.include
.. include:: ../../../../03-exports/aliases-api.include

.. _dds_layer_domainParticipantListener:

DomainParticipantListener
=========================

|DomainParticipantListener-api| is an abstract class defining the callbacks that will be triggered
in response to state changes on the :ref:`dds_layer_domainParticipant`.
By default, all these callbacks are empty and do nothing.
The user should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overridden will maintain their empty implementation.

DomainParticipantListener inherits from :ref:`dds_layer_topic_topicListener`,
:ref:`dds_layer_publisher_publisherListener`, and :ref:`dds_layer_subscriber_subscriberListener`.
Therefore, it has the ability to react to every kind of event that is
reported to any of its attached Entities.
Since events are always notified to the most specific Entity Listener that can handle the event,
callbacks that DomainParticipantListener inherits from other Listeners will only be called
if no other Listener was able to handle the event.

Additionally, DomainParticipantListener adds the following callbacks:

 * |DomainParticipantListener::on_participant_discovery-api|: A new DomainParticipant is discovered in the same domain,
   a previously known DomainParticipant has been removed, or some DomainParticipant
   has changed its QoS.

 * |DomainParticipantListener::on_subscriber_discovery-api|: A new :ref:`dds_layer_subscriber_subscriber` is discovered in the same domain,
   a previously known Subscriber has been removed,
   or some Subscriber has changed its QoS.

 * |DomainParticipantListener::on_publisher_discovery-api|: A new :ref:`dds_layer_publisher_publisher` is discovered in the same domain,
   a previously known Publisher has been removed,
   or some Publisher has changed its QoS.

 * |DomainParticipantListener::on_type_discovery-api|: A new data Type is discovered in the same domain.

 * |DomainParticipantListener::on_type_dependencies_reply-api|: The Type lookup client received a replay to a :func:`getTypeDependencies` request.
   This callback can be used to retrieve the new type using the :func:`getTypes` request and create a new
   dynamic type using the retrieved type object.

 * |DomainParticipantListener::on_type_information_received-api|: A new |TypeInformation-api| has been received from a newly discovered
   DomainParticipant.

 * |DomainParticipantListener::onParticipantAuthentication-api|: Informs about the result of the authentication process
   of a remote DomainParticipant (either on failure or success).

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DDS_DOMAINPARTICIPANT_LISTENER_SPECIALIZATION
   :end-before: //!


