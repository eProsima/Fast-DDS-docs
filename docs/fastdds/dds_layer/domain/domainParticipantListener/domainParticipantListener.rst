.. _dds_layer_domainParticiantListener:

DomainParticipantListener
=========================

DomainParticipantListener is an abstract class defining the callbacks that will be triggered
in response to state changes on the DomainParticipant.
By default, all these callbacks are empty and do nothing.
The client should implement a specialization of this class overriding the callbacks
that are needed on the application.
Callbacks that are not overriden will maintain their empty implementation.

DomainParticipantListener inherits from :class:`TopicListener`, :class:`PublisherListener`, and
:class:`SubscriberListener`.
Therefore, it has the ability to react to every kind of event that is
reported to the client application.

Additionally, DomainParticipantListener add the following callbacks:

 * **on_participant_discovery**: A new DomainParticipant is discovered in the same domain, a previously known
   DomainParticipant has been removed, or some DomainParticipant has changed its QoS.

 * **on_subscriber_discovery**: A new Subscriber is discovered in the same domain, a previously known
   Subscriber has been removed, or some Subscriber has changed its QoS.

 * **on_publisher_discovery**: A new Publisher is discovered in the same domain, a previously known
   Publisher has been removed, or some Publisher has changed its QoS.

 * **on_type_discovery**: A new data Type is discovered in the same domain.

 * **on_type_dependencies_reply**: The typelookup client has received a replay to a :fun:`getTypeDependencies` request.
   This callback can be used to retrieve the new type using the :fun:`getTypes` request and create a new
   :class:`DynamicType` using the retrieved :class:`TypeObject`.

 * **on_type_information_received**: A new :class:`TypeInformation` has been received from a newly discovered
   DomainParticipant
   
 * **onParticipantAuthentication**: Informs about the result of the authentication process
   of a remote DomainParticipant (either on failure or success).


