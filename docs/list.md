Objects and Data Structures
=================


In order to make the most of *eProsima Fast RTPS* it is important to have a grasp of the objects and data structures that conform the library. *eProsima Fast RTPS* objects are classified by modules, which are briefly listed and descripted in this section. For full coverage take a look at the API Reference document that comes with the distribution.

## Publisher Subscriber Module

This module composes the Publisher-Subscriber abstraction we saw in the Library Overview. The concepts here are higher level than the RTPS standard. 

* Domain: Used  to create, manage and destroy high-level Participants.
* Participant: Contains Publishers and Subscribers, and manages their configuration.
	* ParticipantAttributes: Configuration parameters used in the creation of a Participant.
	* ParticipantListener: Allows you to implement callbacks within scope of the Participant.
* Publisher: Sends (publishes) data in the form of topic changes.
	* PublisherAttributes: Configuration parameters for the construction of a Publisher.
	* PublisherListener: Allows you to implement callbacks within scope of the Publisher.
* Subscriber: Receives data for the topics it subscribes to.
	* SubscriberAttributes: Configuration parameters for the construction of a Subscriber.
	* SubscriberListener: Allows you to implement callbacks within scope of the Subscriber.

## RTPS Module

This module directly maps to the ideas defined in the RTPS standard, and allows you to interact with RTPS entities directly. It consists of a few sub-modules:

### RTPS Common

* CacheChange_t: Represents a change to a topic, to be stored in a history cache.
* Data: Payload associated to a cache change. May be empty depending on the message and change type.
* Message: Defines the organization of a RTPS Message.
* Header: Standard header that identifies a message as belonging to the RTPS protocol, and includes the vendor id.
* Sub-Message Header: Identifier for an RTPS sub-message. An RTPS Message can be composed of several sub-messages.
* MessageReceiver: Deserializes and processes received RTPS messages.
* RTPSMessageCreator: Composes RTPS messages.

### RTPS Domain

* RTPSDomain: Use it to create, manage and destroy low-level RTPSParticipants.
* RTPSParticipant: Contains RTPS Writers and Readers, and manages their configuration.
	* RTPSParticipantAttributes: Configuration parameters used in the creation of an RTPS Participant.
	* PDPSimple: Allows the participant to become aware of the other participants within the Network, through the Participant Discovery Protocol.
	* EDPSimple: Allows the Participant to become aware of the endpoints (RTPS Writers and Readers) present in the other Participants within the network, through the Endpoint Discovery Protocol.
	* EDPStatic: Reads information about remote endpoints from a user file.
	* TimedEvent:  Base class for periodic or timed events.

### RTPS Reader

* RTPSReader: Base class for the reader endpoint. 
	* ReaderAttributes: Configuration parameters used in the creation of an RTPS Reader.
	* ReaderHistory: History data structure. Stores recent topic changes.
	* ReaderListener: Use it to define callbacks in scope of the Reader.

### RTPS Writer

* RTPSWriter: Base class for the writer endpoint.
	* WriterAttributes: Configuration parameters used in the creation of an RTPS Writer.
	* WriterHistory: History data structure. Stores outgoing topic changes and schedules them to be sent.

