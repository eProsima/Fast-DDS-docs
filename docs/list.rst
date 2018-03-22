Objects and Data Structures
===========================


In order to make the most of *eProsima Fast RTPS* it is important to have a grasp of the objects and data structures that conform the library. *eProsima Fast RTPS* objects are classified by modules, which are briefly listed and described in this section. For full coverage take a look at the API Reference document that comes with the distribution.

Publisher Subscriber Module
---------------------------

This module composes the Publisher-Subscriber abstraction we saw in the Library Overview. The concepts here are higher level than the RTPS standard. 

* :class:`Domain` Used to create, manage and destroy high-level Participants.
* :class:`Participant` Contains Publishers and Subscribers, and manages their configuration.

    * :class:`ParticipantAttributes` Configuration parameters used in the creation of a Participant.
    * :class:`ParticipantListener` Allows you to implement callbacks within scope of the Participant.

* :class:`Publisher` Sends (publishes) data in the form of topic changes.

    * :class:`PublisherAttributes` Configuration parameters for the construction of a Publisher.
    * :class:`PublisherListener` Allows you to implement callbacks within scope of the Publisher.

* :class:`Subscriber` Receives data for the topics it subscribes to.

    * :class:`SubscriberAttributes` Configuration parameters for the construction of a Subscriber.
    * :class:`SubscriberListener` Allows you to implement callbacks within scope of the Subscriber.

RTPS Module
-----------

This module directly maps to the ideas defined in the RTPS standard, and allows you to interact with RTPS entities directly. It consists of a few sub-modules:

RTPS Common
^^^^^^^^^^^

* :class:`CacheChange_t` Represents a change to a topic, to be stored in a history cache.
* :class:`Data` Payload associated to a cache change. May be empty depending on the message and change type.
* :class:`Message` Defines the organization of a RTPS Message.
* :class:`Header` Standard header that identifies a message as belonging to the RTPS protocol, and includes the vendor id.
* :class:`Sub-Message Header` Identifier for an RTPS sub-message. An RTPS Message can be composed of several sub-messages.
* :class:`MessageReceiver` Deserializes and processes received RTPS messages.
* :class:`RTPSMessageCreator` Composes RTPS messages.

RTPS Domain
^^^^^^^^^^^

* :class:`RTPSDomain` Use it to create, manage and destroy low-level RTPSParticipants.
* :class:`RTPSParticipant` Contains RTPS Writers and Readers, and manages their configuration.

    * :class:`RTPSParticipantAttributes` Configuration parameters used in the creation of an RTPS Participant.
    * :class:`PDPSimple` Allows the participant to become aware of the other participants within the Network, through the Participant Discovery Protocol.
    * :class:`EDPSimple` Allows the Participant to become aware of the endpoints (RTPS Writers and Readers) present in the other Participants within the network, through the Endpoint Discovery Protocol.
    * :class:`EDPStatic` Reads information about remote endpoints from a user file.
    * :class:`TimedEvent`  Base class for periodic or timed events.

RTPS Reader
^^^^^^^^^^^

* :class:`RTPSReader` Base class for the reader endpoint. 

    * :class:`ReaderAttributes` Configuration parameters used in the creation of an RTPS Reader.
    * :class:`ReaderHistory` History data structure. Stores recent topic changes.
    * :class:`ReaderListener` Use it to define callbacks in scope of the Reader.

RTPS Writer
^^^^^^^^^^^

* :class:`RTPSWriter` Base class for the writer endpoint.

    * :class:`WriterAttributes` Configuration parameters used in the creation of an RTPS Writer.
    * :class:`WriterHistory` History data structure. Stores outgoing topic changes and schedules them to be sent.

