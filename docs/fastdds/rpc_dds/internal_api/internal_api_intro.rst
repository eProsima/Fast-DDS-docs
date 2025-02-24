.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _internal_api_intro:

RPC over DDS internal API overview
==================================

*Fast DDS* |DomainParticipant| provides a high-level internal API
to create all the DDS entities required for RPC over DDS communication.
The API provides methods for creating and deleting three types of entities:

* |Service-api|:
  Represents the subset of DDS entities associated with a DomainParticipant
  that participate in the same RPC Request/Reply communication.
  This includes the topics where Requests and Replies are published and
  the endpoints responsible for publishing and receiving Requests and Replies,
  which are grouped in Requesters and Repliers.

* |Requester-api|:
  Represents the subset of DDS entities, associated with a Service instance,
  responsible for processing samples on the client side.
  This includes a |DataWriter| for publishing Request samples and
  a |DataReader| for receiving Reply samples.

* |Replier-api|:
  Represents the subset of DDS entities, associated with a Service instance,
  responsible for processing samples on the server side.
  This includes a DataWriter for publishing Reply samples and
  a DataReader for receiving Request samples.

All these entities inherit from the abstract base class |RPCEntity-api|,
which represents a generic entity in the context of RPC over DDS communication.
According to RPC over DDS Standard,
each entity has two different states: enabled and disabled.
A disabled entity is an entity that has been created
but is ignored by the middleware,
so it does not participate in the communication.

.. warning::
  Enabled and disabled states are not supported in the current version of *Fast DDS*.
  All entities are created with the default state (enabled) and it is ignored.

To match a request with a reply, Requesters and Repliers use a correlation mechanism.
When a Request sample is sent, Requester associates a unique identifier to it
represented by |RequestInfo-api| struct.
This identifier contains a |SampleIdentity-api| instance, which is formed by the Requester's DataWriter |Guid_t-api|
and a sequence number and can be accessed through |SampleInfo::related_sample_identity-api| member.
When a Replier takes a new received Request sample,
it extracts the |SampleInfo::related_sample_identity-api| information
and uses it to create a new |RequestInfo-api| which is then associated to the Reply sample.
Finally, when a Requester receives a Reply sample,
correlation of Requests and Replies is done by comparing both |RequestInfo-api| instances.

A request/reply communication can be established between different DomainParticipants
on the same domain if they have Requesters and Repliers in the same Service.
In a multiple Requester scenario (for example, one Requester per DomainParticipant),
reply samples sent by each Replier will be received by all Requester's DataReaders,
whether it corresponds to a request sample sent by its associated Requester or not.
Replier adds to each reply sample the |Guid_t-api| of the DataWriter which sent its associated request sample,
so reply samples are filtered on Requester side creating a |ContentFilteredTopic-api| from the Reply Topic.
The content filtered topic filters reply samples comparing the DataReader's |GuidPrefix_t-api| with
the |GuidPrefix_t-api| of the received |SampleInfo::related_sample_identity-api|,
so only replier samples associated to request samples sent by the same DomainParticipant are received.
When a Requester is created, its DataReader is created from this |ContentFilteredTopic-api| instance.

.. toctree::
  :maxdepth: 1

  /fastdds/rpc_dds/internal_api/rpc_service
  /fastdds/rpc_dds/internal_api/rpc_requester
  /fastdds/rpc_dds/internal_api/rpc_replier
