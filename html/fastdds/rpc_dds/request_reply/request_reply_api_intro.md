<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="request-reply-api-intro"></a>

<a id="rpc-over-dds-request-reply-api-overview"></a>

# RPC over DDS Request-Reply API overview

*Fast DDS* [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) provides a high-level Request-Reply API
to create all the DDS entities required for RPC over DDS communication.
The API provides methods for creating and deleting three types of entities:

* [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE):
  Represents the subset of DDS entities associated with a DomainParticipant
  that participate in the same RPC Request/Reply communication.
  This includes the topics where Requests and Replies are published and
  the endpoints responsible for publishing and receiving Requests and Replies,
  which are grouped in Requesters and Repliers.
* [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE):
  Represents the subset of DDS entities, associated with a Service instance,
  responsible for processing samples on the client side.
  This includes a [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) for publishing Request samples and
  a [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) for receiving Reply samples.
* [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE):
  Represents the subset of DDS entities, associated with a Service instance,
  responsible for processing samples on the server side.
  This includes a DataWriter for publishing Reply samples and
  a DataReader for receiving Request samples.

All these entities inherit from the abstract base class [`RPCEntity`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntityE),
which represents a generic entity in the context of RPC over DDS communication.
According to RPC over DDS Standard,
each entity has two different states: enabled and disabled.
A disabled entity is an entity that has been created
but is ignored by the middleware,
so it does not participate in the communication.

User can enable and disable an entity using [`enable()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity6enableEv) and [`close()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity5closeEv) methods, respectively.
Internally, when an entity is enabled, the middleware creates the DDS entities associated with it.
Similarly, when an entity is disabled, the middleware deletes their DDS entities.

#### NOTE
Trying to use an RPC entity (Requester or Replier) for RPC communication when it is disabled will return an error.
The user can check if an entity is enabled using [`is_enabled()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4NK8eprosima7fastdds3dds3rpc9RPCEntity10is_enabledEv) method.

To match a request with a reply, Requesters and Repliers use a correlation mechanism.
When a Request sample is sent, Requester associates a unique identifier to it
represented by [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) struct.

This identifier contains a [`SampleIdentity`](../../api_reference/rtps/common/SampleIdentity/SampleIdentity.md#_CPPv4N8eprosima7fastdds4rtps14SampleIdentityE) instance, which is formed by the Requester’s DataWriter [`Guid_t`](../../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE)
and a sequence number, and can be accessed through [`related_sample_identity`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo23related_sample_identityE) member.
When a Replier takes a new received Request sample,
it extracts the [`related_sample_identity`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo23related_sample_identityE) information
and uses it to create a new [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) which is then associated to the Reply sample.
The Replier can use the [`has_more_replies`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo16has_more_repliesE) field to indicate that additional replies for the same
request will be sent.
The Requester can take special actions when that field is set.

[`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) works like a way to match a Reply with a previously sent Request:
when a Requester receives a Reply sample,
correlation of Requests and Replies is done by comparing both [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) instances. If they are the same,
the Reply sample is associated with the Request sample.

A request/reply communication can be established between different DomainParticipants
on the same domain if they have Requesters and Repliers in the same Service.
In a multiple Requester scenario (for example, one Requester per DomainParticipant),
reply samples sent by each Replier will be received by all Requester’s DataReaders,
whether it corresponds to a request sample sent by its associated Requester or not.
Replier adds to each reply sample the [`Guid_t`](../../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) of the DataWriter which sent its associated request sample,
so reply samples are filtered on Requester side creating a [`ContentFilteredTopic`](../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) from the Reply Topic.
The content filtered topic filters reply samples comparing the DataReader’s [`GuidPrefix_t`](../../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) with
the [`GuidPrefix_t`](../../api_reference/rtps/common/GuidPrefix_t/GuidPrefix_t.md#_CPPv4N8eprosima7fastdds4rtps12GuidPrefix_tE) of the received [`related_sample_identity`](../../api_reference/dds_pim/subscriber/sampleinfo.md#_CPPv4N8eprosima7fastdds3dds10SampleInfo23related_sample_identityE),
so only replier samples associated to request samples sent by the same DomainParticipant are received.
When a Requester is created, its DataReader is created from this [`ContentFilteredTopic`](../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) instance.

* [RPC Service](rpc_service.md)
  * [Service types](rpc_service.md#service-types)
  * [Creating and deleting a Service](rpc_service.md#creating-and-deleting-a-service)
  * [Enabling and disabling a Service](rpc_service.md#enabling-and-disabling-a-service)
  * [Example](rpc_service.md#example)
* [RPC Replier](rpc_replier.md)
  * [Creating a Replier](rpc_replier.md#creating-a-replier)
  * [Enabling and disabling a Replier](rpc_replier.md#enabling-and-disabling-a-replier)
  * [Deleting a Replier](rpc_replier.md#deleting-a-replier)
  * [Sending and receiving samples](rpc_replier.md#sending-and-receiving-samples)
  * [Example](rpc_replier.md#example)
* [RPC Requester](rpc_requester.md)
  * [Creating a Requester](rpc_requester.md#creating-a-requester)
  * [Enabling and disabling a Requester](rpc_requester.md#enabling-and-disabling-a-requester)
  * [Deleting a Requester](rpc_requester.md#deleting-a-requester)
  * [Sending and receiving samples](rpc_requester.md#sending-and-receiving-samples)
  * [Example](rpc_requester.md#example)
