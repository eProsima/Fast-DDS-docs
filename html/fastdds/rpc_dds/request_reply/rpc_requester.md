<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="rpc-requester"></a>

<a id="id1"></a>

# RPC Requester

A [Requester](#rpc-requester) is the RPC Entity used in the communication at the client side,
sending Request samples and processing the received Reply samples.

Since each Requester is used in only one Request/Reply communication, represented by a Service,
all of them are registered in a [Service](rpc_service.md#rpc-service) instance when created.
The name of the service associated with a Requester can be accessed using [`Requester::get_service_name()`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4NK8eprosima7fastdds3dds3rpc9Requester16get_service_nameEv) method.

Similarly to the rest of the RPC entities, [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) instances can be enabled or disabled,
according to whether they contain DDS entities or not:

* Each active Requester contains two DDS entities: a [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), which is used to publish Request samples,
  and a [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader), which is used to receive Reply samples.
* Each disabled Requester does not contain any DDS entity and is ignored by the middleware.
  Therefore, it cannot send requests nor take replies.

For consistency, the states of each Requester instance and its associated Service
must follow the compatibility rule below:

$$
requester\_state \leq service\_state

$$

where `requester_state` is the state of the Requester instance and `service_state`
is the state of the associated Service, and the order of the states is defined as $disabled < enabled$.

<a id="creating-a-requester"></a>

## Creating a Requester

A new Requester instance can be created in an enabled or disabled Service using
[`DomainParticipant::create_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24create_service_requesterEPN3rpc7ServiceERK12RequesterQosR12ReturnCode_t) method.
When a new Requester instance is created, it is registered internally in the Service.
Each Service can contain multiple Requester instances.

#### NOTE
Following the state compatibility rule, calling [`DomainParticipant::create_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24create_service_requesterEPN3rpc7ServiceERK12RequesterQosR12ReturnCode_t) using a
disabled [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance as input parameter creates a disabled [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE).

Reciprocally, calling [`DomainParticipant::create_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24create_service_requesterEPN3rpc7ServiceERK12RequesterQosR12ReturnCode_t) with an enabled [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance tries
to return an enabled [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE).
If the process of enabling the Requester fails (i.e., the creation of the internal DDS entities),
[`DomainParticipant::create_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24create_service_requesterEPN3rpc7ServiceERK12RequesterQosR12ReturnCode_t) returns `nullptr`.

All Requester’s DDS entities Qos can be configured manually when
creating the [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) instance using [`RequesterQos`](../../api_reference/dds_pim/rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE).
Before creating the Requester, Service validates [`DataWriterQos`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) and [`DataReaderQos`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) provided
in the [`RequesterQos`](../../api_reference/dds_pim/rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) instance:
if some field is not valid, Service will notify it to the user using a log message error and Requester is not created.

User must ensure that [`ReliabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE)
of both DataWriter and DataReader are set to [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).
This is configured automatically when a new [`RequesterQos`](../../api_reference/dds_pim/rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) instance is created.

#### NOTE
When a new [`RequesterQos`](../../api_reference/dds_pim/rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE) instance is created, [`HistoryQosPolicyKind`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE) and
[`DurabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE) are by default [`KEEP_ALL_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE) and [`VOLATILE_DURABILITY_QOS`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)
in both DataWriter and DataReader QoS, respectively.

#### NOTE
Before creating a new Requester, user must create its associated [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance in the DomainParticipant.
If a null pointer or a Service associated with a different participant are provided,
Requester is not created and [`DomainParticipant::create_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24create_service_requesterEPN3rpc7ServiceERK12RequesterQosR12ReturnCode_t) method returns a null pointer.

<a id="enabling-and-disabling-a-requester"></a>

## Enabling and disabling a Requester

[`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) instances can be enabled or disabled using
[`enable()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity6enableEv) and [`close()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity5closeEv) methods, respectively.

When a disabled [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) instance is enabled, a new DataReader on the content filtered Reply topic and
a new DataWriter on Request topic are created using the Qos configured at creation through [`RequesterQos`](../../api_reference/dds_pim/rpc/requesterqos.md#_CPPv4N8eprosima7fastdds3dds12RequesterQosE).

User can access to the DataWriter and DataReader instances using `Requester::get_requester_writer()` and
`Requester::get_requester_reader()` methods, respectively.

Reciprocally, when an enabled Requester is disabled, its respective DataWriter and DataReader are destroyed,
making the Requester not participate in the communication through the Service.

#### WARNING
A disabled Requester does not contain DDS entities, so `Requester::get_requester_writer()` and
`Requester::get_requester_reader()` return a null pointer in this case.
The user must be responsible for checking that the Requester is enabled
using the [`is_enabled()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4NK8eprosima7fastdds3dds3rpc9RPCEntity10is_enabledEv) method before accessing the Requester’s internal DDS entities.

<a id="deleting-a-requester"></a>

## Deleting a Requester

A [`Requester`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9RequesterE) instance can be unregistered from a Service and deleted using
[`DomainParticipant::delete_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24delete_service_requesterERKNSt6stringEPN3rpc9RequesterE) method.

This method can be called on Requesters in any state: if enabled, it will try to disable the instance,
returning a `ReturnCode_t` error if it was not possible to close the Requester.

#### NOTE
If there is no [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) with the provided `service_name`
or the Requester is associated with a different Service,
[`DomainParticipant::delete_service_requester()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant24delete_service_requesterERKNSt6stringEPN3rpc9RequesterE) method will return a `ReturnCode_t` error.

<a id="sending-and-receiving-samples"></a>

## Sending and receiving samples

Request samples can be sent using the [`Requester::send_request()`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9Requester12send_requestEPvR11RequestInfo) method.
When this method is called, the created DataWriter sends a new Request sample with the provided data and
fills the `related_sample_identity` field in the input [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) struct
with the [`Guid_t`](../../api_reference/rtps/common/Guid/GUID_t.md#_CPPv4N8eprosima7fastdds4rtps6GUID_tE) of the DataWriter and a sequence number.

Similarly, received Reply samples are taken using [`Requester::take_reply()`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9Requester10take_replyEPvR11RequestInfo) method.
When this method is called, Requester takes received Reply samples from the history of the DataReader
and fills the provided [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) struct with the `related_sample_identity` of the received Reply sample.
This way, user can match Request and Reply samples by checking if the `related_sample_identity`
attributes of their [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) instances are equal.

The Requester public API provides two overloads for [`Requester::take_reply()`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9Requester10take_replyEPvR11RequestInfo),
depending on whether you want to take only the next sample or all samples from the DataReader history.

#### NOTE
In case of using the overload that takes all samples, the loan must be returned to avoid memory problems.
The user must therefore be responsible for calling the [`Requester::return_loan()`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9Requester11return_loanER18LoanableCollectionR16LoanableSequenceI11RequestInfoE) method after
finishing processing the samples.

#### WARNING
If a Request sample is sent before matching a [Replier](rpc_replier.md#rpc-replier) completely, [`Requester::send_request()`](../../api_reference/dds_pim/rpc/requester.md#_CPPv4N8eprosima7fastdds3dds3rpc9Requester12send_requestEPvR11RequestInfo) method
will fail, returning `RETCODE_PRECONDITION_NOT_MET`.

#### WARNING
RPC over DDS implementation in *Fast DDS* is designed to be incompatible with [Listeners](../../dds_layer/core/entity/entity.md#dds-layer-core-entity-commonchars-listener).
If user needs to process status changes, it can be done creating a [WaitSet](../../dds_layer/core/waitsets/waitsets.md#dds-layer-core-waitsets) on a different thread.

<a id="example"></a>

## Example

The following code snippet shows how to use a Requester instance:

```c++
ReturnCode_t ret;

// Get a valid service
Service* service = participant->find_service("Service");
if (!service)
{
    // Error
    return;
}

/* Create a new Requester instance */
RequesterQos requester_qos;

Requester* requester = participant->create_service_requester(service, requester_qos);
if (!requester)
{
    // Error
    return;
}

/* Send a new Request sample and check if a received Reply sample is a match */
// Make sure that all RPC Entities are enabled
if (!service->is_enabled())
{
    ret = service->enable();
    if (RETCODE_OK != ret)
    {
        // Error
        return;
    }
}

//  Enabling the service enables the registered Requester unless
//  the creation of the internal DataWriter and DataReader failed.
//  We can make sure that they have been created correctly by checking if the Requester is enabled.
if (!requester->is_enabled())
{
    // Error
    return;
}

RequestInfo expected_request_info;
RequestInfo received_request_info;
// Create a new Request sample
void* request_data = request_type_support->create_data();
ret = requester->send_request(request_data, expected_request_info);
if (RETCODE_OK != ret)
{
    // Error
    return;
}

// Wait for some time until a Reply sample is received
requester->get_requester_reader()->wait_for_unread_message(Duration_t{3, 0});

void* data = nullptr;
ret = requester->take_reply(data, received_request_info);
if (RETCODE_OK != ret)
{
    // Error
    return;
}

if (expected_request_info.related_sample_identity == received_request_info.related_sample_identity)
{
    // Received Reply sample is associated to the sent Request sample
    if (received_request_info.has_more_replies)
    {
        // More replies for the same request will be received
    }
}

// Delete created Requester
ret = participant->delete_service_requester(requester->get_service_name(), requester);
if (RETCODE_OK != ret)
{
    // Error
    return;
}
```
