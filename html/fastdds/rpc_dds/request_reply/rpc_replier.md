<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="rpc-replier"></a>

<a id="id1"></a>

# RPC Replier

A [Replier](#rpc-replier) is the RPC Entity used in the communication at the server side, processing the received Request samples
and sending Reply samples back to the Requester when the result of the operation is ready.

Since each Replier is used in only one Request/Reply communication, represented by a Service, all of them are registered
in a [Service](rpc_service.md#rpc-service) instance when created.
The name of the service associated with a Replier can be accessed using [`Replier::get_service_name()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4NK8eprosima7fastdds3dds3rpc7Replier16get_service_nameEv) method.

Similarly to the rest of the RPC entities, [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) instances can be enabled or disabled,
according to whether they contain DDS entities or not:

* Each active Replier contains two DDS entities: a [DataWriter](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter), which is used to publish Reply samples,
  and a [DataReader](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader), which is used to receive Request samples.
* Each disabled Replier does not contain any DDS entity and is ignored by the middleware.
  Therefore, it cannot take requests nor send replies.

For consistency, the states of each Replier instance and its associated Service
must follow the compatibility rule below:

$$
replier\_state \leq service\_state

$$

where `replier_state` is the state of the Replier instance and `service_state`
is the state of the associated Service, and the order of the states is defined as $disabled < enabled$.

<a id="creating-a-replier"></a>

## Creating a Replier

A new Replier instance can be created in an enabled or disabled Service using
[`DomainParticipant::create_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22create_service_replierEPN3rpc7ServiceERK10ReplierQosR12ReturnCode_t) method.
When a new Replier instance is created, it is registered internally in the Service.
Each Service can contain multiple Replier instances.

#### NOTE
Following the state compatibility rule, calling [`DomainParticipant::create_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22create_service_replierEPN3rpc7ServiceERK10ReplierQosR12ReturnCode_t) using a
disabled [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance as input parameter creates a disabled [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE).

Reciprocally, calling [`DomainParticipant::create_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22create_service_replierEPN3rpc7ServiceERK10ReplierQosR12ReturnCode_t) with an enabled [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance tries
to return an enabled [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE).
If the process of enabling the Replier fails (i.e., the creation of the internal DDS entities),
[`DomainParticipant::create_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22create_service_replierEPN3rpc7ServiceERK10ReplierQosR12ReturnCode_t) returns `nullptr`.

All Replier’s DDS entities Qos can be configured manually when
creating the [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) instance using [`ReplierQos`](../../api_reference/dds_pim/rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE).
Before creating the Replier, Service validates [`DataWriterQos`](../../api_reference/dds_pim/publisher/datawriterqos.md#_CPPv4N8eprosima7fastdds3dds13DataWriterQosE) and [`DataReaderQos`](../../api_reference/dds_pim/subscriber/datareaderqos.md#_CPPv4N8eprosima7fastdds3dds13DataReaderQosE) provided
in the [`ReplierQos`](../../api_reference/dds_pim/rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) instance:
if some field is not valid, Service will notify it to the user using a log message error and Replier is not created.

User must ensure that [`ReliabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKindE)
of both DataWriter and DataReader are set to [`RELIABLE_RELIABILITY_QOS`](../../api_reference/dds_pim/core/policy/reliabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds24ReliabilityQosPolicyKind24RELIABLE_RELIABILITY_QOSE).
This is configured automatically when a new [`ReplierQos`](../../api_reference/dds_pim/rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) instance is created.

#### NOTE
When a new [`ReplierQos`](../../api_reference/dds_pim/rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE) instance is created, [`HistoryQosPolicyKind`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKindE) and
[`DurabilityQosPolicyKind`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKindE) are by default [`KEEP_ALL_HISTORY_QOS`](../../api_reference/dds_pim/core/policy/historyqospolicykind.md#_CPPv4N8eprosima7fastdds3dds20HistoryQosPolicyKind20KEEP_ALL_HISTORY_QOSE) and [`VOLATILE_DURABILITY_QOS`](../../api_reference/dds_pim/core/policy/durabilityqospolicykind.md#_CPPv4N8eprosima7fastdds3dds23DurabilityQosPolicyKind23VOLATILE_DURABILITY_QOSE)
in both DataWriter and DataReader QoS, respectively.

#### NOTE
Before creating a new Replier, user must create its associated [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance in the DomainParticipant.
If a null pointer or a Service associated with a different participant are provided,
Replier is not created and [`DomainParticipant::create_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22create_service_replierEPN3rpc7ServiceERK10ReplierQosR12ReturnCode_t) method returns a null pointer.

<a id="enabling-and-disabling-a-replier"></a>

## Enabling and disabling a Replier

[`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) instances can be enabled or disabled using
[`enable()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity6enableEv) and [`close()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity5closeEv) methods, respectively.

When a disabled [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) instance is enabled, a new DataWriter on the Reply topic and
a new DataReader on Request topic are created using the Qos configured at creation through [`ReplierQos`](../../api_reference/dds_pim/rpc/replierqos.md#_CPPv4N8eprosima7fastdds3dds10ReplierQosE).

User can access to the DataWriter and DataReader instances using `Replier::get_replier_writer()` and
`Replier::get_replier_reader()` methods, respectively.

Reciprocally, when an enabled Replier is disabled, its respective DataWriter and DataReader are destroyed,
making the Replier not participate in the communication through the Service.

#### WARNING
A disabled Replier does not contain DDS entities, so `Replier::get_replier_writer()` and
`Replier::get_replier_reader()` return a null pointer in this case.
The user must be responsible for checking that the Replier is enabled
using the [`is_enabled()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4NK8eprosima7fastdds3dds3rpc9RPCEntity10is_enabledEv) method before accessing the Replier’s internal DDS entities.

<a id="deleting-a-replier"></a>

## Deleting a Replier

A [`Replier`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7ReplierE) instance can be unregistered from a Service and deleted using
[`DomainParticipant::delete_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22delete_service_replierERKNSt6stringEPN3rpc7ReplierE) method.

This method can be called on Repliers in any state: if enabled, it will try to disable the instance,
returning a `ReturnCode_t` error if it was not possible to close the Replier.

#### NOTE
If there is no [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) with the provided `service_name`
or the Replier is associated with a different Service,
[`DomainParticipant::delete_service_replier()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant22delete_service_replierERKNSt6stringEPN3rpc7ReplierE) method will return a `ReturnCode_t` error.

<a id="sending-and-receiving-samples"></a>

## Sending and receiving samples

Request samples are taken using the [`Replier::take_request()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7Replier12take_requestEPvR11RequestInfo) method.
When this method is called, Replier takes received Request samples from the history of the internal DataReader
and fills the `related_sample_identity` member of the provided [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) struct
with the `related_sample_identity`` of the received Request sample.

The Replier public API provides two overloads for [`Replier::take_request()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7Replier12take_requestEPvR11RequestInfo),
depending on whether you want to take only the next sample or all samples from the DataReader history.

#### NOTE
In case of using the overload that takes all samples, the loan must be returned to avoid memory problems.
The user must therefore be responsible for calling the [`Replier::return_loan()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7Replier11return_loanER18LoanableCollectionR16LoanableSequenceI11RequestInfoE) method after
finishing processing the samples.

After processing the Request, Replier sends a new Reply sample using the [`Replier::send_reply()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7Replier10send_replyEPvRK11RequestInfo) method
and passing the previously created [`RequestInfo`](../../api_reference/dds_pim/rpc/requestinfo.md#_CPPv4N8eprosima7fastdds3dds3rpc11RequestInfoE) as an input parameter.
When [`Replier::send_reply()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7Replier10send_replyEPvRK11RequestInfo) method is called,
the created DataWriter sends a new Reply sample with the provided data and
the `related_sample_identity` of the Request sample that originated the Reply,
to allow Request and Reply samples correlation at the Requester side.

#### WARNING
[`Replier::send_reply()`](../../api_reference/dds_pim/rpc/replier.md#_CPPv4N8eprosima7fastdds3dds3rpc7Replier10send_replyEPvRK11RequestInfo) method will fail if the communication with the Requester cannot be established,
returning `RETCODE_NO_DATA` if the Requester that sent the request is disconnected and `RETCODE_TIMEOUT`
if it is only partially matched (*i.e* if Request topic endpoints are matched but
Reply topic endpoints are not).

#### WARNING
RPC over DDS implementation in *Fast DDS* is designed to be incompatible with [Listeners](../../dds_layer/core/entity/entity.md#dds-layer-core-entity-commonchars-listener).
If user needs to process status changes, it can be done creating a [WaitSet](../../dds_layer/core/waitsets/waitsets.md#dds-layer-core-waitsets) on a different thread.

<a id="example"></a>

## Example

The following code snippet shows how to use a Replier instance:

```c++
ReturnCode_t ret;

// Get a valid service
Service* service = participant->find_service("Service");
if (!service)
{
    // Error
    return;
}

/* Create a new Replier instance */
ReplierQos replier_qos;

Replier* replier = participant->create_service_replier(service, replier_qos);
if (!replier)
{
    // Error
    return;
}

/* Take a received Request sample and send a new Reply sample */

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

//  Enabling the service enables the registered Replier unless
//  the creation of the internal DataWriter and DataReader failed.
//  We can make sure that they have been created correctly by checking if the Replier is enabled.
if (!replier->is_enabled())
{
    // Error
    return;
}

RequestInfo received_request_info;

// Wait for some time until a Request sample is received
replier->get_replier_reader()->wait_for_unread_message(Duration_t{3, 0});

void* received_data = nullptr;

ret = replier->take_request(received_data, received_request_info);
if (RETCODE_OK != ret)
{
    // Error
    return;
}

// ... Process received data

// Send a Reply with the received related_sample_identity, indicating there will be more replies.
void* reply_data = reply_type_support->create_data();
received_request_info.has_more_replies = true;
ret = replier->send_reply(reply_data, received_request_info);
if (RETCODE_OK != ret)
{
    // Error
    return;
}

// Send a Reply with the received related_sample_identity, indicating it is the last one.
void* reply_data_2 = reply_type_support->create_data();
received_request_info.has_more_replies = false;
ret = replier->send_reply(reply_data_2, received_request_info);
if (RETCODE_OK != ret)
{
    // Error
    return;
}

/* Delete created Replier */
ret = participant->delete_service_replier(replier->get_service_name(), replier);
if (RETCODE_OK != ret)
{
    // Error
    return;
}
```
