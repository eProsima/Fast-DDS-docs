<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="rpc-service"></a>

<a id="id1"></a>

# RPC Service

A Service is the RPC entity used to register all RPC entities involved in an RPC communication,
both on the client side ([Requesters](rpc_requester.md#rpc-requester)) and on the server side ([Repliers](rpc_replier.md#rpc-replier)).
Two or more participants can communicate with each other via RPC if they register the same service
and create Requesters and Repliers in it.

A service is represented by its name and a Service type,
so that two [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instances represent the same RPC service if they have the same name and type.
The name of the Service and the name of its type can be obtained through the methods [`get_service_name()`](../../api_reference/dds_pim/rpc/service.md#_CPPv4NK8eprosima7fastdds3dds3rpc7Service16get_service_nameEv)
and [`get_service_type_name()`](../../api_reference/dds_pim/rpc/service.md#_CPPv4NK8eprosima7fastdds3dds3rpc7Service21get_service_type_nameEv) respectively.

Similarly to the rest of the RPC entities, [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instances can be enabled or disabled,
according to whether they contain DDS entities or not:

* Each active Service contains at least two DDS entities: a Request [Topic](../../dds_layer/topic/topic.md#dds-layer-topic),
  which is used to publish and receive Request samples,
  and a Reply [Topic](../../dds_layer/topic/topic.md#dds-layer-topic), which is used to publish and receive Reply samples.
  If it also contains enabled Requesters and Repliers,
  it also includes their respective internal [DataWriters](../../dds_layer/publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) and [DataReaders](../../dds_layer/subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader).
  Therefore, an enabled [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance can also be understood as the subset of DDS entities
  associated with a [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) that participate in the same communication through RPC.
* Each disabled Replier does not contain any DDS entity and is ignored by the middleware.
  Therefore, it cannot take requests nor send replies.

<a id="service-types"></a>

## Service types

Due to the fact that a Service represents a Request/Reply communication,
a Service type is represented by a pair of topic types: the Request topic type and the Reply topic type.
A new Service type can be defined creating an instance of [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) class,
which is constructed using [`TypeSupport`](../../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) instances of both Request and Reply types.

[`DomainParticipant`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipantE) class have [`register_service_type()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant21register_service_typeEN3rpc18ServiceTypeSupportERKNSt6stringE) and
[`unregister_service_type()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant23unregister_service_typeERKNSt6stringE) methods
to register and unregister a [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) instance, respectively.

#### NOTE
Request and Reply types associated with a service type named `<service_type_name>` are registered
in the participant with the names `<service_type_name>_Request`` and
`<service_type_name>_Reply`, respectively.

A registered [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) can be found by name using [`find_service_type()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant17find_service_typeERKNSt6stringE) method;
if a Service type with the provided name is not found, an empty [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) is returned.

#### NOTE
Trying to register a different [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) instance with a
`service_type_name` already registered will return a `ReturnCode_t` error.

Registering and unregistering a [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) instance involves
registering and unregistering the Request and Reply types in the participant, respectively.
If a [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) contains a Request or Reply type with the same name
as a previously registered type but with a different [`TypeSupport`](../../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE),
the [`register_service_type()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant21register_service_typeEN3rpc18ServiceTypeSupportERKNSt6stringE) method will fail.
Similarly, attempting to unregister a [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) instance with a Request or Reply type
that is used by a DDS endpoint on a topic not created by the Service will fail.
To avoid these issues, it is recommended to make sure that the Request and Reply types
are reserved for RPC communication though a unique service and
not used by external DDS entities which are not part of the Service.

<a id="creating-and-deleting-a-service"></a>

## Creating and deleting a Service

A DomainParticipant can create and register a
[`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance using [`DomainParticipant::create_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant14create_serviceERKNSt6stringERKNSt6stringER12ReturnCode_t) method.
Respectively, a [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) can be unregistered and deleted using [`DomainParticipant::delete_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant14delete_serviceEPKN3rpc7ServiceE) method.
Each registered Service can be found by name using [`DomainParticipant::find_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4NK8eprosima7fastdds3dds17DomainParticipant12find_serviceERKNSt6stringE) method.

#### NOTE
Before creating a new Service,
user must register the [`ServiceTypeSupport`](../../api_reference/dds_pim/rpc/servicetypesupport.md#_CPPv4N8eprosima7fastdds3dds3rpc18ServiceTypeSupportE) instance associated with the Service type.
If there is no Service type registered with the `service_type_name` provided,
Service is not created and [`DomainParticipant::create_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant14create_serviceERKNSt6stringERKNSt6stringER12ReturnCode_t) method will return a null pointer.

When a new Service is created, it is enabled by default. It means that DomainParticipant creates the topics
used for publishing and receiving Request and Reply samples.

#### NOTE
If the DomainParticipant is unable to enable the Service when it is created, it is considered an error.
The instance is destroyed, and [`DomainParticipant::create_service()`](../../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant14create_serviceERKNSt6stringERKNSt6stringER12ReturnCode_t) returns `nullptr`.

Only the DomainParticipant that created the [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance can delete it.
Attempting to delete a [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance from a different DomainParticipant
will result in a `ReturnCode_t` error.

<a id="enabling-and-disabling-a-service"></a>

## Enabling and disabling a Service

[`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instances can be enabled or disabled using
[`enable()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity6enableEv) and [`close()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity5closeEv) methods, respectively.

When a disabled [`Service`](../../api_reference/dds_pim/rpc/service.md#_CPPv4N8eprosima7fastdds3dds3rpc7ServiceE) instance is enabled, new Request and Reply topics are created in the DomainParticipant
using the registered Service type. It also creates the content filtered topic used for Reply samples.
Additionally, if it contains Requesters and Repliers, an attempt will be made to enable all possible entities.

#### NOTE
Request and Reply topics are created with names
`<service_name>_Request` and `<service_name>_Reply`, respectively.
The [`ContentFilteredTopic`](../../api_reference/dds_pim/topic/contentfilteredtopic.md#_CPPv4N8eprosima7fastdds3dds20ContentFilteredTopicE) instance is also created
from the Reply topic instance with name `<service_name>_ReplyFiltered`.

#### NOTE
If some of the RPCEntities within the Service cannot be enabled, an error log is displayed.
However, [`enable()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity6enableEv) ignores this failure and returns `RETCODE_OK`` as long as
the topics have been created successfully.

Reciprocally, when an enabled Service is disabled, all the DDS entities that the Service contains are destroyed,
making the Service not participate in the communication through RPC.
It implies disabling all containing Requesters and Repliers and deleting the Request and Reply topics.

#### NOTE
If some of the RPCEntities within the Service cannot be disabled,
[`close()`](../../api_reference/dds_pim/rpc/rpcentity.md#_CPPv4N8eprosima7fastdds3dds3rpc9RPCEntity5closeEv) returns an error code.

<a id="example"></a>

## Example

The following code snippet shows how to create and delete a Service instance:

```c++
TypeSupport request_type_support = TypeSupport(new CustomDataType());
TypeSupport reply_type_support = TypeSupport(new CustomDataType());
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);

// Create a new ServiceTypeSupport instance
ServiceTypeSupport service_type_support(request_type_support, reply_type_support);

// Register the ServiceTypeSupport instance in a DomainParticipant
ReturnCode_t ret;
ret = participant->register_service_type(service_type_support, "ServiceType");
if (RETCODE_OK != ret)
{
    // Error
    return;
}

// Create a new Service instance
Service* service = participant->create_service("Service", "ServiceType");
if (!service)
{
    // Error
    return;
}

// ... Create Requesters and Repliers here

// Delete the created Service
ret = participant->delete_service(service);
if (RETCODE_OK != ret)
{
    // Error
    return;
}
```
