<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dds-layer-definition-data-types"></a>

<a id="definition-of-data-types"></a>

# Definition of data types

The definition of the data type exchanged in a [Topic](../topic/topic.md#dds-layer-topic-topic) is divided in
two classes: the [`TypeSupport`](../../../api_reference/dds_pim/topic/typesupport.md#_CPPv4N8eprosima7fastdds3dds11TypeSupportE) and the [`TopicDataType`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE).

TopicDataType describes the data type exchanged between a publication and a subscription, i.e.,
the data corresponding to a Topic.
The user has to create a specialized class for each specific type that will be used by the application.

Any specialization of TopicDataType must be registered in the [DomainParticipant](../../domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant)
before it can be used to create Topic objects.
A TypeSupport object encapsulates an instance of TopicDataType, providing the functions needed to
register the type and interact with the publication and subscription.
To register the data type, create a new TypeSupport with a TopicDataType instance
and use the [`register_type()`](../../../api_reference/dds_pim/topic/typesupport.md#_CPPv4NK8eprosima7fastdds3dds11TypeSupport13register_typeEP17DomainParticipant) member function on the TypeSupport.
Then the Topic can be created with the registered type name.

#### NOTE
Registering two different data types on the same DomainParticipant with identical names is not
allowed and will issue an error.
However, it is allowed to register the same data type within the same DomainParticipant,
with the same or different names.
If the same data type is registered twice on the same DomainParticipant with the same
name, the second registering will have no effect, but will not issue any error.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Register the data type in the DomainParticipant.
// If nullptr is used as name argument, the one returned by the type itself is used
TypeSupport custom_type_support(new CustomDataType());
custom_type_support.register_type(participant, nullptr);

// The previous instruction is equivalent to the following one
// Even if we are registering the same data type with the same name twice, no error will be issued
custom_type_support.register_type(participant, custom_type_support.get_type_name());

// Create a Topic with the registered type.
Topic* topic =
        participant->create_topic("topic_name", custom_type_support.get_type_name(), TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}

// Create an alias for the same data type using a different name.
custom_type_support.register_type(participant, "data_type_name");

// We can now use the aliased name to If no name is given, it uses the name returned by the type itself
Topic* another_topic =
        participant->create_topic("other_topic_name", "data_type_name", TOPIC_QOS_DEFAULT);
if (nullptr == another_topic)
{
    // Error
    return;
}
```

<a id="dds-layer-topic-dynamic-data-types"></a>

<a id="dynamic-data-types"></a>

## Dynamic data types

Instead of directly writing the specialized [`TopicDataType`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) class, it is possible to dynamically define
data types following the OMG Extensible and Dynamic Topic Types for DDS interface.
Data types can also be described on an XML file that is dynamically loaded.

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Load the XML file with the type description
DomainParticipantFactory::get_instance()->load_XML_profiles_file("example_type.xml");

// Retrieve the an instance of the desired type
DynamicTypeBuilder::_ref_type dyn_type_builder;
DomainParticipantFactory::get_instance()->get_dynamic_type_builder_from_xml_by_name("DynamicType",
        dyn_type_builder);

// Register dynamic type
TypeSupport dyn_type_support(new DynamicPubSubType(dyn_type_builder->build()));
dyn_type_support.register_type(participant, nullptr);

// Create a Topic with the registered type.
Topic* topic =
        participant->create_topic("topic_name", dyn_type_support.get_type_name(), TOPIC_QOS_DEFAULT);
if (nullptr == topic)
{
    // Error
    return;
}
```

A complete description of the dynamic definition of types can be found on the [XTypes](../../../xtypes/xtypes.md#dynamic-types) section.

<a id="dds-layer-topic-keyed-data-types"></a>

<a id="data-types-with-a-key"></a>

## Data types with a key

Data types that define a set of fields to form a unique key can distinguish different data sets
within the same data type.

To define a keyed Topic, the `getKey()` member function on the [`TopicDataType`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE)
has to be overridden to return the appropriate key value according to the data fields.
Additionally, the `m_isGetKeyDefined` data member needs to be set to `true` to let the entities
know that this is a keyed Topic and that `getKey()` should be used.
Types that do not define a key will have `m_isGetKeyDefined` set to false.

There are three ways to implement keys on the TopicDataType:

* Adding a `@Key` annotation to the members that form the key in the IDL file when using [Fast DDS-Gen](../../../../fastddsgen/introduction/introduction.md#fastddsgen-intro).
* Adding the attribute `Key` to the member and its parents when using [XTypes](../../../xtypes/xtypes.md#dynamic-types).
* Manually implementing the `getKey()` member function on the TopicDataType and setting
  the `m_isGetKeyDefined` data member value to `true`.

Data types with key are used to define data sub flows on a single Topic.
Data values with the same key on the same Topic represent data from the same sub-flow,
while data values with different keys on the same Topic represent data
from different sub-flows.
The middleware keeps these sub-flows separated, but all will be restricted to the same QoS values of
the Topic.
If no key is provided, the data set associated with the Topic is restricted to a single flow.

<a id="dds-layer-topic-type-support-context"></a>

<a id="type-support-context"></a>

## Type support context

Fast DDS allows passing a user-defined *context* object to the type support callbacks, enabling
type-specific information (e.g., upper bounds for strings and sequences) to be available during
serialization, deserialization, and related operations without needing global state.

The context is represented by the [`TopicDataType::Context`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType7ContextE) interface.
Users subclass it to carry whatever per-endpoint information their type requires:

```c++
struct MyContext : eprosima::fastdds::dds::TopicDataType::Context
{
    uint32_t string_max_length = 256;
    uint32_t sequence_max_length = 1024;
};
```

The context-aware virtual methods of [`TopicDataType`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataTypeE) all follow the same pattern: they receive a
`const std::shared_ptr<TopicDataType::Context>&` as their first argument, and their default
implementation ignores the context and delegates to the corresponding context-free method.
Users may override any subset of the following methods:

- [`serialize_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType13serialize_ctxERKNSt10shared_ptrI7ContextEEPCKvRN4rtps19SerializedPayload_tEN8eprosima7fastdds3dds22DataRepresentationId_tE) — serialize a sample using the context.
- [`deserialize_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType15deserialize_ctxERKNSt10shared_ptrI7ContextEERN4rtps19SerializedPayload_tEPv) — deserialize a payload using the context.
- [`calculate_serialized_size_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType29calculate_serialized_size_ctxERKNSt10shared_ptrI7ContextEEPCKvN8eprosima7fastdds3dds22DataRepresentationId_tE) — compute the serialized size with the context.
- [`create_data_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType15create_data_ctxERKNSt10shared_ptrI7ContextEE) — allocate a new sample using the context.
- [`delete_data_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType15delete_data_ctxERKNSt10shared_ptrI7ContextEEPv) — deallocate a sample using the context.
- [`compute_key_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType15compute_key_ctxERKNSt10shared_ptrI7ContextEERN4rtps19SerializedPayload_tERN4rtps16InstanceHandle_tEb) — extract the key using the context.
- [`is_bounded_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4NK8eprosima7fastdds3dds13TopicDataType14is_bounded_ctxERKNSt10shared_ptrI7ContextEE) — check whether the type is bounded with this context.
- [`is_plain_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4NK8eprosima7fastdds3dds13TopicDataType12is_plain_ctxERKNSt10shared_ptrI7ContextEE22DataRepresentationId_t) — check whether the type is plain with this context.
- [`construct_sample_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4NK8eprosima7fastdds3dds13TopicDataType20construct_sample_ctxERKNSt10shared_ptrI7ContextEEPv) — in-place construct a sample using the context.
- [`get_max_serialized_size_ctx()`](../../../api_reference/dds_pim/topic/topicdatatype.md#_CPPv4N8eprosima7fastdds3dds13TopicDataType27get_max_serialized_size_ctxERKNSt10shared_ptrI7ContextEE) — return the maximum serialized size with this context.

The context is attached to a [DataWriter](../../publisher/dataWriter/dataWriter.md#dds-layer-publisher-datawriter) or
[DataReader](../../subscriber/dataReader/dataReader.md#dds-layer-subscriber-datareader) **before enabling** the entity, using
[`DataWriter::set_type_support_context()`](../../../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriter24set_type_support_contextERKNSt10shared_ptrIN13TopicDataType7ContextEEE) or [`DataReader::set_type_support_context()`](../../../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReader24set_type_support_contextERKNSt10shared_ptrIN13TopicDataType7ContextEEE)
respectively.
Because the context is set before the entity is enabled, it is guaranteed to be available for every
write or read operation performed during the entity’s lifetime.

#### NOTE
**[Pro]** *Fast DDS Pro* provides [`BufferAwareContext`](../../../api_reference/utils/buffer/bufferawarecontext.md#_CPPv4N8eprosima7fastdds18BufferAwareContextE), a context that allows customizing how the
`Buffer<uint8_t>` members of a type are serialized and deserialized.
See [Custom buffers](../../../use_cases/custom_buffers/custom_buffers.md#use-case-custom-buffers)  for further details.
