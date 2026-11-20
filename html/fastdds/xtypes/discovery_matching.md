<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="xtypes-discovery-matching"></a>

<a id="remote-data-types-discovery"></a>

# Remote Data Types Discovery

[DDS-XTypes specification](https://www.omg.org/spec/DDS-XTypes/1.3) defines an internal mechanism to discover the remote data types at runtime and match
depending on the extensible types compatibility rules configured using the [TypeConsistencyEnforcementQosPolicy](../dds_layer/core/policy/xtypesExtensions.md#typeconsistencyenforcementqospolicy).

#### NOTE
*eProsima Fast DDS* does not support XTypes compatibility check yet.

The remote data type discovery mechanism is based on the exchange of the data type information optimized in order to
reduce the required bandwidth.
On the one hand, `TypeInformation` structure defined in the IDL below (extracted from Annex B of the
[DDS-XTypes specification](https://www.omg.org/spec/DDS-XTypes/1.3)), is used to communicate the Topic Data Type and its dependencies.

```omg-idl
@extensibility(APPENDABLE) @nested
struct TypeIdentfierWithSize
{
    TypeIdentifier      type_id;
    unsigned long       typeobject_serialized_size;
};

@extensibility(APPENDABLE) @nested
struct TypeIdentifierWithDependencies
{
    TypeIdentfierWithSize           typeid_with_size;
    long                            dependent_type_id_count;
    sequence<TypeIdentfierWithSize> dependent_typeids;
};

@extensibility(MUTABLE) @nested
struct TypeInformation
{
    @id(0x1001) TypeIdentifierWithDependencies minimal;
    @id(0x1002) TypeIdentifierWithDependencies complete;
};
```

`TypeInformation` includes the `TypeIdentifier`, the data type information hashed which identifies almost
univocally the data type.
The data type information is contained in the `TypeObject` union:

```omg-idl
@extensibility(APPENDABLE) @nested
union TypeObject switch(octet)
{
    case EK_COMPLETE:
        CompleteTypeObject  complete;
    case EK_MINIMAL:
        MinimalTypeObject   minimal;
};
```

The `CompleteTypeObject` includes the data type full description.
On the other hand, `MinimalTypeObject` only includes the minimum required information in order to check type
compatibility.

#### IMPORTANT
Current TypeObject representation implementation does not support forward declarations or recursive data types
defined using the `@external` annotation.
Please, remember to disable TypeObject generation code using `-no-typeobjectsupport` option when generating
the code using Fast DDS-Gen.

<a id="prerequisites"></a>

## Prerequisites

The remote data type discovery feature only works if some requisites are met:

1. The local data types must be registered into the [`ITypeObjectRegistry`](../api_reference/dds_pim/xtypes/typeobjectregistry.md#_CPPv4N8eprosima7fastdds3dds6xtypes19ITypeObjectRegistryE).
   The types are automatically registered when calling [`register_type()`](../api_reference/dds_pim/domain/domainparticipant.md#_CPPv4N8eprosima7fastdds3dds17DomainParticipant13register_typeE11TypeSupportRKNSt6stringE) /
   [`register_type()`](../api_reference/dds_pim/topic/typesupport.md#_CPPv4NK8eprosima7fastdds3dds11TypeSupport13register_typeEP17DomainParticipant) if the code required for registration has been generated using
   [eProsima Fast DDS-Gen](../../fastddsgen/introduction/introduction.md#fastddsgen-intro).
   Fast DDS-Gen generates the required files ( *<IDLFileName>TypeObjectSupport.cxx/.hpp*) by default.

   #### NOTE
   `-no-typeobjectsupport` option disables the generation of these files and effectively disables the
   discovery of remote types.
2. `TypeInformation` should be received with the DomainParticipant’s endpoint discovery information.
   A DomainParticipant that does not inform about its `TypeInformation` would not trigger the remote data type
   discovery mechanism.

If the prerequisites are not met, endpoint matching relies on type name and topic name in order to match the discovered
endpoints.

<a id="xtypes-discovery-matching-config"></a>

<a id="configuration"></a>

## Configuration

The level of propagation of local data types can be configured as specified in [Type Propagation](../property_policies/non_consolidated_qos.md#property-type-propagation).

<a id="remote-types-discovery-example"></a>

## Remote types discovery example

Please, refer to [Remote type discovery and endpoint matching](../use_cases/remote_type_discovery_matching/remote_type_discovery_matching.md#use-case-remote-type-discovery-and-matching) for more information about how to leverage this
feature.
