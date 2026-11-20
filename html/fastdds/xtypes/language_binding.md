<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="xtypes-language-binding"></a>

<a id="dynamic-language-binding"></a>

# Dynamic Language Binding

The Dynamic Language Binding API allows to define data types at runtime instead of having the types predefined as it is
required by the Plain Language Binding.
This API includes both the type definition and, the getters and setters required to use the defined types.
Type definition can also be done using a XML configuration file as explained in [Dynamic Types profiles](../xml_configuration/dynamic_types.md#xmldynamictypes) section or by
parsing an IDL file at runtime, as explained in [Dynamic Types IDL Parsing](idl_parsing.md#dynamic-types-idl-parsing) section.

This section presents first the Dynamic Language Binding API, and then the supported types and specific examples
defining and using those types.

<a id="xtypes-interfaces"></a>

<a id="dynamic-language-binding-interfaces"></a>

## Dynamic Language Binding Interfaces

This section briefly presents the Dynamic Language Binding API.
For more information, please refer both to the [DDS-XTypes specification](https://www.omg.org/spec/DDS-XTypes/1.3)
and the [API reference](../api_reference/dds_pim/xtypes/dynamic_types.md#dynamic-language-binding-api).

<a id="typedescriptor"></a>

### TypeDescriptor

[`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) is in charge of describing the state of a type.
Objects of this interface have value semantics allowing the TypeDescriptor data to be deeply copied and compared.

<a id="annotationdescriptor"></a>

### AnnotationDescriptor

[`AnnotationDescriptor`](../api_reference/dds_pim/xtypes/annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE) is in charge of describing the user-defined applied annotation to a specific element.
Objects of this interface have value semantics allowing the AnnotationDescriptor data to be deeply copied and compared.

<a id="memberdescriptor"></a>

### MemberDescriptor

[`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) is in charge of describing the state of a specific member of a type.
Objects of this interface have value semantics allowing the MemberDescriptor data to be deeply copied and compared.

<a id="verbatimtextdescriptor"></a>

### VerbatimTextDescriptor

[`VerbatimTextDescriptor`](../api_reference/dds_pim/xtypes/verbatimtextdescriptor.md#_CPPv4N8eprosima7fastdds3dds22VerbatimTextDescriptorE) is in charge of describing the `@verbatim` builtin annotation application.
Objects of this interface have value semantics allowing the VerbatimTextDescriptor data to be deeply copied and
compared.

<a id="dynamictypebuilderfactory"></a>

### DynamicTypeBuilderFactory

The [`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) serves as a singleton which instance is responsible for both creating and deleting
[`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) objects.
This class provides the generic [`DynamicTypeBuilderFactory::create_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory11create_typeEN6traitsI14TypeDescriptorE8ref_typeE) API, and also specific APIs to define other
basic types such as strings, sequences, etc.
More information can be found in [Supported Types](#xtypes-supportedtypes) section.

<a id="dynamictype"></a>

### DynamicType

[`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE) objects represents a specific type definition.
Once the DynamicType has been built, it cannot be modified.
Objects of this interface have reference semantics, so the API receives a nil-reference which is then returned pointing
to the correct DynamicType address.

<a id="dynamictypemember"></a>

### DynamicTypeMember

[`DynamicTypeMember`](../api_reference/dds_pim/xtypes/dynamictypemember.md#_CPPv4N8eprosima7fastdds3dds17DynamicTypeMemberE) represents a data member of a DynamicType.
Objects of this interface have reference semantics, so the API receives a nil-reference which is then returned pointing
to the correct DynamicTypeMember address.

<a id="dynamictypebuilder"></a>

### DynamicTypeBuilder

[`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) interface allows the instantiation of concrete DynamicType objects and serves as a transitional
state for configuring the DynamicType before its creation.
Upon definition, DynamicTypeBuilderFactory leverages the information contained in the builder to create
the DynamicType.
[`DynamicTypeBuilder::build`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder5buildEv) allows for creating the fully constructed DynamicType.
Builders remain reusable after DynamicType creation, ensuring changes to the builder do not affect previously
created types.

<a id="dynamicdatafactory"></a>

### DynamicDataFactory

[`DynamicDataFactory`](../api_reference/dds_pim/xtypes/dynamicdatafactory.md#_CPPv4N8eprosima7fastdds3dds18DynamicDataFactoryE) serves as a singleton which instance is responsible for both creating and deleting
[`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) objects from a given DynamicType instance.

<a id="dynamicdata"></a>

### DynamicData

[`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) represents a data instance of a DynamicType, providing functionalities to access
and modify data values.
Each DynamicData object corresponds to an object of the type represented by its DynamicType.
Offering reflective getters and setters, DynamicData enables manipulation of individual data samples.

<a id="xtypes-supportedtypes"></a>

<a id="supported-types"></a>

## Supported Types

This section describes the supported Type System including examples of how to instantiate those specific types using the
Dynamic Language Binding API and the XML configuration file.
The C++ examples also include instantiating the corresponding DynamicData sample, and setting and reading a value.

The table below summarizes which types can be used with each of the ways of building a dynamic type at runtime.
Data types defined in an IDL file and processed with *Fast DDS-Gen* to generate code are covered by
[Supported IDL types](../../fastddsgen/dataTypes/dataTypes.md#fastddsgen-supported-idl-types) instead.

| Type                                                        | Dynamic Language Binding support                          | XML Dynamic Type profiles support                         | IDL Parsing Dynamic Type support                          |
|-------------------------------------------------------------|-----------------------------------------------------------|-----------------------------------------------------------|-----------------------------------------------------------|
| [Primitive Types](#xtypes-supportedtypes-primitive)         | ✅                                                         | ✅                                                         | ✅                                                         |
| [String Types](#xtypes-supportedtypes-string)               | ✅                                                         | ✅                                                         | ✅                                                         |
| [Enumeration Types](#xtypes-supportedtypes-enumeration)     | ✅                                                         | ✅                                                         | ✅                                                         |
| [Alias Types](#xtypes-supportedtypes-alias)                 | ✅                                                         | ✅                                                         | ✅                                                         |
| [Sequence types](#xtypes-supportedtypes-sequence)           | ✅                                                         | ✅                                                         | ✅                                                         |
| [Array types](#xtypes-supportedtypes-array)                 | ✅                                                         | ✅                                                         | ✅                                                         |
| [Map Types](#xtypes-supportedtypes-map)                     | ✅                                                         | ✅                                                         | ❌                                                         |
| [Structure Types](#xtypes-supportedtypes-structure)         | ✅                                                         | ✅                                                         | ✅                                                         |
| [Structure Inheritance](#xtypes-supportedtypes-inheritance) | ✅                                                         | ❌                                                         | ✅ **[Pro]**                                               |
| [Union Types](#xtypes-supportedtypes-union)                 | ✅                                                         | ✅                                                         | ✅                                                         |
| [Bitmask Types](#xtypes-supportedtypes-bitmask)             | ✅                                                         | ✅                                                         | ✅ **[Pro]**                                               |
| [Bitset Types](#xtypes-supportedtypes-bitset)               | ✅                                                         | ⚠️ (inheritance not supported)                            | ✅ **[Pro]**                                               |
| [Annotations](#xtypes-annotations)                          | See the [annotations table](#xtypes-builtin-annotations). | See the [annotations table](#xtypes-builtin-annotations). | See the [annotations table](#xtypes-builtin-annotations). |

<a id="xtypes-supportedtypes-primitive"></a>

<a id="primitive-types"></a>

### Primitive Types

Primitive types are self-describing and can be created without configuration parameters.
The [`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) interface exposes the method [`DynamicTypeBuilderFactory::get_primitive_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory18get_primitive_typeE8TypeKind)
to allow users to directly get the corresponding primitive DynamicType.
The [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) class provides specific getters and setters for each primitive data type.

The following table shows the supported primitive types and their corresponding `TypeKind`.
The `TypeKind` is used to query the DynamicTypeBuilderFactory for the specific primitive DynamicType.

| C++ Type      | TypeKind               |
|---------------|------------------------|
| `bool`        | `TK_BOOLEAN`           |
| `char`        | `TK_CHAR8`             |
| `wchar_t`     | `TK_CHAR16`            |
| `uint8_t`     | `TK_BYTE` / `TK_UINT8` |
| `int8_t`      | `TK_INT8`              |
| `int16_t`     | `TK_INT16`             |
| `uint16_t`    | `TK_UINT16`            |
| `int32_t`     | `TK_INT32`             |
| `uint32_t`    | `TK_UINT32`            |
| `int64_t`     | `TK_INT64`             |
| `uint64_t`    | `TK_UINT64`            |
| `float`       | `TK_FLOAT32`           |
| `double`      | `TK_FLOAT64`           |
| `long double` | `TK_FLOAT128`          |

The example below shows how to create an structure with primitive members.

### OMG-IDL

```omg-idl
struct PrimitivesStruct
{
    boolean my_bool;
    octet my_octet;
    char my_char;
    wchar my_wchar;
    long my_long;
    unsigned long my_ulong;
    int8 my_int8;
    uint8 my_uint8;
    short my_short;
    unsigned short my_ushort;
    long long my_longlong;
    unsigned long long my_ulonglong;
    float my_float;
    double my_double;
    long double my_longdouble;
};
```

### XML

```xml
<struct name="PrimitivesStruct">
    <member name="my_bool" type="boolean"/>
    <member name="my_octet" type="byte"/>
    <member name="my_char" type="char8"/>
    <member name="my_wchar" type="char16"/>
    <member name="my_long" type="int32"/>
    <member name="my_ulong" type="uint32"/>
    <member name="my_int8" type="int8"/>
    <member name="my_uint8" type="uint8"/>
    <member name="my_short" type="int16"/>
    <member name="my_ushort" type="uint16"/>
    <member name="my_longlong" type="int64"/>
    <member name="my_ulonglong" type="uint64"/>
    <member name="my_float" type="float32"/>
    <member name="my_double" type="float64"/>
    <member name="my_longdouble" type="float128"/>
</struct>
```

### C++

```c++
// Define a struct type with various primitive members
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("PrimitivesStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};

// Define and add primitive members to the struct
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->name("my_bool");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_octet");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BYTE));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_char");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR8));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_wchar");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR16));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_long");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_ulong");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT32));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_int8");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT8));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_uint8");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT8));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_short");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_ushort");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT16));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_longlong");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT64));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_ulonglong");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT64));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_float");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT32));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_double");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT64));
struct_builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_longdouble");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT128));
struct_builder->add_member(member_descriptor);

// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for a member of type int32_t
int32_t in_value {2};
int32_t out_value {0};
data->set_int32_value(data->get_member_id_by_name("my_long"), in_value);
data->get_int32_value(out_value, data->get_member_id_by_name("my_long"));
```

For a detailed explanation about the XML definition of this type,
please refer to [XML Primitive Types](../xml_configuration/dynamic_types.md#xmldynamictypes-primivites).

<a id="type-promotions"></a>

#### Type promotions

The Dynamic Language Binding also supports type promotion, enabling implicit promotion of types during both `get()`
and `set()` operations.
This means that a smaller type can be implicitly promoted to a larger type, but not the other way around.

The following promotions are supported:

| TypeKind      | Allowed promotions                                                                                                                              |
|---------------|-------------------------------------------------------------------------------------------------------------------------------------------------|
| `TK_INT8`     | `TK_INT16`, `TK_INT32`, `TK_INT64`, `TK_FLOAT32`, `TK_FLOAT64`, `TK_FLOAT128`                                                                   |
| `TK_INT16`    | `TK_INT32`, `TK_INT64`, `TK_FLOAT32`, `TK_FLOAT64`, `TK_FLOAT128`                                                                               |
| `TK_INT32`    | `TK_INT64`, `TK_FLOAT64`, `TK_FLOAT128`                                                                                                         |
| `TK_INT64`    | `TK_FLOAT128`                                                                                                                                   |
| `TK_UINT8`    | `TK_INT16`, `TK_INT32`, `TK_INT64`, `TK_UINT16`, `TK_UINT32`, `TK_UINT64`,<br/>`TK_FLOAT32`, `TK_FLOAT64`, `TK_FLOAT128`                        |
| `TK_UINT16`   | `TK_INT32`, `TK_INT64`, `TK_UINT32`, `TK_UINT64`, `TK_FLOAT32`, `TK_FLOAT64`,<br/>`TK_FLOAT128`                                                 |
| `TK_UINT32`   | `TK_INT64`, `TK_UINT64`, `TK_FLOAT64`, `TK_FLOAT128`                                                                                            |
| `TK_UINT64`   | `TK_FLOAT128`                                                                                                                                   |
| `TK_FLOAT32`  | `TK_FLOAT64`, `TK_FLOAT128`                                                                                                                     |
| `TK_FLOAT64`  | `TK_FLOAT128`                                                                                                                                   |
| `TK_FLOAT128` | (none)                                                                                                                                          |
| `TK_CHAR8`    | `TK_CHAR16`, `TK_INT16`, `TK_INT32`, `TK_INT64`, `TK_FLOAT32`, `TK_FLOAT64`,<br/>`TK_FLOAT128`                                                  |
| `TK_CHAR16`   | `TK_INT32`, `TK_INT64`, `TK_FLOAT32`, `TK_FLOAT64`, `TK_FLOAT128`                                                                               |
| `TK_BYTE`     | (any)                                                                                                                                           |
| `TK_BOOLEAN`  | `TK_INT8`, `TK_INT16`, `TK_INT32`, `TK_INT64`, `TK_UINT8`, `TK_UINT16`,<br/>`TK_UINT32`, `TK_UINT64`, `TK_FLOAT32`, `TK_FLOAT64`, `TK_FLOAT128` |

<a id="xtypes-supportedtypes-string"></a>

<a id="string-types"></a>

### String Types

String types are one-dimensional collections of characters (`TK_CHAR8` or `TK_CHAR16`.
The latest are also known as wide strings or `wstring`).
The `TypeKinds` used to identify string types are `TK_STRING8` and `TK_STRING16`.
The string may be bounded, setting a maximum length, or unbounded.
This is configured using [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `bound` property.

[`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) exposes the functions [`DynamicTypeBuilderFactory::create_string_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory18create_string_typeENSt8uint32_tE) and
[`DynamicTypeBuilderFactory::create_wstring_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory19create_wstring_typeENSt8uint32_tE) that eases string creation providing the corresponding maximum length
parameter (`LENGTH_UNLIMITED` is used for unbounded strings).

[`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) class provides also with specific getters and setters: [`DynamicData::get_string_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData16get_string_valueERNSt6stringE8MemberId),
[`DynamicData::get_wstring_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17get_wstring_valueERNSt7wstringE8MemberId), [`DynamicData::set_string_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData16set_string_valueE8MemberIdRKNSt6stringE), and [`DynamicData::set_wstring_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17set_wstring_valueE8MemberIdRKNSt7wstringE).

### OMG-IDL

```omg-idl
struct StringsStruct
{
    string my_string;
    wstring my_wstring;
    string<41925> my_bounded_string;
    wstring<20925> my_bounded_wstring;
};
```

### XML

```xml
<struct name="StringsStruct">
    <member name="my_string" type="string"/>
    <member name="my_wstring" type="wstring"/>
    <member name="my_bounded_string" type="string" stringMaxLength="41925"/>
    <member name="my_bounded_wstring" type="wstring" stringMaxLength="20925"/>
</struct>
```

### C++

```c++
// Define a struct type to contain the strings
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("StringsStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};

// Define and add string members to the struct
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};

member_descriptor->name("my_string");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                create_string_type(static_cast<uint32_t>(LENGTH_UNLIMITED))->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_STRING8);
    type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR8));
    type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
    member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
 */

struct_builder->add_member(member_descriptor);
member_descriptor->name("my_wstring");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                create_wstring_type(static_cast<uint32_t>(LENGTH_UNLIMITED))->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_STRING16);
    type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR16));
    type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
    member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
 */

struct_builder->add_member(member_descriptor);
member_descriptor->name("my_bounded_string");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                create_string_type(41925)->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_STRING8);
    type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR8));
    type_descriptor->bound().push_back(41925);
    member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
 */

struct_builder->add_member(member_descriptor);
member_descriptor->name("my_bounded_wstring");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                create_wstring_type(20925)->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_STRING16);
    type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_CHAR16));
    type_descriptor->bound().push_back(20925);
    member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)->build());
 */

struct_builder->add_member(member_descriptor);

// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for a string member
std::string in_value {"helloworld"};
std::string out_value;
data->set_string_value(data->get_member_id_by_name("my_string"), in_value);
data->get_string_value(out_value, data->get_member_id_by_name("my_string"));
```

For a detailed explanation about the XML definition of this type, please refer to
[XML String Types](../xml_configuration/dynamic_types.md#xmldynamictypes-strings).

<a id="xtypes-supportedtypes-enumeration"></a>

<a id="enumeration-types"></a>

### Enumeration Types

An enumeration contains a set of supported values (enumeration literals) and a selected value among those supported.
The `TypeKind` used to identify enumeration types is `TK_ENUM`.

The enumeration literals must be configured using the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) by calling the
[`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE) function for the respective supported values.
The [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) passed to the previous function must determine the enumeration literal name by using
`name` property.

The underlying primitive type related to the enumeration is configured using [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `type`
property.
As the primitive type is determined when adding the first enumeration literal,
the remaining enumeration literals must be of the same primitive type to keep the enumeration type consistent.
The underlying primitive type can also be forced by setting the [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `literal_type` property.
If `literal_type` is set and the first enumeration literal is not of the same type,
[`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE) will fail when trying to add it.

Additionally, the enumeration literal value might be set using [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `literal_value` property.
The behavior is the same as setting the `@value` [builtin annotation](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations).
The default literal of the enumeration type can be modified by setting the [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE)
`is_default_literal` property, which has the same behavior as annotating the member with
`@default_literal` [builtin annotation](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations).

As the enumeration type is basically a signed integer type which might take only some specific values defined with the
enumeration literals, the corresponding DynamicData getters and setters are the ones corresponding to the underlying
signed integer type (and any other method promotable to that specific primitive type).

### OMG-IDL

```omg-idl
enum MyEnum
{
    A,
    B,
    C
};

struct EnumStruct
{
    MyEnum my_enum;
};
```

### XML

```xml
<enum name="MyEnum">
    <enumerator name="A" value="0"/>
    <enumerator name="B" value="1"/>
    <enumerator name="C"/>
</enum>

<struct name="EnumStruct">
    <member name="my_enum" type="nonBasic" nonBasicTypeName="MyEnum"/>
</struct>
```

### C++

```c++
enum MyEnum : int32_t
{
    A,
    B,
    C
};

// Define a struct type to contain an enum
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("EnumStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};
// Define the enum type
TypeDescriptor::_ref_type enum_type_descriptor {traits<TypeDescriptor>::make_shared()};
enum_type_descriptor->kind(TK_ENUM);
enum_type_descriptor->name("MyEnum");
DynamicTypeBuilder::_ref_type enum_builder {DynamicTypeBuilderFactory::get_instance()->
                                                    create_type(enum_type_descriptor)};
// Add enum literals to the enum type
MemberDescriptor::_ref_type enum_member_descriptor {traits<MemberDescriptor>::make_shared()};
enum_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
enum_member_descriptor->name("A");
enum_builder->add_member(enum_member_descriptor);
enum_member_descriptor = traits<MemberDescriptor>::make_shared();
enum_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
enum_member_descriptor->name("B");
enum_builder->add_member(enum_member_descriptor);
enum_member_descriptor = traits<MemberDescriptor>::make_shared();
enum_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
enum_member_descriptor->name("C");
enum_builder->add_member(enum_member_descriptor);
// Build the enum type
DynamicType::_ref_type enum_type = enum_builder->build();

// Add an enum member to the struct
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->name("my_enum");
member_descriptor->type(enum_type);
struct_builder->add_member(member_descriptor);
// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for an enum member
MyEnum in_value {MyEnum::C};

/* Alternative
    uint32_t in_value {2}; // Selecting MyEnum::C
 */

uint32_t out_value {0};
data->set_uint32_value(data->get_member_id_by_name("my_enum"), in_value);
data->get_uint32_value(out_value, data->get_member_id_by_name("my_enum"));
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Enumeration Types](../xml_configuration/dynamic_types.md#xmldynamictypes-enums).

<a id="xtypes-supportedtypes-alias"></a>

<a id="alias-types"></a>

### Alias Types

Alias types provide an alternative name to an already existing type.
The `TypeKind` used to identify aliases is `TK_ALIAS`.
Besides defining the alias name, the underlying type must be set using [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `base_type` property.
Alias recursion is supported by defining another alias type as the base type.

Once the [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) is created, information can be accessed as if working with the base type.

### OMG-IDL

```omg-idl
typedef MyEnum MyAliasedEnum;
typedef string<100> MyAliasedBoundedString;
typedef MyAliasedEnum MyRecursiveAlias;

struct AliasStruct
{
    MyAliasedEnum my_aliased_enum;
    MyAliasedBoundedString my_aliased_bounded_string;
    MyRecursiveAlias my_recursive_alias;
};
```

### XML

```xml
<typedef name="MyAliasedEnum" type="nonBasic" nonBasicTypeName="MyEnum"/>
<!-- XSD does not allow to set bounds to aliased strings -->
<typedef name="MyAliasedBoundedString" type="string"/>
<typedef name="MyRecursiveAlias" type="nonBasic" nonBasicTypeName="MyAliasedEnum"/>

<struct name="AliasStruct">
    <member name="my_aliased_enum" type="nonBasic" nonBasicTypeName="MyAliasedEnum"/>
    <member name="my_aliased_bounded_string" type="nonBasic" nonBasicTypeName="MyAliasedBoundedString"/>
    <member name="my_recursive_alias" type="nonBasic" nonBasicTypeName="MyRecursiveAlias"/>
</struct>
```

### C++

```c++
// Define a struct type to contain the alias
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("AliasStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};

// Define an alias type for the enum
TypeDescriptor::_ref_type aliasenum_type_descriptor {traits<TypeDescriptor>::make_shared()};
aliasenum_type_descriptor->kind(TK_ALIAS);
aliasenum_type_descriptor->name("MyAliasedEnum");
aliasenum_type_descriptor->base_type(enum_type);
DynamicTypeBuilder::_ref_type aliasenum_builder {DynamicTypeBuilderFactory::get_instance()->
                                                         create_type(aliasenum_type_descriptor)};
// Build the alias type
DynamicType::_ref_type aliasenum_type = aliasenum_builder->build();

// Define an alias type for a bounded string
TypeDescriptor::_ref_type alias_bounded_string_type_descriptor {traits<TypeDescriptor>::make_shared()};
alias_bounded_string_type_descriptor->kind(TK_ALIAS);
alias_bounded_string_type_descriptor->name("MyAliasedBoundedString");
alias_bounded_string_type_descriptor->base_type(DynamicTypeBuilderFactory::get_instance()->
                create_string_type(100)->build());
DynamicTypeBuilder::_ref_type alias_bounded_string_builder {DynamicTypeBuilderFactory::get_instance()->
                                                                    create_type(
                                                                alias_bounded_string_type_descriptor)};
// Build the alias type for the bounded string
DynamicType::_ref_type alias_bounded_string_type = alias_bounded_string_builder->build();

// Define a recursive alias
TypeDescriptor::_ref_type recursive_alias_type_descriptor {traits<TypeDescriptor>::make_shared()};
recursive_alias_type_descriptor->kind(TK_ALIAS);
recursive_alias_type_descriptor->name("MyRecursiveAlias");
recursive_alias_type_descriptor->base_type(aliasenum_type);
DynamicTypeBuilder::_ref_type recursive_alias_builder {DynamicTypeBuilderFactory::get_instance()->
                                                               create_type(recursive_alias_type_descriptor)};
// Build the recursive alias type
DynamicType::_ref_type recursive_alias_type = recursive_alias_builder->build();

// Add alias enum member to the structure
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->name("my_aliased_enum");
member_descriptor->type(aliasenum_type);
struct_builder->add_member(member_descriptor);
// Add alias bounded string member to the structure
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_aliased_bounded_string");
member_descriptor->type(alias_bounded_string_type);
struct_builder->add_member(member_descriptor);
// Add recursive alias member to the structure
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->name("my_recursive_alias");
member_descriptor->type(recursive_alias_type);
struct_builder->add_member(member_descriptor);

// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for the alias enum member
MyEnum in_value {MyEnum::C};
int32_t out_value {0};
data->set_int32_value(data->get_member_id_by_name("my_alias_enum"), in_value);
data->get_int32_value(out_value, data->get_member_id_by_name("my_alias_enum"));
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Alias Types](../xml_configuration/dynamic_types.md#xmldynamictypes-typedef).

<a id="xtypes-supportedtypes-sequence"></a>

<a id="sequence-types"></a>

### Sequence types

Sequence types are one-dimensional collections of any type.
The `TypeKind` used to identify sequences is `TK_SEQUENCE`.
[`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `element_type` property must be set with the collection’s type.
Additionally, `bound` property must also be configured with the sequence’s maximum length, or
`LENGTH_UNLIMITED` in case of unbounded sequences.

[`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) exposes the function [`DynamicTypeBuilderFactory::create_sequence_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory20create_sequence_typeEN6traitsI11DynamicTypeE8ref_typeENSt8uint32_tE) to
facilitate the creation of this type.
This API requires the type stored in the collection and the collection’s bound, using `LENGTH_UNLIMITED` in case
of unbounded sequences.

[`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) class provides specific `get_values()` and `set_values()` functions for each primitive
type, allowing users to easily work with sequences of primitive types.
Primitive type promotion is also applicable for these methods.
For sequences of more complex types, please refer to [Managing Complex Types Data](#xtypes-complextypes).

If a specific range of values within the sequence are to be modified, passing the starting index to
`get_values()` / `set_values()` would only manage data from that element forward until the length of the given
input.
Specific collection’s element can be also be modified using the `get_value()` / `set_value()` passing the index
of the element to be modified.

### OMG-IDL

```omg-idl
struct SequenceStruct
{
    sequence<MyBitMask> bitmask_sequence;
    sequence<short, 5> short_sequence;
};
```

### XML

```xml
<struct name="SequenceStruct">
    <member name="bitmask_sequence" type="nonBasic" nonBasicTypeName="MyBitMask" sequenceMaxLength="-1"/>
    <member name="short_sequence" sequenceMaxLength="5" type="int16"/>
</struct>
```

### C++

```c++
// Define a struct type to contain the sequence
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("SequenceStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};

// Define a member for the sequence
MemberDescriptor::_ref_type sequence_member_descriptor {traits<MemberDescriptor>::make_shared()};
sequence_member_descriptor->name("bitmask_sequence");
sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_sequence_type(bitmask_type,
        static_cast<uint32_t>(LENGTH_UNLIMITED))->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_SEQUENCE);
    type_descriptor->element_type(bitmask_type);
    type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
    sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
            type_descriptor)->build());
 */

// Add the sequence member to the struct
struct_builder->add_member(sequence_member_descriptor);

sequence_member_descriptor = traits<MemberDescriptor>::make_shared();
sequence_member_descriptor->name("short_sequence");
sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_sequence_type(
            DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16), 5)->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_SEQUENCE);
    type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
    type_descriptor->bound().push_back(5);
    sequence_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
            type_descriptor)->build());
 */

// Add the sequence member to the struct
struct_builder->add_member(sequence_member_descriptor);
// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for the sequence member
Int16Seq in_value = {1, 2};
Int16Seq out_value;
data->set_int16_values(data->get_member_id_by_name("short_sequence"), in_value);
data->get_int16_values(out_value, data->get_member_id_by_name("short_sequence"));

DynamicData::_ref_type sequence_data {data->loan_value(data->get_member_id_by_name("short_sequence"))};
// Set the two latest possible values on the sequence
sequence_data->set_int16_values(3, in_value);
// Read every sequence value from index 1 to the end
sequence_data->get_int16_values(out_value, 1);

int16_t in_simple_value = 8;
int16_t out_simple_value;
sequence_data->set_int16_value(2, in_simple_value);
sequence_data->get_int16_value(out_simple_value, 2);

data->return_loaned_value(sequence_data);
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Sequence Types](../xml_configuration/dynamic_types.md#xmldynamictypes-sequence).

<a id="xtypes-supportedtypes-array"></a>

<a id="array-types"></a>

### Array types

Array types are multi-dimensional collections of any type.
The `TypeKind` used to identify arrays is `TK_ARRAY`.
[`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `element_type` property must be set with the collection’s type.
Additionally, `bound` property must be configured with the sequence containing the size of each dimension.
Bound sequence must have at least one dimension and it is not allowed for any dimension to have size `0`.

[`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) exposes the function [`DynamicTypeBuilderFactory::create_array_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory17create_array_typeEN6traitsI11DynamicTypeE8ref_typeERK8BoundSeq) to
facilitate the creation of this type.
This API requires the type stored in the collection and the sequence with the collection’s dimensions.

[`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) class provides specific `get_values()` and `set_values()` functions for each primitive
type, allowing users to easily work with arrays of primitives types.
For arrays of more complex types, please refer to [Managing Complex Types Data](#xtypes-complextypes).

#### NOTE
Multi-dimensional arrays *flatten* every dimension into a single-dimension array.

Primitive type promotion is also applicable for these methods.

If a specific range of values within the array are to be modified, passing the starting index to
`get_values()` / `set_values()` would only manage data from that element forward until the length of the given
input.
Specific collection’s element can be also be modified using the `get_value()` / `set_value()` passing the index
of the element to be modified.

### OMG-IDL

```omg-idl
struct ArrayStruct
{
    long long_array[2][3][4];
};
```

### XML

```xml
<struct name="ArrayStruct">
    <member name="long_array" type="int32" arrayDimensions="2,3,4"/>
</struct>
```

### C++

```c++
// Define a struct type to contain the array
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("ArrayStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};
// Define a member for the array
MemberDescriptor::_ref_type array_member_descriptor {traits<MemberDescriptor>::make_shared()};
array_member_descriptor->name("long_array");
array_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_array_type(
            DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32), { 2, 3, 4 })->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_ARRAY);
    type_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
    type_descriptor->bound().push_back(2);
    type_descriptor->bound().push_back(3);
    type_descriptor->bound().push_back(4);
    array_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
            type_descriptor)->build());
 */

// Add the array member to the struct
struct_builder->add_member(array_member_descriptor);
// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for the array member
Int32Seq in_value = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 23, 23, 24};
Int32Seq out_value;
data->set_int32_values(data->get_member_id_by_name("long_array"), in_value);
data->get_int32_values(out_value, data->get_member_id_by_name("long_array"));

DynamicData::_ref_type array_data {data->loan_value(data->get_member_id_by_name("long_array"))};
// Set the two latest possible values on the array
Int32Seq small_in_value = {0, 1};
array_data->set_int32_values(22, small_in_value);
// Read every array value from index 1 to the end
array_data->get_int32_values(out_value, 1);

int32_t in_simple_value = 8;
int32_t out_simple_value;
array_data->set_int32_value(2, in_simple_value);
array_data->get_int32_value(out_simple_value, 2);

data->return_loaned_value(array_data);
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Array Types](../xml_configuration/dynamic_types.md#xmldynamictypes-array).

<a id="xtypes-supportedtypes-map"></a>

<a id="map-types"></a>

### Map Types

Map types are a collection of key/value pair types.
Access to the value element is done through the key which is unique within the map type.
The `TypeKind` used to identify maps is `TK_MAP`.
[`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `element_type` property must be set with the map value type.
[`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `key_type` property must be set with the map key type.
Allowed key types are signed and unsigned integer types and string types.

#### NOTE
Currently, wide string keys are not supported as map keys.

Additionally, `bound` property must also be configured with the map’s maximum length, or `LENGTH_UNLIMITED`
in case of unbounded maps.

[`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) exposes the [`DynamicTypeBuilderFactory::create_map_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory15create_map_typeEN6traitsI11DynamicTypeE8ref_typeEN6traitsI11DynamicTypeE8ref_typeENSt8uint32_tE) function to
facilitate the creation of this type.
This API requires the type of both the key and the value stored in the collection, and the collection’s bound, using
`LENGTH_UNLIMITED` in case of unbounded maps.

Manipulating map types data is more complex.
First the `MemberId` corresponding to a specific `key` must be retrieved using
[`DynamicData::get_member_id_by_name`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData21get_member_id_by_nameERK10ObjectName) API.
This API either returns the `MemberId` corresponding to the existing `key` or, if the `key` does not
exist yet, it creates the `key` and returns the `memberId` associated to the just created `key`.
In order to call this method, the correct `string` representation of the key value must be passed.
The map value can now be set using the API corresponding to the map value type.
For complex map values, please refer to [Managing Complex Types Data](#xtypes-complextypes).

### OMG-IDL

```omg-idl
struct MapStruct
{
    map<string, MyAliasedBoundedString> string_alias_unbounded_map;
    map<short, long, 2> short_long_map;
};
```

### XML

```xml
<struct name="MapStruct">
    <member name="string_alias_unbounded_map" type="nonBasic" nonBasicTypeName="MyAliasedBoundedString" key_type="string" mapMaxLength="-1"/>
    <member name="short_long_map" type="int32" key_type="int16" mapMaxLength="2"/>
</struct>
```

### C++

```c++
// Define a struct type to contain the map
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("MapStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};
// Define a member for the map
MemberDescriptor::_ref_type map_member_descriptor {traits<MemberDescriptor>::make_shared()};
map_member_descriptor->name("string_long_array_unbounded_map");
map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_map_type(
            DynamicTypeBuilderFactory::get_instance()->create_string_type(static_cast<uint32_t>(
                LENGTH_UNLIMITED))->build(), alias_bounded_string_type, static_cast<uint32_t>(
                LENGTH_UNLIMITED))->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_MAP);
    type_descriptor->key_element_type(DynamicTypeBuilderFactory::get_instance()->create_string_type(
            static_cast<uint32_t>(LENGTH_UNLIMITED)->build());
    type_descriptor->element_type(alias_bounded_string_type);
    type_descriptor->bound().push_back(static_cast<uint32_t>(LENGTH_UNLIMITED));
    map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
            type_descriptor)->build());
 */

// Add the map member to the struct
struct_builder->add_member(map_member_descriptor);

map_member_descriptor = traits<MemberDescriptor>::make_shared();
map_member_descriptor->name("short_long_map");
map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_map_type(
            DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16),
            DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32), 2)->build());

/* Alternative
    type_descriptor = traits<TypeDescriptor>::make_shared();
    type_descriptor->kind(TK_MAP);
    type_descriptor->key_element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
    type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
    type_descriptor->bound().push_back(2);
    map_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_type(
            type_descriptor)->build());
 */

struct_builder->add_member(map_member_descriptor);
// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Get the data loan of the map member
DynamicData::_ref_type map_data = data->loan_value(data->get_member_id_by_name("short_long_map"));

// Set and retrieve values for the map member
int32_t key {1};
int32_t in_value {2};
int32_t out_value;
map_data->set_int32_value(map_data->get_member_id_by_name(std::to_string(key)), in_value);
map_data->get_int32_value(out_value, map_data->get_member_id_by_name(std::to_string(key)));

// Return de data loan of the map member
data->return_loaned_value(map_data);
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Map Types](../xml_configuration/dynamic_types.md#xmldynamictypes-map).

<a id="xtypes-supportedtypes-structure"></a>

<a id="structure-types"></a>

### Structure Types

Structure types are an aggregation of members of different types.
The `TypeKind` used to identify structures is `TK_STRUCTURE`.
Structure types have single inheritance support, so a structure type might extend one other already defined structure.
The structure type which is extended should be configured in the [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `base_type` property.
[Structure extensibility](../../fastddsgen/dataTypes/dataTypes.md#extensibility) might be configured using [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `extensibility_kind`
property.

#### NOTE
Currently, `@nested` builtin annotation is not supported.

Structure members must be configured using the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) by calling the [`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE)
function with the corresponding [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE).

#### NOTE
Empty structures, with no members, are allowed.

Member name is configured using [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `name` property and the member type is set using
`type` property.
Structure members might be keyed to create [topic instances](../dds_layer/topic/instances.md#dds-layer-topic-instances) by setting the
[`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `is_key` property.
The behavior is the same as setting the `@key` [builtin annotation](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations).
Additionally, [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `default_value` property might be set to configure the member default value,
and [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `id` property sets explicitly the member ID.
This behavior is the same as setting the `@default` and `@id`
[builtin annotations](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations).

#### NOTE
Currently, Fast DDS-Gen does not support `@default` builtin annotation.

#### NOTE
Currently, Dynamic Language Binding API implementation does not support the following builtin annotations:

* `@must_understand`
* `@external`
* `@try_construct`

Member data can be managed using the corresponding accessors for the underlying member type.
Member ID might be retrieved using [`DynamicData::get_member_id_by_name`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData21get_member_id_by_nameERK10ObjectName) API.
For managing complex type members, please refer to [Managing Complex Types Data](#xtypes-complextypes).

### OMG-IDL

```omg-idl
struct InnerStruct
{
    @id(0x10) long first;
};

struct ParentStruct
{
    float first;
    long long second;
};

struct ComplexStruct : ParentStruct
{
    InnerStruct complex_member;
};
```

### XML

```xml
<struct name="InnerStruct">
    <!-- XML does not support setting Member ID -->
    <member name="first" type="int32"/>
</struct>

```

### C++

```c++
// Create inner struct type
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(eprosima::fastdds::dds::TK_STRUCTURE);
type_descriptor->name("InnerStruct");
DynamicTypeBuilder::_ref_type builder {DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor)};
// Add members to the inner struct type
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                get_primitive_type(eprosima::fastdds::dds::TK_INT32));
member_descriptor->name("first");
member_descriptor->id(16);
builder->add_member(member_descriptor);
// Build the inner struct type
DynamicType::_ref_type inner_struct_type {builder->build()};

// Create parent struct type
TypeDescriptor::_ref_type parentstruct_type_descriptor {traits<TypeDescriptor>::make_shared()};
parentstruct_type_descriptor->kind(TK_STRUCTURE);
parentstruct_type_descriptor->name("ParentStruct");
DynamicTypeBuilder::_ref_type parentstruct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                            create_type(parentstruct_type_descriptor)};
// Add members to the parent struct type
MemberDescriptor::_ref_type parentstruct_member {traits<MemberDescriptor>::make_shared()};
parentstruct_member->name("first");
parentstruct_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT32));
parentstruct_builder->add_member(parentstruct_member);
parentstruct_member = traits<MemberDescriptor>::make_shared();
parentstruct_member->name("second");
parentstruct_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT64));
parentstruct_builder->add_member(parentstruct_member);
// Build the parent struct type
DynamicType::_ref_type parentstruct_type = parentstruct_builder->build();

// Create complex struct type
TypeDescriptor::_ref_type complexstruct_type_descriptor {traits<TypeDescriptor>::make_shared()};
complexstruct_type_descriptor->kind(TK_STRUCTURE);
complexstruct_type_descriptor->name("ComplexStruct");
complexstruct_type_descriptor->base_type(parentstruct_type);
DynamicTypeBuilder::_ref_type complexstruct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                             create_type(complexstruct_type_descriptor)};
// Add members to the complex struct type
MemberDescriptor::_ref_type complexstruct_member {traits<MemberDescriptor>::make_shared()};
complexstruct_member->name("complex_member");
complexstruct_member->type(inner_struct_type);
complexstruct_builder->add_member(complexstruct_member);

// Build the complex struct type
DynamicType::_ref_type complexstruct_type = complexstruct_builder->build();
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(complexstruct_type)};

// Set and retrieve values for member of type float
float in_value {3.14};
float out_value {0.0};
data->set_float32_value(data->get_member_id_by_name("first"), in_value);
data->get_float32_value(out_value, data->get_member_id_by_name("first"));
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Structure Types](../xml_configuration/dynamic_types.md#xmldynamictypes-struct).

<a id="xtypes-supportedtypes-inheritance"></a>

<a id="inheritance"></a>

#### Inheritance

A structure type can also extend another structure type, inheriting all its members.
The example below shows `DerivedStruct` extending `BaseStruct`: `second` is inherited from
`BaseStruct`, while `third` is declared by `DerivedStruct` itself.

### OMG-IDL

```omg-idl
struct BaseStruct
{
    float first;
    long long second;
};

struct DerivedStruct : BaseStruct
{
    long third;
};
```

### XML

#### WARNING
Currently, structure inheritance is not supported when loading dynamic types from XML, so there is
no XML example for this section.

### C++

```c++
// Create the base struct type
TypeDescriptor::_ref_type base_type_descriptor {traits<TypeDescriptor>::make_shared()};
base_type_descriptor->kind(TK_STRUCTURE);
base_type_descriptor->name("BaseStruct");
DynamicTypeBuilder::_ref_type base_builder {DynamicTypeBuilderFactory::get_instance()->
                                                    create_type(base_type_descriptor)};
// Add members to the base struct type
MemberDescriptor::_ref_type base_member {traits<MemberDescriptor>::make_shared()};
base_member->name("first");
base_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_FLOAT32));
base_builder->add_member(base_member);
base_member = traits<MemberDescriptor>::make_shared();
base_member->name("second");
base_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT64));
base_builder->add_member(base_member);
DynamicType::_ref_type base_type {base_builder->build()};

// Create the derived struct type, extending the base struct type
TypeDescriptor::_ref_type derived_type_descriptor {traits<TypeDescriptor>::make_shared()};
derived_type_descriptor->kind(TK_STRUCTURE);
derived_type_descriptor->name("DerivedStruct");
derived_type_descriptor->base_type(base_type);
DynamicTypeBuilder::_ref_type derived_builder {DynamicTypeBuilderFactory::get_instance()->
                                                       create_type(derived_type_descriptor)};
// Add its own member; the base struct's members are inherited automatically
MemberDescriptor::_ref_type derived_member {traits<MemberDescriptor>::make_shared()};
derived_member->name("third");
derived_member->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT32));
derived_builder->add_member(derived_member);
DynamicType::_ref_type derived_type {derived_builder->build()};

// Create dynamic data based on the derived struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(derived_type)};

// Inherited and own members are both accessible through the derived type
data->set_int64_value(data->get_member_id_by_name("second"), 42);
data->set_int32_value(data->get_member_id_by_name("third"), 7);
```

<a id="xtypes-supportedtypes-union"></a>

<a id="union-types"></a>

### Union Types

Union types are a special type of structure type where only one member exists.
The `TypeKind` used to identify unions is `TK_UNION`.
Member selection is performed by setting another special member called discriminator.
The discriminator type must be defined using [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `discriminator_type` property.
Supported discriminator `TypeKind` are the following:

* `TK_BOOLEAN`
* `TK_BYTE`
* `TK_CHAR8`
* `TK_CHAR16`
* `TK_INT8`
* `TK_UINT8`
* `TK_INT16`
* `TK_UINT16`
* `TK_INT32`
* `TK_UINT32`
* `TK_INT64`
* `TK_UINT64`
* `TK_ENUM`
* `TK_ALIAS` that resolves, directly or indirectly to one of the aforementioned types.

[Union extensibility](../../fastddsgen/dataTypes/dataTypes.md#extensibility) might be configured using [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `extensibility_kind`
property.

#### NOTE
Currently, `@nested` builtin annotation is not supported.

Union members must be configured using the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) by calling the [`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE)
function with the corresponding [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE).
At least one union member must be added to the union type.
Union member name is configured using [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `name` property and the member type is set using
`type` property.
It is also mandatory to either set [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `is_default_label` property or configure the
`label` property.
This latest property indicates the discriminator values that select this specific member.
If no labels are configured, then the flag indicating the member to be the default one, must be set.
Only one union member must be configured as default.

Additionally, [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `default_value` property might be set to configure the member default value,
and [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `id` property sets explicitly the member ID.
This behavior is the same as setting the `@default` and `@id`
[builtin annotations](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations).

#### NOTE
Currently, Fast DDS-Gen does not support `@default` builtin annotation.

#### NOTE
Currently, Dynamic Language Binding API implementation does not support the following builtin annotations:

* `@must_understand`
* `@external`
* `@try_construct`

Member data can be managed using the corresponding accessors for the underlying member type.
Setting a member automatically changes the discriminator value selecting the set member.
When reading a member, the discriminator must be selecting the member being read.
Member ID might be retrieved using [`DynamicData::get_member_id_by_name`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData21get_member_id_by_nameERK10ObjectName) API.
For managing complex type members, please refer to [Managing Complex Types Data](#xtypes-complextypes).

### OMG-IDL

```omg-idl
union InnerUnion switch (short)
{
    case 0:
        @id(0x10) PrimitivesStruct first;
    case 1:
    default:
        long long second;
};

union ComplexUnion switch (long)
{
    case 0:
    case 1:
        long third;
    default:
        InnerUnion fourth;
};
```

### XML

```xml
<union name="InnerUnion">
    <discriminator type="int16"/>
    <case>
        <caseDiscriminator value="0"/>
        <member name="first" type="nonBasic" nonBasicTypeName="PrimitivesStruct"/>
    </case>
    <case>
        <caseDiscriminator value="1"/>
        <caseDiscriminator value="default"/>
        <member name="second" type="int64"/>
    </case>
</union>
<union name="ComplexUnion">
    <discriminator type="int32"/>
    <case>
        <caseDiscriminator value="0"/>
        <caseDiscriminator value="1"/>
        <member name="third" type="int32"/>
    </case>
    <case>
        <caseDiscriminator value="default"/>
        <member name="fourth" type="nonBasic" nonBasicTypeName="InnerUnion"/>
    </case>
</union>
```

### C++

```c++
// Define a struct type to contain the union
// Create the inner union
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_UNION);
type_descriptor->name("InnerUnion");
type_descriptor->discriminator_type(DynamicTypeBuilderFactory::get_instance()->
                get_primitive_type(TK_INT16));
DynamicTypeBuilder::_ref_type builder {DynamicTypeBuilderFactory::get_instance()->
                                               create_type(type_descriptor)};
// Add members to the inner union type
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->type(struct_type);
member_descriptor->name("first");
member_descriptor->id(16);
member_descriptor->label({0});
builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                get_primitive_type(TK_INT64));
member_descriptor->name("second");
member_descriptor->label({1});
member_descriptor->is_default_label(true);
builder->add_member(member_descriptor);
// Build the inner union type
DynamicType::_ref_type inner_union_type {builder->build()};

// Create a complex union type
type_descriptor = traits<TypeDescriptor>::make_shared();
type_descriptor->kind(TK_UNION);
type_descriptor->name("ComplexUnion");
type_descriptor->discriminator_type(DynamicTypeBuilderFactory::get_instance()->
                get_primitive_type(TK_INT32));
builder = DynamicTypeBuilderFactory::get_instance()->create_type(type_descriptor);
// Add members to the complex union type
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->
                get_primitive_type(TK_INT32));
member_descriptor->name("third");
member_descriptor->label({0, 1});
builder->add_member(member_descriptor);
member_descriptor = traits<MemberDescriptor>::make_shared();
member_descriptor->type(inner_union_type);
member_descriptor->name("fourth");
member_descriptor->is_default_label(true);
builder->add_member(member_descriptor);
// Build the complex union type
DynamicType::_ref_type union_type {builder->build()};

// Create dynamic data based on the union type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(union_type)};
// Get the loan for the InnerUnion member
DynamicData::_ref_type union_data = data->loan_value(data->get_member_id_by_name("InnerUnion"));

// Set and retrieve values for the long long member within InnerUnion member
int64_t in_value {2};
int64_t out_value;
union_data->set_int64_value(union_data->get_member_id_by_name("second"), in_value);
union_data->get_int64_value(out_value, union_data->get_member_id_by_name("second"));
// Return de data loan of the member
data->return_loaned_value(union_data);
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Union Types](../xml_configuration/dynamic_types.md#xmldynamictypes-union).

<a id="xtypes-supportedtypes-bitmask"></a>

<a id="bitmask-types"></a>

### Bitmask Types

Bitmask types are a collection of boolean flags (bitflags) that can be set individually.
The `TypeKind` used to identify bitmask types is `TK_BITMASK`.
The bitmasks bound, maximum number of bits, must be set using the [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `bound` property.
The maximum bound allowed is 64 bits.

The bitflags must be configured using the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) by calling the [`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE)
function.
Each bitflag is described using a [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) defining the bitflag name using the `name` property.
The underlying primitive type related to bitflags must be of kind `TK_BOOLEAN` and must be set in
[`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `type` property.
The [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `position` property might be used to indicate the bitflag position within the bitmask.
This behavior is the same as setting the `@position` [builtin annotation](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations).
If the position is not specified, sequential order is followed.

The [`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) exposes the function [`DynamicTypeBuilderFactory::create_bitmask_type`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory19create_bitmask_typeENSt8uint32_tE) to
facilitate the creation of bitmask types.

Bitmask types can be manipulated either using the [`DynamicData::get_boolean_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17get_boolean_valueERb8MemberId)/[`DynamicData::set_boolean_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17set_boolean_valueE8MemberIdb) in
order to set a specific bitflag, or by using the unsigned integer setter/getter corresponding to the bitmask bound.
In this latest case, only bitflags are going to be set (bits not named are always unset).

### OMG-IDL

```omg-idl
@bit_bound(8)
bitmask MyBitMask
{
    @position(0) flag0,
    flag1,
    flag2,
    @position(5) flag5
};

struct BitmaskStruct
{
    MyBitMask my_bitmask;
};
```

### XML

```xml
<bitmask name="MyBitMask" bit_bound="8">
    <bit_value name="flag0" position="0"/>
    <bit_value name="flag1"/>
    <bit_value name="flag2"/>
    <bit_value name="flag5" position="5"/>
</bitmask>

<struct name="BitmaskStruct">
    <member name="my_bitmask" type="nonBasic" nonBasicTypeName="MyBitMask"/>
</struct>
```

### C++

```c++
// Define a struct type to contain a bitmask
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("BitmaskStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->
                                                      create_type(type_descriptor)};

// Define the bitmask type
DynamicTypeBuilder::_ref_type bitmask_builder {DynamicTypeBuilderFactory::get_instance()->create_bitmask_type(
                                                   8)};

/* Alternative
    TypeDescriptor::_ref_type bitmask_type_descriptor {traits<TypeDescriptor>::make_shared()};
    bitmask_type_descriptor->kind(TK_BITMASK);
    bitmask_type_descriptor->name("MyBitMask");
    bitmask_type_descriptor->element_type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(
                TK_BOOLEAN));
    bitmask_type_descriptor->bound().push_back(8);
    DynamicTypeBuilder::_ref_type bitmask_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                    bitmask_type_descriptor)};
 */

// Add bitfield members to the bitmask type
MemberDescriptor::_ref_type bitfield_member_descriptor {traits<MemberDescriptor>::make_shared()};
bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
bitfield_member_descriptor->name("flag0");
bitfield_member_descriptor->position(0);
bitmask_builder->add_member(bitfield_member_descriptor);
bitfield_member_descriptor = traits<MemberDescriptor>::make_shared();
bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
bitfield_member_descriptor->name("flag1");
bitmask_builder->add_member(bitfield_member_descriptor);
bitfield_member_descriptor = traits<MemberDescriptor>::make_shared();
bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
bitfield_member_descriptor->name("flag2");
bitmask_builder->add_member(bitfield_member_descriptor);
bitfield_member_descriptor = traits<MemberDescriptor>::make_shared();
bitfield_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_BOOLEAN));
bitfield_member_descriptor->name("flag5");
bitfield_member_descriptor->position(5);
bitmask_builder->add_member(bitfield_member_descriptor);
// Build the bitmask type
DynamicType::_ref_type bitmask_type =  bitmask_builder->build();

// Add a bitmask member to the struct
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->name("my_bitmask");
member_descriptor->type(bitmask_type);
struct_builder->add_member(member_descriptor);
// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};

// Set and retrieve values for bitmask member.
uint8_t in_value {3}; // Setting both "flag0" and "flag1" simultaneously.
uint8_t out_value {0};
data->set_uint8_value(data->get_member_id_by_name("my_bitmask"), in_value);
data->get_uint8_value(out_value, data->get_member_id_by_name("my_bitmask"));

// Set and retrieve specific bitflag
bool in_bitflag_value = true;
bool out_bitflag_value = false;
DynamicData::_ref_type bitmask_data = data->loan_value(data->get_member_id_by_name("my_bitmask"));
bitmask_data->set_boolean_value(bitmask_data->get_member_id_by_name("flag5"), in_bitflag_value);
bitmask_data->get_boolean_value(out_bitflag_value, bitmask_data->get_member_id_by_name("flag5"));
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Bitmask Types](../xml_configuration/dynamic_types.md#xmldynamictypes-bitmask).

<a id="xtypes-supportedtypes-bitset"></a>

<a id="bitset-types"></a>

### Bitset Types

Bitset types are an aggregation of bitfields.
The `TypeKind` used to identify bitsets is `TK_BITSET`.
`bound` property contains the sequence with the bitfield’s bitcount (number of bits).
In order to be consistent, the length of the `bound` sequence must agree with the number of bitfields.
Bitset types have single inheritance support, so a bitset type might extend one other already defined bitset.
The bitset type which is extended should be configured in the [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `base_type` property.

Bitfields must be configured using the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) by calling the [`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE)
function with the corresponding [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE).
At least one bitfield is required for the bitset to be consistent (**[Pro]** allows bitsets with unnamed bitfields only).
Bitfield name is configured using [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `name` property, and the bitfield initial bit position
is set using [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `id` property.

#### NOTE
For derived bitsets, the first bitfield initial position must be after the bits defined by the parent bitset type.

#### NOTE
Bitsets spanning more than 64 bits are rejected.
The trailing unused bits of a bitset are preserved through the `@eprosima::bit_span` annotation.

A bitfield manages exclusively a set of bits, so no bitfield superposition is allowed.
Additionally, [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `type` property might be set to configure an integer type to access bitfield
data.
If not set, the minimum unsigned integer type is used instead:

| Number of bits   | C++ holder type   |
|------------------|-------------------|
| `1`              | `bool`            |
| `2-8`            | `uint8_t`         |
| `9-16`           | `uint16_t`        |
| `17-32`          | `uint32_t`        |
| `33-64`          | `uint64_t`        |

Each bitfield (or member) works like its primitive type with the only difference that the internal storage only
modifies the involved bits instead of the full primitive value.

### OMG-IDL

```omg-idl
bitset ParentBitSet
{
    bitfield<3> a;
    bitfield<1> b;
    bitfield<4>;
    bitfield<10> c;
    bitfield<12, short> d;
};

bitset ChildBitSet : ParentBitSet
{
    bitfield<1> e;
    bitfield<20, unsigned long> f;
};

struct BitsetStruct
{
   ChildBitSet my_bitset;
};
```

### XML

```xml
<bitset name="ParentBitset">
    <bitfield name="a" bit_bound="3"/>
    <bitfield name="b" bit_bound="1"/>
    <bitfield bit_bound="4"/>
    <bitfield name="c" bit_bound="10"/>
    <bitfield name="d" bit_bound="12" type="int16"/>
</bitset>

```

### C++

```c++
// Define a struct type to contain the bitset
TypeDescriptor::_ref_type struct_type_descriptor {traits<TypeDescriptor>::make_shared()};
struct_type_descriptor->kind(TK_STRUCTURE);
struct_type_descriptor->name("BitsetStruct");
DynamicTypeBuilder::_ref_type struct_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                  struct_type_descriptor)};

// Define type for parent bitset
TypeDescriptor::_ref_type bitset_type_descriptor {traits<TypeDescriptor>::make_shared()};
bitset_type_descriptor->kind(TK_BITSET);
bitset_type_descriptor->name("ParentBitSet");
bitset_type_descriptor->bound({3, 1, 10, 12});
DynamicTypeBuilder::_ref_type bitset_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                  bitset_type_descriptor)};
// Add members to the bitset type
MemberDescriptor::_ref_type bitset_member_descriptor {traits<MemberDescriptor>::make_shared()};
bitset_member_descriptor->name("a");
bitset_member_descriptor->id(0);
bitset_builder->add_member(bitset_member_descriptor);
bitset_member_descriptor = traits<MemberDescriptor>::make_shared();
bitset_member_descriptor->name("b");
bitset_member_descriptor->id(3);
bitset_builder->add_member(bitset_member_descriptor);
bitset_member_descriptor = traits<MemberDescriptor>::make_shared();
bitset_member_descriptor->name("c");
bitset_member_descriptor->id(8);
bitset_builder->add_member(bitset_member_descriptor);
bitset_member_descriptor = traits<MemberDescriptor>::make_shared();
bitset_member_descriptor->name("d");
bitset_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
bitset_member_descriptor->id(18);
bitset_builder->add_member(bitset_member_descriptor);
// Build the bitset type
DynamicType::_ref_type parentbitset_type = bitset_builder->build();

// Create child bitset type
TypeDescriptor::_ref_type childbitset_type_descriptor {traits<TypeDescriptor>::make_shared()};
childbitset_type_descriptor->kind(TK_BITSET);
childbitset_type_descriptor->name("ChildBitSet");
childbitset_type_descriptor->base_type(parentbitset_type);
childbitset_type_descriptor->bound({1, 20});
DynamicTypeBuilder::_ref_type childbitset_builder {DynamicTypeBuilderFactory::get_instance()->create_type(
                                                       childbitset_type_descriptor)};
// Add members to the child bitset type
MemberDescriptor::_ref_type childbitset_member_descriptor {traits<MemberDescriptor>::make_shared()};
childbitset_member_descriptor->name("e");
childbitset_member_descriptor->id(30);
childbitset_builder->add_member(childbitset_member_descriptor);
childbitset_member_descriptor = traits<MemberDescriptor>::make_shared();
childbitset_member_descriptor->name("d");
childbitset_member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_UINT32));
childbitset_member_descriptor->id(31);
childbitset_builder->add_member(childbitset_member_descriptor);
// Build the child bitset type
DynamicType::_ref_type bitset_type = childbitset_builder->build();

// Add the bitset member to the struct
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->name("my_bitset");
member_descriptor->type(bitset_type);
struct_builder->add_member(member_descriptor);
// Build the struct type
DynamicType::_ref_type struct_type {struct_builder->build()};
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(struct_type)};
// Get the loan for the bitset member
DynamicData::_ref_type bitset_data = data->loan_value(data->get_member_id_by_name("my_bitset"));

// Set and retrieve bitfield values
int16_t in_value {2};
int16_t out_value;
bitset_data->set_int16_value(bitset_data->get_member_id_by_name("d"), in_value);
bitset_data->get_int16_value(out_value, bitset_data->get_member_id_by_name("d"));
// Return de data loan of the member
data->return_loaned_value(bitset_data);
```

For a detailed explanation about the XML definition of this type, please refer to
[XML Bitset Types](../xml_configuration/dynamic_types.md#xmldynamictypes-bitset).

<a id="xtypes-annotations"></a>

<a id="annotations"></a>

### Annotations

<a id="xtypes-custom-annotations"></a>

<a id="custom-annotations"></a>

#### Custom annotations

Both types and type members might be annotated using [`DynamicTypeBuilder::apply_annotation`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder16apply_annotationEN6traitsI20AnnotationDescriptorE8ref_typeE) and
[`DynamicTypeBuilder::apply_annotation_to_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder26apply_annotation_to_memberE8MemberIdN6traitsI20AnnotationDescriptorE8ref_typeE) API respectively.

Annotations are defined using an [`AnnotationDescriptor`](../api_reference/dds_pim/xtypes/annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE) which provides two properties: `type` and
`value`.
The annotation type must be the DynamicType representing the annotation being applied.
The `TypeKind` used to identify annotations is `TK_ANNOTATION`.
The annotation name is set in [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `name` property.

The annotation type might have any number of parameters.
Annotation parameters must be configured  using the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) by calling the
[`DynamicTypeBuilder::add_member`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilder10add_memberEN6traitsI16MemberDescriptorE8ref_typeE) function with the corresponding [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE).
Annotation parameters must define the annotation parameter name in [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `name` property,
and the parameter type using `type` property.
Only the following types can be used to define an annotation parameter:

* [Primitive types](#primitive-types)
* [String types](#string-types)
* [Enumeration types](#enumeration-types)

#### NOTE
Currently, wide string types are not supported as annotation parameters by the Dynamic Language Binding API.

Annotation parameter values are defined with [`AnnotationDescriptor`](../api_reference/dds_pim/xtypes/annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptorE) `value` property using
[`AnnotationDescriptor::set_value()`](../api_reference/dds_pim/xtypes/annotationdescriptor.md#_CPPv4N8eprosima7fastdds3dds20AnnotationDescriptor9set_valueERK10ObjectNameRK10ObjectName).
The annotation parameter name provided to the API must coincide with the one defined with the annotation type.
The annotation parameter value must be converted to its string representation.

#### NOTE
Currently, custom annotations are not supported when loading dynamic types from
[XML](../xml_configuration/dynamic_types.md#xmldynamictypes), so there is no XML example below.

### OMG-IDL

```omg-idl
@annotation MyAnnotation
{
    short length;
};

@MyAnnotation(length = 5)
struct AnnotatedStruct
{
    @MyAnnotation(length = 10) string string_var;
};
```

### C++

```c++
// Create the structure to annotate
TypeDescriptor::_ref_type type_descriptor {traits<TypeDescriptor>::make_shared()};
type_descriptor->kind(TK_STRUCTURE);
type_descriptor->name("AnnotatedStruct");
DynamicTypeBuilder::_ref_type type_builder {DynamicTypeBuilderFactory::get_instance()->
                                                    create_type(type_descriptor)};
MemberDescriptor::_ref_type member_descriptor {traits<MemberDescriptor>::make_shared()};
member_descriptor->name("string_var");
member_descriptor->type(DynamicTypeBuilderFactory::get_instance()->create_string_type(static_cast<uint32_t>(
            LENGTH_UNLIMITED))->build());
type_builder->add_member(member_descriptor);

// Create the annotation type
AnnotationDescriptor::_ref_type annotation_descriptor {traits<AnnotationDescriptor>::make_shared()};
TypeDescriptor::_ref_type annotation_type {traits<TypeDescriptor>::make_shared()};
annotation_type->kind(TK_ANNOTATION);
annotation_type->name("MyAnnotation");
DynamicTypeBuilder::_ref_type annotation_builder {DynamicTypeBuilderFactory::get_instance()->
                                                          create_type(annotation_type)};

// Add members to the annotation type
MemberDescriptor::_ref_type annotation_parameter {traits<MemberDescriptor>::make_shared()};
annotation_parameter->name("length");
annotation_parameter->type(DynamicTypeBuilderFactory::get_instance()->get_primitive_type(TK_INT16));
annotation_builder->add_member(annotation_parameter);
// Set the annotation type using the annotation descriptor
annotation_descriptor->type(annotation_builder->build());
// Set the value of the annotation
annotation_descriptor->set_value("length", std::to_string(5));
// Apply the annotation to the structure
type_builder->apply_annotation(annotation_descriptor);

// Reuse annotation descriptor to annotate struct member
annotation_descriptor = traits<AnnotationDescriptor>::make_shared();
annotation_descriptor->type(annotation_builder->build());
annotation_descriptor->set_value("length", std::to_string(10));

DynamicTypeMember::_ref_type member;
type_builder->get_member_by_name(member, "string_var");
type_builder->apply_annotation_to_member(member->get_id(), annotation_descriptor);
```

<a id="xtypes-builtin-annotations"></a>

<a id="builtin-annotations"></a>

#### Builtin annotations

Beside the user-defined custom annotations, there are a number of builtin annotations that have already mentioned
throughout this section.

#### WARNING
Currently, annotating union discriminators is not supported.

The table below summarizes the builtin annotations that can be applied using the Dynamic Language Binding API.
Please, refer to [builtin annotations](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations) for the complete list and their behavior.

| Builtin annotation   | Dynamic Language Binding API                                                                                                                                                                                                                                                                                                                                                                                                                                       | Dynamic Language Binding support   | XML Dynamic Type profiles support                              | IDL Parsing Dynamic Type support                                   |
|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|------------------------------------|----------------------------------------------------------------|--------------------------------------------------------------------|
| `@appendable`        | [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `extensibility_kind` property.                                                                                                                                                                                                                                                                                                                 | ✅                                  | ❌                                                              | ✅                                                                  |
| `@autoid`            | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `id` property (automatic assignment).                                                                                                                                                                                                                                                                                                    | ❌                                  | ❌                                                              | ✅ **[Pro]**                                                        |
| `@bit_bound`         | [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `bound` property for [Bitset Types](#xtypes-supportedtypes-bitset) and<br/>[Bitmask Types](#xtypes-supportedtypes-bitmask). <br/><br/>[`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `literal_type` property for [Enumeration Types](#xtypes-supportedtypes-enumeration). | ✅                                  | ⚠️ ([Enumeration types](#enumeration-types) not configurable). | ✅ (**[Pro]** for [Bitmask Types](#xtypes-supportedtypes-bitmask)). |
| `@default`           | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `default_value` property.                                                                                                                                                                                                                                                                                                                | ✅                                  | ❌                                                              | ✅                                                                  |
| `default_literal`    | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `is_default_literal` property.                                                                                                                                                                                                                                                                                                           | ✅                                  | ❌                                                              | ✅                                                                  |
| `@extensibility`     | [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `extensibility_kind` property.                                                                                                                                                                                                                                                                                                                 | ✅                                  | ❌                                                              | ✅                                                                  |
| `@external`          | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `is_shared` property.                                                                                                                                                                                                                                                                                                                    | ❌                                  | ❌                                                              | ✅                                                                  |
| `@final`             | [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `extensibility_kind` property.                                                                                                                                                                                                                                                                                                                 | ✅                                  | ❌                                                              | ✅                                                                  |
| `@hashid`            | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `id` property (hash-derived).                                                                                                                                                                                                                                                                                                            | ❌                                  | ❌                                                              | ✅ **[Pro]**                                                        |
| `@id`                | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `id` property.                                                                                                                                                                                                                                                                                                                           | ✅                                  | ❌                                                              | ✅ **[Pro]**                                                        |
| `@key` / `@Key`      | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `is_key` property.                                                                                                                                                                                                                                                                                                                       | ✅                                  | ❌                                                              | ✅                                                                  |
| `@mutable`           | [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `extensibility_kind` property.                                                                                                                                                                                                                                                                                                                 | ✅                                  | ❌                                                              | ✅                                                                  |
| `@nested`            | [`TypeDescriptor`](../api_reference/dds_pim/xtypes/typedescriptor.md#_CPPv4N8eprosima7fastdds3dds14TypeDescriptorE) `is_nested` property.                                                                                                                                                                                                                                                                                                                          | ❌                                  | ❌                                                              | ✅                                                                  |
| `@optional`          | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `is_optional` property.                                                                                                                                                                                                                                                                                                                  | ✅ **[Pro]**                        | ❌                                                              | ✅                                                                  |
| `@position`          | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `position` property.                                                                                                                                                                                                                                                                                                                     | ✅                                  | ✅                                                              | ✅ **[Pro]**                                                        |
| `@try_construct`     | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `try_construct_kind` property.                                                                                                                                                                                                                                                                                                           | ❌                                  | ❌                                                              | ✅                                                                  |
| `@value`             | [`MemberDescriptor`](../api_reference/dds_pim/xtypes/memberdescriptor.md#_CPPv4N8eprosima7fastdds3dds16MemberDescriptorE) `literal_value` property.                                                                                                                                                                                                                                                                                                                | ✅                                  | ✅                                                              | ✅                                                                  |
| `@verbatim`          | [`VerbatimTextDescriptor`](../api_reference/dds_pim/xtypes/verbatimtextdescriptor.md#_CPPv4N8eprosima7fastdds3dds22VerbatimTextDescriptorE)                                                                                                                                                                                                                                                                                                                        | ❌                                  | ❌                                                              | ❌                                                                  |

<a id="xtypes-complextypes"></a>

<a id="managing-complex-types-data"></a>

## Managing Complex Types Data

Some [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) instances manage complex types that cannot be directly modified with the primitive getters and
setters.
Dynamic Language Binding provides two possible approaches for managing complex data types:

1. [`DynamicData::get_complex_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17get_complex_valueERN6traitsI11DynamicDataE8ref_typeE8MemberId) and [`DynamicData::set_complex_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17set_complex_valueE8MemberIdN6traitsI11DynamicDataE8ref_typeE): This API allows to get/set generic DynamicData.
   The main difference with the next approach is that this API performs always a copy.
2. [`DynamicData::loan_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData10loan_valueE8MemberId): this API allows to loan a reference to a DynamicData to work with preventing the data copy.
   [`DynamicData::return_loaned_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData19return_loaned_valueEN6traitsI11DynamicDataE8ref_typeE) must be called to return the loan.
   Calling [`DynamicData::loan_value`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData10loan_valueE8MemberId) for an already loaned value will fail.

The following snippet includes an example of managing complex data using the same structure as the one defined in
[Structure types](#structure-types):

```c++
// Create dynamic data based on the struct type
DynamicData::_ref_type data {DynamicDataFactory::get_instance()->create_data(complexstruct_type)};

// Get/Set complex API (copy)
DynamicData::_ref_type complex_data;
data->get_complex_value(complex_data, data->get_member_id_by_name("complex_member"));
// Set data
int32_t in_value {10};
complex_data->set_int32_value(complex_data->get_member_id_by_name("first"), in_value);
data->set_complex_value(data->get_member_id_by_name("complex_member"), complex_data);

// Loan API
DynamicData::_ref_type loan_data = data->loan_value(data->get_member_id_by_name("complex_member"));
loan_data->set_int32_value(loan_data->get_member_id_by_name("first"), in_value);
int32_t out_value;
loan_data->get_int32_value(out_value, loan_data->get_member_id_by_name("first"));
data->return_loaned_value(loan_data);
```
