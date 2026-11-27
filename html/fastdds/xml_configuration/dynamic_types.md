<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="xmldynamictypes"></a>

<a id="dynamic-types-profiles"></a>

# Dynamic Types profiles

*Fast DDS* supports the implementation of [Dynamic Language Binding](../xtypes/language_binding.md#xtypes-language-binding) by defining them through XML files.
Thus the topic data types can be modified without the need to modify the source code of the DDS application.

<a id="xml-structure"></a>

## XML Structure

The definition of data type profiles in the XML file is done with the `types` tag.
Each `types` element can contain one or more [Type definitions](#type-definition).
Defining several types within a `types` element or a single type for each `types` element has the same
result.
Below, an example of a stand-alone types definition via XML is shown.

```xml
<types xmlns="http://www.eprosima.com">
    <type>
        <!-- Type definition -->
    </type>
    <type>
        <!-- Type definition -->
        <!-- Type definition -->
    </type>
</types>
```

#### NOTE
For more information on the difference between stand-alone and rooted definitions please refer to section
[Rooted vs Standalone profiles definition](making_xml_profiles.md#rootedvsstandalone).

<a id="type-definition"></a>

<a id="id1"></a>

## Type definition

Below, the types supported by *eProsima Fast DDS* are presented.
For further information about the supported [Dynamic Language Binding](../xtypes/language_binding.md#xtypes-language-binding), please, refer to [Supported Types](../xtypes/language_binding.md#xtypes-supportedtypes).
For each type listed below, an example of how to build the type’s XML profile is provided.

* [Primitive types](#primitive-types)
* [String Types](#xmldynamictypes-strings)
* [Enumeration Types](#xmldynamictypes-enums)
* [Bitmask Types](#xmldynamictypes-bitmask)
* [Alias Types](#xmldynamictypes-typedef)
* [Sequence Types](#xmldynamictypes-sequence)
* [Array Types](#xmldynamictypes-array)
* [Map Types](#xmldynamictypes-map)
* [Structure Types](#xmldynamictypes-struct)
* [Union Types](#xmldynamictypes-union)
* [Bitset Types](#xmldynamictypes-bitset)

<a id="xmldynamictypes-member"></a>

<a id="member-types"></a>

### Member types

Member types are defined as any type that can belong to a [Struct](#xmldynamictypes-struct) or a
[Union](#xmldynamictypes-union), or be aliased by a [Alias](#xmldynamictypes-typedef).
These can be defined by the `<member>` XML tag.
A member can be annotated as `key` (equivalent of the IDL’s `@key`) by setting the `key` attribute to `"true"`.

```xml
<member name="my_long" type="int32" key="true"/>
```

<a id="xmldynamictypes-primivites"></a>

<a id="primitive-types"></a>

### Primitive types

Primitive types are built-in types and they should be declared as members of an aggregated type
([Structure Types](#xmldynamictypes-struct) or [Union Types](#xmldynamictypes-union)).
Primitive types are declared by attribute `type` and the possible values are listed in the table below.
Please, refer to [Primitive Types](../xtypes/language_binding.md#xtypes-supportedtypes-primitive) for more information on primitive types.

| `boolean`   | `byte`   | `char8`   | `char16`   | `int32`    |
|-------------|----------|-----------|------------|------------|
| `uint32`    | `int8`   | `uint8`   | `int16`    | `uint16`   |
| `int64`     | `uint64` | `float32` | `float64`  | `float128` |

All of them are declared as follows:

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

### IDL

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

<a id="xmldynamictypes-strings"></a>

<a id="string-types"></a>

### String Types

String types should be defined as members of an aggregated type ([Structure Types](#xmldynamictypes-struct) or
[Union Types](#xmldynamictypes-union)).
String types are defined with attribute `type` set to `string` or `wstring`.
An optional attribute `stringMaxLength` might used to set a maximum length for the string collection.
Please, refer to [String Types](../xtypes/language_binding.md#xtypes-supportedtypes-string) for more information on string types.

### XML

```xml
<struct name="StringsStruct">
    <member name="my_string" type="string"/>
    <member name="my_wstring" type="wstring"/>
    <member name="my_bounded_string" type="string" stringMaxLength="41925"/>
    <member name="my_bounded_wstring" type="wstring" stringMaxLength="20925"/>
</struct>
```

### IDL

```omg-idl
struct StringsStruct
{
    string my_string;
    wstring my_wstring;
    string<41925> my_bounded_string;
    wstring<20925> my_bounded_wstring;
};
```

<a id="xmldynamictypes-enums"></a>

<a id="enumeration-types"></a>

### Enumeration Types

Enumeration types are defined using the `<enum>` tag.
Attribute `name` and at least one `<enumerator>` child element are mandatory.
Enumeration literals are defined using the `<enumerator>` tag with mandatory attribute `name`.
Optionally, unsigned integer attribute `value` might be added to set a specific value for the enumeration literal.

#### NOTE
`value` attribute is equivalent to `@value` builtin annotation.

Please, refer to [Enumeration Types](../xtypes/language_binding.md#xtypes-supportedtypes-enumeration) for more information on enumeration types.

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

### IDL

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

<a id="xmldynamictypes-bitmask"></a>

<a id="bitmask-types"></a>

### Bitmask Types

Bitmask types are defined using the `<bitmask>` tag.
Attribute `name` and at least on `<bit_value>` child element are mandatory.
Optionally, `bit_bound` attribute might be set to specify the bitmask bound (by default 32 bits).
Bitflag elements are defined using the `<bit_value>` tag with mandatory attribute `name`.
Optionally, `position` attribute might be defined to set the bitflag position within the bitmask.
Please, refer to [Bitmask Types](../xtypes/language_binding.md#xtypes-supportedtypes-bitmask) for more information on bitmask types.

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

### IDL

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

<a id="xmldynamictypes-typedef"></a>

<a id="alias-types"></a>

### Alias Types

Alias types are defined using the `<typedef>` tag.
Attributes `name` and `type` are mandatory.
Depending on the aliased type, some other mandatory and/or optional attributes might be necessary or available.
Non-primitive types must define the `type` attribute as `nonBasic` and include the `nonBasicTypeName`
attribute with the name of the aliased type.
Please, refer to [Alias Types](../xtypes/language_binding.md#xtypes-supportedtypes-alias) for more information on alias types.

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

### IDL

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

<a id="xmldynamictypes-sequence"></a>

<a id="sequence-types"></a>

### Sequence Types

Sequence types should be defined as members of an aggregated type ([Structure Types](#xmldynamictypes-struct) or
[Union Types](#xmldynamictypes-union)).
Sequence types are defined with mandatory attributes `type` set to the collection’s element type, and
`sequenceMaxLength` used to set the maximum collection’s length.
Unbounded sequences should set `sequenceMaxLength` attribute to `-1`.
Please, refer to [Sequence types](../xtypes/language_binding.md#xtypes-supportedtypes-sequence) for more information on sequence types.

### XML

```xml
<struct name="SequenceStruct">
    <member name="bitmask_sequence" type="nonBasic" nonBasicTypeName="MyBitMask" sequenceMaxLength="-1"/>
    <member name="short_sequence" sequenceMaxLength="5" type="int16"/>
</struct>
```

### IDL

```omg-idl
struct SequenceStruct
{
    sequence<MyBitMask> bitmask_sequence;
    sequence<short, 5> short_sequence;
};
```

<a id="xmldynamictypes-array"></a>

<a id="array-types"></a>

### Array Types

Array types should be defined as members of an aggregated type ([Structure Types](#xmldynamictypes-struct) or
[Union Types](#xmldynamictypes-union)).
Array types are defined with mandatory attributes `type` set to the collection’s element type, and
`arrayDimensions` used to set the collection’s dimensions.
The format of `arrayDimensions` attribute value is the size of each dimension separated by commas.
Please, refer to [Array types](../xtypes/language_binding.md#xtypes-supportedtypes-array) for more information on array types.

### XML

```xml
<struct name="ArrayStruct">
    <member name="long_array" type="int32" arrayDimensions="2,3,4"/>
</struct>
```

### IDL

```omg-idl
struct ArrayStruct
{
    long long_array[2][3][4];
};
```

<a id="xmldynamictypes-map"></a>

<a id="map-types"></a>

### Map Types

Map types should be defined as members of an aggregated type ([Structure Types](#xmldynamictypes-struct) or
[Union Types](#xmldynamictypes-union)).
Map types are defined with mandatory attributes `type` set to the map’s value type, `key_type` set to the
map’s key type, and `mapMaxLength` used to set the maximum map’s number of key-value pairs.
Unbounded maps should set `mapMaxLength` attribute to `-1`.
Please, refer to [Map Types](../xtypes/language_binding.md#xtypes-supportedtypes-map) for more information on map types.

### XML

```xml
<struct name="MapStruct">
    <member name="string_alias_unbounded_map" type="nonBasic" nonBasicTypeName="MyAliasedBoundedString" key_type="string" mapMaxLength="-1"/>
    <member name="short_long_map" type="int32" key_type="int16" mapMaxLength="2"/>
</struct>
```

### IDL

```omg-idl
struct MapStruct
{
    map<string, MyAliasedBoundedString> string_alias_unbounded_map;
    map<short, long, 2> short_long_map;
};
```

<a id="xmldynamictypes-struct"></a>

<a id="structure-types"></a>

### Structure Types

Structure types are defined using the `<struct>` tag with mandatory attribute `name`.
Structure inheritance may be configured setting optional attribute `baseType`.
XML Structure Types require at least one member defined.

#### NOTE
[IDL specification](https://www.omg.org/spec/IDL/4.2/About-IDL) introduced in version 4.1 the possibility of void
content structures.
Empty structures are not supported in XML Types profiles yet.

Structure members are defined using the `<member>` tag with mandatory attributes `name` and `type`.
Depending on the member type, some other mandatory and/or optional attributes might be necessary or available.
Non-primitive types must define the `type` attribute as `nonBasic` and include the `nonBasicTypeName`
attribute with the name of the member type.

#### NOTE
Currently, XML Types profiles does not support setting the member ID or marking a member as key.

Please, refer to [Structure Types](../xtypes/language_binding.md#xtypes-supportedtypes-structure) for more information on structure types.

### XML

```xml
<struct name="InnerStruct">
    <!-- XML does not support setting Member ID -->
    <member name="first" type="int32"/>
</struct>

```

### IDL

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

<a id="xmldynamictypes-union"></a>

<a id="union-types"></a>

### Union Types

Union types are defined using the `<union>` tag with mandatory attribute `name`.
A mandatory discriminator child must be defined using `<discriminator>` tag.
Discriminator element requires `<type>` as mandatory attribute.

Union types also require at least one case child defined using the `<case>` tag.
Each case child requires at least one label child using the `<caseDiscriminator>` tag.
`value` attribute is mandatory and defines the label value.
Several labels might be defined using several `<caseDiscriminator>` elements.
Each case child must have exclusively one union member defined.

Union members are defined using the `<member>` tag with mandatory attributes `name` and `type`.
Depending on the member type, some other mandatory and/or optional attributes might be necessary or available.
Non-primitive types must define the `type` attribute as `nonBasic` and include the `nonBasicTypeName`
attribute with the name of the member type.
At least one union member must be defined for the union type to be consistent.

#### NOTE
Currently, XML Types profiles does not support setting the member ID or marking a member as key.

Please, refer to [Union Types](../xtypes/language_binding.md#xtypes-supportedtypes-union) for more information on the union types.

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

### IDL

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

<a id="xmldynamictypes-bitset"></a>

<a id="bitset-types"></a>

### Bitset Types

Bitset types are defined using the `<bitset>` tag with mandatory attribute `name`.
Bitset inheritance may be configured setting optional attribute `baseType`.
At least one bitfield child must be defined using `bitfield` tag.

Bitfield elements require mandatory attribute `bit_bound` with the number of bits managed by the bitfield (maximum
64 bits).
Optionally, attributes `name` and `type` might be defined.
An anonymous bitfield (attribute `name` not set) is not accessible and serves as padding between named bitfields.
The `type` attribute can ease bitfield management explicitly setting an integer type that handles the bitfield.

Please, refer to [Bitset Types](../xtypes/language_binding.md#xtypes-supportedtypes-bitset) for more information about the bitset types.

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

### IDL

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

<a id="usage"></a>

<a id="loading-xml-types-profile-in-fast-dds-application"></a>

## Loading XML Types profile in *Fast DDS* application

*Fast DDS* application can use types defined in XML configuration files once those files have been loaded into the
[`DomainParticipantFactory`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactoryE) using [`load_XML_profiles_file()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory22load_XML_profiles_fileERKNSt6stringE).
Types might be retrieved using [`DomainParticipantFactory::get_dynamic_type_builder_from_xml_by_name()`](../api_reference/dds_pim/domain/domainparticipantfactory.md#_CPPv4N8eprosima7fastdds3dds24DomainParticipantFactory41get_dynamic_type_builder_from_xml_by_nameERKNSt6stringERN18DynamicTypeBuilder9_ref_typeE).
After getting the DynamicType, objects of [`DynamicPubSubType`](../api_reference/dds_pim/xtypes/dynamicpubsubtype.md#_CPPv4N8eprosima7fastdds3dds17DynamicPubSubTypeE) class might be instantiated and used to write/read
data.

```c++
// Create a DomainParticipant
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Load the XML File
if (RETCODE_OK ==
        DomainParticipantFactory::get_instance()->load_XML_profiles_file("my_profiles.xml"))
{
    // Retrieve instance of the desired type
    DynamicTypeBuilder::_ref_type my_struct_type;
    if (RETCODE_OK !=
            DomainParticipantFactory::get_instance()->get_dynamic_type_builder_from_xml_by_name(
                "MyStruct", my_struct_type))
    {
        // Error
        return;
    }

    // Register MyStruct type
    TypeSupport my_struct_type_support(new DynamicPubSubType(my_struct_type->build()));
    my_struct_type_support.register_type(participant, nullptr);
}
else
{
    std::cout << "Cannot open XML file \"types.xml\". "
              << "Please, set the correct path to the XML file"
              << std::endl;
}
```
