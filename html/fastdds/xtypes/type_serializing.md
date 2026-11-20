<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="xtypes-serialization-utilities"></a>

<a id="serialization-utilities"></a>

# Serialization Utilities

Fast DDS provides methods to serialize XTypes objects, enabling efficient data exchange and manipulation
within distributed systems.
Serialization is a crucial process in data distribution services, as it converts complex data structures into a
format that can be easily transmitted and reconstructed across different platforms and programming environments.

<a id="xtypes-serialization-utilities-idl"></a>

<a id="dynamic-type-to-idl"></a>

## Dynamic Type to IDL

The method [`idl_serialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds13idl_serializeERKN11DynamicType9_ref_typeERNSt7ostreamE) serializes a [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE) object to its IDL representation.

#### NOTE
The conversion to IDL only supports the following [builtin annotations](../../fastddsgen/dataTypes/dataTypes.md#builtin-annotations):
`@bit_bound`, `@extensibility`, `@key`, and `@position`.

#### WARNING
The conversion to IDL of a [Bitset](language_binding.md#xtypes-supportedtypes-bitset) with inheritance merges derived
[Bitsets](language_binding.md#xtypes-supportedtypes-bitset) with their base [Bitset](language_binding.md#xtypes-supportedtypes-bitset).

#### WARNING
The conversion to IDL dismisses values explicitly set to their default value.
For example, the default `@bit_bound` value of a [Bitmask](language_binding.md#xtypes-supportedtypes-bitmask) is 32.
If a user were to explicitly set the `@bit_bound` value of a
[Bitmask](language_binding.md#xtypes-supportedtypes-bitmask) to 32 and then serialize the [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE) to IDL, the
`@bit_bound` would not be included in the IDL.

<a id="xtypes-serialization-utilities-idl-example"></a>

<a id="example-convert-a-discovered-type-to-idl-format"></a>

### Example: Convert a discovered type to IDL format

The following example demonstrates how to use the [`idl_serialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds13idl_serializeERKN11DynamicType9_ref_typeERNSt7ostreamE) method in Fast DDS to convert
discovered types to IDL format.
Each time the subscriber discovers a new [`DataReader`](../api_reference/dds_pim/subscriber/datareader.md#_CPPv4N8eprosima7fastdds3dds10DataReaderE) or [`DataWriter`](../api_reference/dds_pim/publisher/datawriter.md#_CPPv4N8eprosima7fastdds3dds10DataWriterE), it uses the
[`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) to build a [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE) and serialize it to IDL format.
Please refer to [Remote type discovery and endpoint matching](../use_cases/remote_type_discovery_matching/remote_type_discovery_matching.md#use-case-remote-type-discovery-and-matching) section for more details on how to implement
remote type discovery.

```c++
/* Custom Callback on_data_reader_discovery */
void on_data_reader_discovery(
        DomainParticipant* /* participant */,
        eprosima::fastdds::rtps::ReaderDiscoveryStatus /* reason */,
        const eprosima::fastdds::dds::SubscriptionBuiltinTopicData& info,
        bool& /* should_be_ignored */) override
{
    // Get remote type information
    xtypes::TypeObject remote_type_object;
    if (RETCODE_OK != DomainParticipantFactory::get_instance()->type_object_registry().get_type_object(
                info.type_information.type_information.complete().typeid_with_size().type_id(),
                remote_type_object))
    {
        // Error
        return;
    }

    // Build remotely discovered type
    DynamicType::_ref_type remote_type = DynamicTypeBuilderFactory::get_instance()->create_type_w_type_object(
        remote_type_object)->build();

    // Serialize DynamicType into its IDL representation
    std::stringstream idl;
    if (RETCODE_OK != idl_serialize(remote_type, idl))
    {
        // Error
        return;
    }

    // Print IDL representation
    std::cout << "Type discovered:\n" << idl.str() << std::endl;
}

/* Custom Callback on_data_writer_discovery */
void on_data_writer_discovery(
        DomainParticipant* /* participant */,
        eprosima::fastdds::rtps::WriterDiscoveryStatus /*reason*/,
        const eprosima::fastdds::dds::PublicationBuiltinTopicData& info,
        bool& /* should_be_ignored */) override
{
    // Get remote type information
    xtypes::TypeObject remote_type_object;
    if (RETCODE_OK != DomainParticipantFactory::get_instance()->type_object_registry().get_type_object(
                info.type_information.type_information.complete().typeid_with_size().type_id(),
                remote_type_object))
    {
        // Error
        return;
    }

    // Build remotely discovered type
    DynamicType::_ref_type remote_type = DynamicTypeBuilderFactory::get_instance()->create_type_w_type_object(
        remote_type_object)->build();

    // Serialize DynamicType into its IDL representation
    std::stringstream idl;
    if (RETCODE_OK != idl_serialize(remote_type, idl))
    {
        // Error
        return;
    }

    // Print IDL representation
    std::cout << "Type discovered:\n" << idl.str() << std::endl;
}

```

<a id="xtypes-serialization-utilities-dyndata-json"></a>

<a id="dynamicdata-to-json"></a>

## DynamicData to JSON

In the context of DDS (Data Distribution Service), [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE) represents the structure of the data being
distributed across the system.
Each [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponds to an object of the type represented by its [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE),
providing functionalities to access and modify data values.
To enhance interoperability and readability, it is often useful to serialize [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) into a more
manageable format, to enable easier data processing and analysis across different systems and applications.
The method [`json_serialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds14json_serializeERKN11DynamicData9_ref_typeE21DynamicDataJsonFormatRNSt7ostreamE) converts a [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object into a JSON object, then
dumped into a `std::ostream`.
The inverse conversion is also possible using the method [`json_deserialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds16json_deserializeERKNSt6stringERKN11DynamicType9_ref_typeE21DynamicDataJsonFormatRN11DynamicData9_ref_typeE), as described in the
[corresponding section](#xtypes-serialization-utilities-json-dyndata).

<a id="supported-types"></a>

### Supported Types

This section provides the serialization of [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) to JSON ostream for all supported types.

<a id="xtypes-serialization-utilities-primitive"></a>

<a id="primitives"></a>

#### Primitives

Primitive types are the basic building blocks for every [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE).
Below is an example of the definition of primitive types in IDL:

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

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "my_bool": false,
    "my_octet": 0,
    "my_char": "\u0000",
    "my_wchar": "\u0000",
    "my_long": 0,
    "my_ulong": 0,
    "my_int8": 0,
    "my_uint8": 0,
    "my_short": 0,
    "my_ushort": 0,
    "my_longlong": 0,
    "my_ulonglong": 0,
    "my_float": 0.0,
    "my_double": 0.0,
    "my_longdouble": 0.0
}
```

<a id="xtypes-serialization-utilities-string"></a>

<a id="strings"></a>

#### Strings

String types are used to represent sequences of characters, which are essential for handling textual
data within the system.
The following example shows the definition of string types in IDL:

```omg-idl
struct StringsStruct
{
    string my_string;
    wstring my_wstring;
    string<41925> my_bounded_string;
    wstring<20925> my_bounded_wstring;
};
```

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "my_string": "",
    "my_wstring": "",
    "my_bounded_string": "",
    "my_bounded_wstring": ""
}
```

<a id="xtypes-serialization-utilities-enumeration"></a>

<a id="enumerations"></a>

#### Enumerations

Enumeration types represent a fixed set of named values, making it easier to work with a predefined
list of options.
Below is an example of the definition of enumeration types in IDL:

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

The previous DynamicData object can be serialized in two different formats, eProsima and OMG, providing
flexibility depending on the required interoperability and compatibility with other systems.
The previous [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object would be serialized as follows in the different formats:

### eProsima

```json
{
    "my_enum": {
        "name": "A",
        "value": 0
    }
}
```

### OMG

```json
{
    "my_enum": "A"
}
```

<a id="xtypes-serialization-utilities-bitmask"></a>

<a id="bitmasks"></a>

#### Bitmasks

Bitmask types allow the representation of a set of flags in a single value, which can be useful
for storing multiple boolean options compactly.
Here is an example of the definition of bitmask types in IDL:

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

Bitmask also present different serialized structures in the different formats eProsima and OMG.
The previous [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object would be serialized as follows in the different formats:

### eProsima

```json
{
    "my_bitmask": {
        "active": [],
        "binary": "00000000",
        "value": 0
    }
}
```

### OMG

```json
{
    "my_bitmask": 0
}
```

<a id="xtypes-serialization-utilities-sequence"></a>

<a id="sequences"></a>

#### Sequences

Sequence types are used to represent ordered collections of elements, similar to arrays but with
dynamic length.
Below is an example of the definition of sequence types in IDL:

```omg-idl
struct SequenceStruct
{
    sequence<MyBitMask> bitmask_sequence;
    sequence<short, 5> short_sequence;
};
```

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "bitmask_sequence": [],
    "short_sequence": []
}
```

<a id="xtypes-serialization-utilities-array"></a>

<a id="arrays"></a>

#### Arrays

Array types are used to represent fixed-size collections of elements.
The following example shows the definition of array types in IDL:

```omg-idl
struct ArrayStruct
{
    long long_array[2][3];
};
```

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "long_array": [
        [
            0,
            0,
            0
        ],
        [
            0,
            0,
            0
        ]
    ]
}
```

<a id="xtypes-serialization-utilities-map"></a>

<a id="maps"></a>

#### Maps

Map types represent collections of key-value pairs, allowing for the efficient lookup of values based
on unique keys.
Below is an example of the definition of map types in IDL:

```omg-idl
struct MapStruct
{
    map<string, MyAliasedBoundedString> string_alias_unbounded_map;
    map<short, long, 2> short_long_map;
};
```

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "short_long_map": null,
    "string_alias_unbounded_map": null
}
```

<a id="xtypes-serialization-utilities-structure"></a>

<a id="structures"></a>

#### Structures

Structure types are used to group different types of data together.
Here is an example of the definition of structure types in IDL:

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

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "complex_member": {
        "first": 0
    },
    "first": 0.0,
    "second": 0
}
```

<a id="xtypes-serialization-utilities-union"></a>

<a id="unions"></a>

#### Unions

Union types are a special type of structure type where only one member exists.
Below is an example of the definition of union types in IDL:

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

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "inner_union": {
        "second": 0
    },
    "complex_union": {
        "fourth": {
            "second": 0
        }
    }
}
```

<a id="xtypes-serialization-utilities-bitsets"></a>

<a id="bitsets"></a>

#### Bitsets

Bitset types are a specialized type of structure where individual bits can be accessed and manipulated.
Below is an example of the definition of bitset types in IDL:

```omg-idl
bitset MyBitSet
{
    bitfield<3> a;
    bitfield<1> b;
    bitfield<4>;
    bitfield<10> c;
    bitfield<12, short> d;
};

struct BitsetStruct
{
   MyBitSet my_bitset;
};
```

A [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object corresponding to the type represented above
would be serialized as follows:

```json
{
    "my_bitset": {
        "a": 0,
        "b": 0,
        "c": 0,
        "d": 0
    }
}
```

<a id="xtypes-serialization-utilities-dyndata-json-serialize-options"></a>

<a id="json-serialize-options-pro"></a>

### JSON serialize options **[Pro]**

The `JsonSerializeOptions` struct controls how [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) is serialized
to JSON.
It is passed to [`json_serialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds14json_serializeERKN11DynamicData9_ref_typeE21DynamicDataJsonFormatRNSt7ostreamE) and exposes the following fields:

| Field                  | Default    | Description                                                                                                                                                                                                                                  |
|------------------------|------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `mapping`              | `EPROSIMA` | Selects the JSON mapping. Accepted values are `DynamicDataJsonMapping::EPROSIMA`<br/>and `DynamicDataJsonMapping::OMG`.                                                                                                                      |
| `max_collection_items` | `0`        | Maximum number of elements to emit per sequence or array. A value of `0` means no limit<br/>and reproduces the legacy behavior. When the limit is exceeded, the collection is replaced<br/>with a compact label instead of its element list. |

<a id="collection-truncation-pro"></a>

#### Collection truncation **[Pro]**

When `max_collection_items` is set to a non-zero value, any sequence or array whose element count
exceeds the limit is replaced by a compact label that describes the collection instead of listing its
elements:

* A sequence emits `"<sequence: N <element-type>>"`, where `N` is the actual number of elements and
  `<element-type>` is the IDL keyword of the element type (for example `"<sequence: 8 long>"`).
* An array emits `"<array: <element-type>[d1,d2,...]>"`, where `d1,d2,...` are the array bounds
  (for example `"<array: long[3,4]>"`). The bounds are part of the type, so the element count is not
  spliced in.

The element type is resolved through any alias, and nested collections are rendered using their IDL form
(for example `sequence<long>` or `long[2]`).

Truncation is applied independently at every nesting level, so an outer collection that fits within
the limit is still expanded while inner collections that exceed it are replaced by their label.
This feature is intended for display contexts where large collections would otherwise produce
unmanageably large JSON output.

<a id="xtypes-serialization-utilities-dyndata-json-example"></a>

<a id="example-convert-received-data-into-json-format"></a>

### Example: Convert received data into JSON format

The following code demonstrates how to use the [`json_serialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds14json_serializeERKN11DynamicData9_ref_typeE21DynamicDataJsonFormatRNSt7ostreamE) function in Fast DDS to serialize
received data into a more manageable and understandable JSON format.
Each time the subscriber receives new data, the corresponding [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) can be obtained from the
[`DynamicDataFactory`](../api_reference/dds_pim/xtypes/dynamicdatafactory.md#_CPPv4N8eprosima7fastdds3dds18DynamicDataFactoryE) and serialized into a JSON string format.

#### NOTE
**[Pro]** This example uses `JsonSerializeOptions`, which is only available in *Fast DDS* **[Pro]**.
In *Fast DDS*, pass a `DynamicDataJsonFormat` value (e.g. `DynamicDataJsonFormat::EPROSIMA`) instead.

Please refer to [Remote type discovery and endpoint matching](../use_cases/remote_type_discovery_matching/remote_type_discovery_matching.md#use-case-remote-type-discovery-and-matching) section for more details on how to implement
remote type discovery.

```c++
void on_data_available(
        DataReader* reader)
{
    // Create data using the DynamicType created from discovered TypeObject or through Dynamic Language Binding API
    DynamicData::_ref_type new_data =
            DynamicDataFactory::get_instance()->create_data(dyn_type_serialization_);

    // Configure serialization mapping and optional collection truncation
    JsonSerializeOptions options;
    options.mapping = DynamicDataJsonMapping::EPROSIMA;
    options.max_collection_items = 0;  // 0 disables truncation; set a positive value to cap collection size

    SampleInfo info;
    while ((RETCODE_OK == reader->take_next_sample(&new_data, &info)))
    {
        std::stringstream output;
        output << std::setw(4);

        // Serialize DynamicData into JSON string format
        if (RETCODE_OK != json_serialize(
                    new_data,
                    options,
                    output))
        {
            // Error
            return;
        }

        // Print JSON representation
        std::cout << "Message received:\n" << output.str() << std::endl;
    }
}

// DynamicType corresponding to received data
DynamicType::_ref_type dyn_type_serialization_;
```

<a id="xtypes-serialization-utilities-json-dyndata"></a>

<a id="json-to-dynamicdata"></a>

## JSON to DynamicData

Apart from having the possibility to serialize [DynamicData to JSON](#xtypes-serialization-utilities-dyndata-json),
Fast DDS also provides a way to perform the inverse conversion.
The method [`json_deserialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds16json_deserializeERKNSt6stringERKN11DynamicType9_ref_typeE21DynamicDataJsonFormatRN11DynamicData9_ref_typeE) is able to convert a JSON object into a [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) instance,
by only providing its associated [`DynamicType`](../api_reference/dds_pim/xtypes/dynamictype.md#_CPPv4N8eprosima7fastdds3dds11DynamicTypeE).
This method is useful for injecting data from external sources into a DDS network, allowing for the integration of data
from various systems and applications.

<a id="xtypes-serialization-utilities-dyndata-json-deserialize-options"></a>

<a id="json-deserialize-options-pro"></a>

### JSON deserialize options **[Pro]**

The `JsonDeserializeOptions` struct controls how [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) is deserialized from
JSON.
It is passed to [`json_deserialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds16json_deserializeERKNSt6stringERKN11DynamicType9_ref_typeE21DynamicDataJsonFormatRN11DynamicData9_ref_typeE) and exposes the following fields:

| Field     | Default    | Description                                                                                                             |
|-----------|------------|-------------------------------------------------------------------------------------------------------------------------|
| `mapping` | `EPROSIMA` | Selects the JSON mapping. Accepted values are `DynamicDataJsonMapping::EPROSIMA`<br/>and `DynamicDataJsonMapping::OMG`. |

<a id="xtypes-serialization-utilities-json-dyndata-example"></a>

<a id="example-convert-json-data-into-dynamicdata"></a>

### Example: Convert JSON data into DynamicData

The following code demonstrates how to use the [`json_deserialize`](../api_reference/dds_pim/xtypes/utils.md#_CPPv4N8eprosima7fastdds3dds16json_deserializeERKNSt6stringERKN11DynamicType9_ref_typeE21DynamicDataJsonFormatRN11DynamicData9_ref_typeE) function in Fast DDS to convert
JSON data into a [`DynamicData`](../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) object.

#### NOTE
**[Pro]** This example uses `JsonDeserializeOptions`, which is only available in *Fast DDS* **[Pro]**.
In *Fast DDS*, pass a `DynamicDataJsonFormat` value (e.g. `DynamicDataJsonFormat::EPROSIMA`) instead.

```c++
void json_to_data(
        const std::string& json_data)
{
    // Deserialize JSON string into DynamicData
    // The required DynamicType can be created from discovered TypeObject or through Dynamic Language Binding API
    JsonDeserializeOptions options;
    options.mapping = DynamicDataJsonMapping::EPROSIMA;
    DynamicData::_ref_type new_data;
    if (RETCODE_OK != json_deserialize(
                json_data,
                dyn_type_deserialization_,
                options,
                new_data))
    {
        // Error
        return;
    }

    // Process the new data
}

// DynamicType corresponding to JSON data to be deserialized into DynamicData
DynamicType::_ref_type dyn_type_deserialization_;
```
