<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="dynamic-types-idl-parsing"></a>

<a id="id1"></a>

# Dynamic Types IDL Parsing

*Fast DDS* supports the implementation of [Dynamic Language Binding](language_binding.md#xtypes-language-binding) for parsing IDL files at runtime to generate dynamic
data types.

This feature enables applications to dynamically load type definitions from IDL files instead of relying solely on
pre-generated types or XML profiles.
This enhances flexibility, as new data types can be introduced without modifying and recompiling the application.

<a id="supported-types"></a>

## Supported Types

Below, the types supported by *eProsima Fast DDS* IDL parsing are presented.
For further information about the supported [Dynamic Language Binding](language_binding.md#xtypes-language-binding), please, refer to [Supported Types](language_binding.md#xtypes-supportedtypes).

* [Primitive Types](language_binding.md#xtypes-supportedtypes-primitive)
* [String Types](language_binding.md#xtypes-supportedtypes-string)
* [Structure Types](language_binding.md#xtypes-supportedtypes-structure)
* [Alias Types](language_binding.md#xtypes-supportedtypes-alias)
* [Array types](language_binding.md#xtypes-supportedtypes-array)
* [Sequence types](language_binding.md#xtypes-supportedtypes-sequence)
* [Union Types](language_binding.md#xtypes-supportedtypes-union)
* [Enumeration Types](language_binding.md#xtypes-supportedtypes-enumeration)
* [Bitmask Types](language_binding.md#xtypes-supportedtypes-bitmask) **[Pro]**
* [Bitset Types](language_binding.md#xtypes-supportedtypes-bitset) **[Pro]**
* Arithmetic expressions
* Union/struct forward declarations
* Modules/scoped types
* [Builtin annotations](language_binding.md#xtypes-builtin-annotations)
* Member ID **[Pro]**
* [Inheritance](language_binding.md#xtypes-supportedtypes-inheritance) **[Pro]**

The following types are currently not supported by the IDL parsing feature:

* [Map Types](language_binding.md#xtypes-supportedtypes-map)
* [Custom annotations](language_binding.md#xtypes-custom-annotations)

<a id="create-a-dynamic-type-from-a-idl-file"></a>

## Create a Dynamic Type from a IDL file

*Fast DDS* application can use dynamic types generated in runtime just by loading the corresponding IDL files into the
[`DynamicTypeBuilderFactory`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactoryE) using [`DynamicTypeBuilderFactory::create_type_w_uri`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory17create_type_w_uriERKNSt6stringERKNSt6stringERK14IncludePathSeq).
After getting the DynamicType, objects of [`DynamicPubSubType`](../api_reference/dds_pim/xtypes/dynamicpubsubtype.md#_CPPv4N8eprosima7fastdds3dds17DynamicPubSubTypeE) class might be instantiated and used to write/read
data.

#### WARNING
[`DynamicTypeBuilderFactory::create_type_w_uri`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory17create_type_w_uriERKNSt6stringERKNSt6stringERK14IncludePathSeq) requires the fully qualified name of the
type defined in the IDL file. If no scope is provided, global scope is assumed.

#### NOTE
The preprocessor can be manually selected using [`DynamicTypeBuilderFactory::set_preprocessor`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory16set_preprocessorERKNSt6stringE)

<a id="example"></a>

### Example

The following snippet shows the previously explained steps:

```c++
// Create a DomainParticipant in the desired domain
DomainParticipant* participant =
        DomainParticipantFactory::get_instance()->create_participant(0, PARTICIPANT_QOS_DEFAULT);
if (nullptr == participant)
{
    // Error
    return;
}

// Optional: Set the preprocessor to be executed before parsing the IDL
DynamicTypeBuilderFactory::get_instance()->set_preprocessor("<optional_path_to_your_preprocessor_executable>");

// Load the IDL file
std::string idl_file = "<path_to_idl>.idl";
std::string type_name = "YourType";
std::vector<std::string> include_paths;
include_paths.push_back("<path/to/folder/containing/included/idl/files>");

// Retrieve the instance of the desired type
DynamicTypeBuilder::_ref_type dyn_type_builder =
        DynamicTypeBuilderFactory::get_instance()->create_type_w_uri(idl_file, type_name, include_paths);

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

<a id="iterate-over-the-parsed-idl-types"></a>

## Iterate over the parsed IDL types

*Fast DDS* application supports a simple way to iterate over each aggregated type declared and parsed in the IDL file,
*i.e*:

* [Union Types](language_binding.md#xtypes-supportedtypes-union)
* [Enumeration Types](language_binding.md#xtypes-supportedtypes-enumeration)
* [Structure Types](language_binding.md#xtypes-supportedtypes-structure)
* [Alias Types](language_binding.md#xtypes-supportedtypes-alias)

using [`DynamicTypeBuilderFactory::for_each_type_w_uri`](../api_reference/dds_pim/xtypes/dynamictypebuilderfactory.md#_CPPv4N8eprosima7fastdds3dds25DynamicTypeBuilderFactory19for_each_type_w_uriERKNSt6stringERK14IncludePathSeqNSt8functionIFbN6traitsI18DynamicTypeBuilderE8ref_typeEEEE) method.
User can customize the runtime behavior by providing a lambda expression, which will be triggered
when a new aggregated type is parsed.

The callback provided by the user receives the [`DynamicTypeBuilder`](../api_reference/dds_pim/xtypes/dynamictypebuilder.md#_CPPv4N8eprosima7fastdds3dds18DynamicTypeBuilderE) instance related to the parsed type
and returns a boolean value, indicating whether the iteration should continue or not.
In case of returning false for a given type,
the parsing process will stop and no further types will be processed.

<a id="id2"></a>

### Example

The following snippet shows how to iterate over the aggregated types declared in the IDL file,
storing their names in a vector:

```c++
// Optional: Set the preprocessor to be executed before parsing the IDL
DynamicTypeBuilderFactory::get_instance()->set_preprocessor("<optional_path_to_your_preprocessor_executable>");

// Load the IDL file
std::string idl_file = "<path_to_idl>.idl";
std::vector<std::string> include_paths;
include_paths.push_back("<path/to/folder/containing/included/idl/files>");

// Vector to store the names of the parsed types
std::vector<std::string> parsed_types;

// Define the function callback to be executed for each aggregated type parsed in the IDL file
auto callback = [&parsed_types](traits<DynamicTypeBuilder>::ref_type builder)
        {
            if (builder)
            {
                parsed_types.emplace_back(builder->get_name().to_string());
            }

            return true; // continue parsing
        };

// Store the names of the parsed types in the vector
DynamicTypeBuilderFactory::get_instance()->for_each_type_w_uri(idl_file, include_paths, callback);

```
