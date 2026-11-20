<a id="dynamic-types"></a>

<a id="xtypes"></a>

# XTypes

*eProsima Fast DDS* supports the OMG [Extensible and Dynamic Topic Types for DDS](https://www.omg.org/spec/DDS-XTypes/1.3) specification (also known as XTypes).
This specification defines the following concepts:

* DDS supported type system, including the concept of extensible types that might evolve in time.
* Type representation, including [IDL](../../fastddsgen/dataTypes/dataTypes.md#fastddsgen-idl-datatypes) and [TypeObject](discovery_matching.md#xtypes-discovery-matching)
  representations.
* Data representation over the wire.
* Language binding, defining both a plain and a dynamic language binding.
  [eProsima Fast DDS-Gen](../../fastddsgen/introduction/introduction.md#fastddsgen-intro) generates the plain language binding given an IDL type representation.
  [Dynamic Language Binding](language_binding.md#xtypes-language-binding) section explains the required API to define and set/read the data types.
* DDS builtin mechanism to automatically discover remote data types.
  More information in [Remote Data Types Discovery](discovery_matching.md#xtypes-discovery-matching) section.

* [Remote Data Types Discovery](discovery_matching.md)
  * [Prerequisites](discovery_matching.md#prerequisites)
  * [Configuration](discovery_matching.md#configuration)
  * [Remote types discovery example](discovery_matching.md#remote-types-discovery-example)
* [Dynamic Language Binding](language_binding.md)
  * [Dynamic Language Binding Interfaces](language_binding.md#dynamic-language-binding-interfaces)
    * [TypeDescriptor](language_binding.md#typedescriptor)
    * [AnnotationDescriptor](language_binding.md#annotationdescriptor)
    * [MemberDescriptor](language_binding.md#memberdescriptor)
    * [VerbatimTextDescriptor](language_binding.md#verbatimtextdescriptor)
    * [DynamicTypeBuilderFactory](language_binding.md#dynamictypebuilderfactory)
    * [DynamicType](language_binding.md#dynamictype)
    * [DynamicTypeMember](language_binding.md#dynamictypemember)
    * [DynamicTypeBuilder](language_binding.md#dynamictypebuilder)
    * [DynamicDataFactory](language_binding.md#dynamicdatafactory)
    * [DynamicData](language_binding.md#dynamicdata)
  * [Supported Types](language_binding.md#supported-types)
    * [Primitive Types](language_binding.md#primitive-types)
    * [String Types](language_binding.md#string-types)
    * [Enumeration Types](language_binding.md#enumeration-types)
    * [Alias Types](language_binding.md#alias-types)
    * [Sequence types](language_binding.md#sequence-types)
    * [Array types](language_binding.md#array-types)
    * [Map Types](language_binding.md#map-types)
    * [Structure Types](language_binding.md#structure-types)
    * [Union Types](language_binding.md#union-types)
    * [Bitmask Types](language_binding.md#bitmask-types)
    * [Bitset Types](language_binding.md#bitset-types)
    * [Annotations](language_binding.md#annotations)
  * [Managing Complex Types Data](language_binding.md#managing-complex-types-data)
* [Serialization Utilities](type_serializing.md)
  * [Dynamic Type to IDL](type_serializing.md#dynamic-type-to-idl)
    * [Example: Convert a discovered type to IDL format](type_serializing.md#example-convert-a-discovered-type-to-idl-format)
  * [DynamicData to JSON](type_serializing.md#dynamicdata-to-json)
    * [Supported Types](type_serializing.md#supported-types)
    * [JSON serialize options <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](type_serializing.md#json-serialize-options-pro)
    * [Example: Convert received data into JSON format](type_serializing.md#example-convert-received-data-into-json-format)
  * [JSON to DynamicData](type_serializing.md#json-to-dynamicdata)
    * [JSON deserialize options <span class="sd-badge sd-outline-primary sd-text-primary" title="Exclusive to Fast DDS Pro">Pro</span>](type_serializing.md#json-deserialize-options-pro)
    * [Example: Convert JSON data into DynamicData](type_serializing.md#example-convert-json-data-into-dynamicdata)
* [Dynamic Types IDL Parsing](idl_parsing.md)
  * [Supported Types](idl_parsing.md#supported-types)
  * [Create a Dynamic Type from a IDL file](idl_parsing.md#create-a-dynamic-type-from-a-idl-file)
    * [Example](idl_parsing.md#example)
  * [Iterate over the parsed IDL types](idl_parsing.md#iterate-over-the-parsed-idl-types)
    * [Example](idl_parsing.md#id2)

#### NOTE
Type compatibility rules among evolved types are still unsupported in Fast DDS.
