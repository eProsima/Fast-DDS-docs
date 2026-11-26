<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="freq-xtypes-questions"></a>

<a id="xtypes-frequently-asked-questions"></a>

# XTypes Frequently Asked Questions

**What are XTypes?**

<br/> Extensible and Dynamic Topic Types for DDS specification.
This specification defines the following concepts: DDS supported type system; type representation, including IDL and [TypeObject representation](../../xtypes/discovery_matching.md#xtypes-discovery-matching) representations; data representation over the wire; language binding; DDS builtin mechanism to automatically discover remote data types.
For more information,  refer to [XTypes](../../xtypes/xtypes.md#dynamic-types)

---

**How does the remote data type discovery mechanism work?**

<br/> The remote data type discovery mechanism is based on the exchange of the data type information optimized in order to reduce the required bandwidth.
For more information, refer to [Remote Data Types Discovery](../../xtypes/discovery_matching.md#xtypes-discovery-matching).

---

**What prerequisites are required for the remote data type discovery feature to work?**

<br/> The local data types must be registered into the [`ITypeObjectRegistry`](../../api_reference/dds_pim/xtypes/typeobjectregistry.md#_CPPv4N8eprosima7fastdds3dds6xtypes19ITypeObjectRegistryE).
[TypeInformation](../../xtypes/discovery_matching.md#xtypes-discovery-matching) should be received with the DomainParticipant’s endpoint discovery information.
A [DomainParticipant](../../dds_layer/domain/domainParticipant/domainParticipant.md#dds-layer-domainparticipant) that does not inform about its [TypeInformation](../../xtypes/discovery_matching.md#xtypes-discovery-matching) would not trigger the remote data type discovery mechanism.
For more information, refer to [Remote Data Types Discovery](../../xtypes/discovery_matching.md#xtypes-discovery-matching).

---

**What is the purpose of the Dynamic Language Binding API?**

<br/> The Dynamic Language Binding API allows to define data types at runtime instead of having the types predefined as it is required by the Plain Language Binding.
For further information, refer to [Dynamic Language Binding](../../xtypes/language_binding.md#xtypes-language-binding).

---

**How can complex data types be managed?**

<br/> Dynamic Language Binding provides two possible approaches for managing complex data types: [`DynamicData::get_complex_value`](../../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17get_complex_valueERN6traitsI11DynamicDataE8ref_typeE8MemberId) and [`DynamicData::set_complex_value`](../../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData17set_complex_valueE8MemberIdN6traitsI11DynamicDataE8ref_typeE); [`DynamicData::loan_value`](../../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicData10loan_valueE8MemberId), which allows to loan a reference to a [`DynamicData`](../../api_reference/dds_pim/xtypes/dynamicdata.md#_CPPv4N8eprosima7fastdds3dds11DynamicDataE) to work with preventing the data copy.
For further information, refer to [Dynamic Language Binding](../../xtypes/language_binding.md#xtypes-language-binding).

---

**What is serialization?**

<br/> Serialization is a crucial process in data distribution services, as it converts complex data structures into a format that can be easily transmitted and reconstructed across different platforms and programming environments.
For further information, refer to [Serialization Utilities](../../xtypes/type_serializing.md#xtypes-serialization-utilities).

---

**What are the formats supported for the serialization?**

<br/> A Dynamic Type can be serialized to its IDL representation and to JSON (primitives, strings, enumerations, bitmasks, sequences, arrays, maps, structures, unions, bitsets).
For further information, refer to [Serialization Utilities](../../xtypes/type_serializing.md#xtypes-serialization-utilities).

<br/>
