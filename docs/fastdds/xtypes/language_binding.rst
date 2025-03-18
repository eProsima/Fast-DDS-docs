.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _xtypes_language_binding:

Dynamic Language Binding
========================

The Dynamic Language Binding API allows to define data types at runtime instead of having the types predefined as it is
required by the Plain Language Binding.
This API includes both the type definition and, the getters and setters required to use the defined types.
Type definition can also be done using a XML configuration file as explained in :ref:`xmldynamictypes` section or by
parsing an IDL file at runtime, as explained in :ref:`dynamic-types-idl-parsing` section.

This section presents first the Dynamic Language Binding API, and then the supported types and specific examples
defining and using those types.

.. _xtypes_interfaces:

Dynamic Language Binding Interfaces
-----------------------------------

This section briefly presents the Dynamic Language Binding API.
For more information, please refer both to the `DDS-XTypes specification <https://www.omg.org/spec/DDS-XTypes/1.3>`__
and the :ref:`API reference<dynamic_language_binding_api>`.

TypeDescriptor
^^^^^^^^^^^^^^

|TypeDescriptor-api| is in charge of describing the state of a type.
Objects of this interface have value semantics allowing the TypeDescriptor data to be deeply copied and compared.

AnnotationDescriptor
^^^^^^^^^^^^^^^^^^^^

|AnnotationDescriptor-api| is in charge of describing the user-defined applied annotation to a specific element.
Objects of this interface have value semantics allowing the AnnotationDescriptor data to be deeply copied and compared.

MemberDescriptor
^^^^^^^^^^^^^^^^

|MemberDescriptor-api| is in charge of describing the state of a specific member of a type.
Objects of this interface have value semantics allowing the MemberDescriptor data to be deeply copied and compared.

VerbatimTextDescriptor
^^^^^^^^^^^^^^^^^^^^^^

|VerbatimTextDescriptor-api| is in charge of describing the :code:`@verbatim` builtin annotation application.
Objects of this interface have value semantics allowing the VerbatimTextDescriptor data to be deeply copied and
compared.

DynamicTypeBuilderFactory
^^^^^^^^^^^^^^^^^^^^^^^^^

The |DynamicTypeBuilderFactory-api| serves as a singleton which instance is responsible for both creating and deleting
|DynamicTypeBuilder-api| objects.
This class provides the generic |DynamicTypeBuilderFactory::create_type| API, and also specific APIs to define other
basic types such as strings, sequences, etc.
More information can be found in :ref:`xtypes_supportedtypes` section.

DynamicType
^^^^^^^^^^^

|DynamicType-api| objects represents a specific type definition.
Once the DynamicType has been built, it cannot be modified.
Objects of this interface have reference semantics, so the API receives a nil-reference which is then returned pointing
to the correct DynamicType address.

DynamicTypeMember
^^^^^^^^^^^^^^^^^

|DynamicTypeMember-api| represents a data member of a DynamicType.
Objects of this interface have reference semantics, so the API receives a nil-reference which is then returned pointing
to the correct DynamicTypeMember address.

DynamicTypeBuilder
^^^^^^^^^^^^^^^^^^

|DynamicTypeBuilder-api| interface allows the instantiation of concrete DynamicType objects and serves as a transitional
state for configuring the DynamicType before its creation.
Upon definition, DynamicTypeBuilderFactory leverages the information contained in the builder to create
the DynamicType.
|DynamicTypeBuilder::build| allows for creating the fully constructed DynamicType.
Builders remain reusable after DynamicType creation, ensuring changes to the builder do not affect previously
created types.

DynamicDataFactory
^^^^^^^^^^^^^^^^^^

|DynamicDataFactory-api| serves as a singleton which instance is responsible for both creating and deleting
|DynamicData-api| objects from a given DynamicType instance.

DynamicData
^^^^^^^^^^^

|DynamicData-api| represents a data instance of a DynamicType, providing functionalities to access
and modify data values.
Each DynamicData object corresponds to an object of the type represented by its DynamicType.
Offering reflective getters and setters, DynamicData enables manipulation of individual data samples.

.. _xtypes_supportedtypes:

Supported Types
---------------

This section describes the supported Type System including examples of how to instantiate those specific types using the
Dynamic Language Binding API and the XML configuration file.
The C++ examples also include instantiating the corresponding DynamicData sample, and setting and reading a value.

.. _xtypes_supportedtypes_primitive:

Primitive Types
^^^^^^^^^^^^^^^

Primitive types are self-describing and can be created without configuration parameters.
The |DynamicTypeBuilderFactory-api| interface exposes the method |DynamicTypeBuilderFactory::get_primitive_type|
to allow users to directly get the corresponding primitive DynamicType.
The |DynamicData-api| class provides specific getters and setters for each primitive data type.

The following table shows the supported primitive types and their corresponding :code:`TypeKind`.
The :code:`TypeKind` is used to query the DynamicTypeBuilderFactory for the specific primitive DynamicType.

.. list-table::
    :header-rows: 1
    :align: left

    * - C++ Type
      - TypeKind
    * - :code:`bool`
      - :code:`TK_BOOLEAN`
    * - :code:`char`
      - :code:`TK_CHAR8`
    * - :code:`wchar_t`
      - :code:`TK_CHAR16`
    * - :code:`uint8_t`
      - :code:`TK_BYTE` / :code:`TK_UINT8`
    * - :code:`int8_t`
      - :code:`TK_INT8`
    * - :code:`int16_t`
      - :code:`TK_INT16`
    * - :code:`uint16_t`
      - :code:`TK_UINT16`
    * - :code:`int32_t`
      - :code:`TK_INT32`
    * - :code:`uint32_t`
      - :code:`TK_UINT32`
    * - :code:`int64_t`
      - :code:`TK_INT64`
    * - :code:`uint64_t`
      - :code:`TK_UINT64`
    * - :code:`float`
      - :code:`TK_FLOAT32`
    * - :code:`double`
      - :code:`TK_FLOAT64`
    * - :code:`long double`
      - :code:`TK_FLOAT128`

The example below shows how to create an structure with primitive members.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_PRIMITIVES
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_PRIMITIVES<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_PRIMITIVES
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type,
please refer to :ref:`XML Primitive Types <xmldynamictypes_primivites>`.

Type promotions
"""""""""""""""

The Dynamic Language Binding also supports type promotion, enabling implicit promotion of types during both :func:`get`
and :func:`set` operations.
This means that a smaller type can be implicitly promoted to a larger type, but not the other way around.

The following promotions are supported:

.. list-table::
    :header-rows: 1
    :align: left

    * - TypeKind
      - Allowed promotions
    * - :code:`TK_INT8`
      - :code:`TK_INT16`, :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_FLOAT32`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_INT16`
      - :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_FLOAT32`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_INT32`
      - :code:`TK_INT64`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_INT64`
      - :code:`TK_FLOAT128`
    * - :code:`TK_UINT8`
      - :code:`TK_INT16`, :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_UINT16`, :code:`TK_UINT32`, :code:`TK_UINT64`,
        :code:`TK_FLOAT32`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_UINT16`
      - :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_UINT32`, :code:`TK_UINT64`, :code:`TK_FLOAT32`, :code:`TK_FLOAT64`,
        :code:`TK_FLOAT128`
    * - :code:`TK_UINT32`
      - :code:`TK_INT64`, :code:`TK_UINT64`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_UINT64`
      - :code:`TK_FLOAT128`
    * - :code:`TK_FLOAT32`
      - :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_FLOAT64`
      - :code:`TK_FLOAT128`
    * - :code:`TK_FLOAT128`
      - (none)
    * - :code:`TK_CHAR8`
      - :code:`TK_CHAR16`, :code:`TK_INT16`, :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_FLOAT32`, :code:`TK_FLOAT64`,
        :code:`TK_FLOAT128`
    * - :code:`TK_CHAR16`
      - :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_FLOAT32`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`
    * - :code:`TK_BYTE`
      - (any)
    * - :code:`TK_BOOLEAN`
      - :code:`TK_INT8`, :code:`TK_INT16`, :code:`TK_INT32`, :code:`TK_INT64`, :code:`TK_UINT8`, :code:`TK_UINT16`,
        :code:`TK_UINT32`, :code:`TK_UINT64`, :code:`TK_FLOAT32`, :code:`TK_FLOAT64`, :code:`TK_FLOAT128`


.. _xtypes_supportedtypes_string:

String Types
^^^^^^^^^^^^

String types are one-dimensional collections of characters (:code:`TK_CHAR8` or :code:`TK_CHAR16`.
The latest are also known as wide strings or :code:`wstring`).
The :code:`TypeKinds` used to identify string types are :code:`TK_STRING8` and :code:`TK_STRING16`.
The string may be bounded, setting a maximum length, or unbounded.
This is configured using |TypeDescriptor-api| :code:`bound` property.

|DynamicTypeBuilderFactory-api| exposes the functions |DynamicTypeBuilderFactory::create_string_type| and
|DynamicTypeBuilderFactory::create_wstring_type| that eases string creation providing the corresponding maximum length
parameter (:code:`LENGTH_UNLIMITED` is used for unbounded strings).

|DynamicData-api| class provides also with specific getters and setters: |DynamicData::get_string_value|,
|DynamicData::get_wstring_value|, |DynamicData::set_string_value|, and |DynamicData::set_wstring_value|.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_STRINGS
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_STRINGS<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_STRINGS
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML String Types <xmldynamictypes_strings>`.


.. _xtypes_supportedtypes_enumeration:

Enumeration Types
^^^^^^^^^^^^^^^^^

An enumeration contains a set of supported values (enumeration literals) and a selected value among those supported.
The :code:`TypeKind` used to identify enumeration types is :code:`TK_ENUM`.

The enumeration literals must be configured using the |DynamicTypeBuilder-api| by calling the
|DynamicTypeBuilder::add_member| function for the respective supported values.
The |MemberDescriptor-api| passed to the previous function must determine the enumeration literal name by using
:code:`name` property.
The underlying primitive type related to the enumeration is configured using |MemberDescriptor-api| :code:`type`
property.
This primitive type is determined when adding the first enumeration literal.
For the enumeration type to be consistent, the remaining enumeration literals must be of the same primitive type.
Additionally, the enumeration literal value might be set using |MemberDescriptor-api| :code:`default_value` property.
The behavior is the same as setting the :code:`@value` :ref:`builtin annotation<builtin_annotations>`.

As the enumeration type is basically a signed integer type which might take only some specific values defined with the
enumeration literals, the corresponding DynamicData getters and setters are the ones corresponding to the underlying
signed integer type (and any other method promotable to that specific primitive type).

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_ENUM
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_ENUM<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_ENUM
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Enumeration Types <xmldynamictypes_enums>`.


.. _xtypes_supportedtypes_bitmask:

Bitmask Types
^^^^^^^^^^^^^

Bitmask types are a collection of boolean flags (bitflags) that can be set individually.
The :code:`TypeKind` used to identify bitmask types is :code:`TK_BITMASK`.
The bitmasks bound, maximum number of bits, must be set using the |TypeDescriptor-api| :code:`bound` property.
The maximum bound allowed is 64 bits.

The bitflags must be configured using the |DynamicTypeBuilder-api| by calling the |DynamicTypeBuilder::add_member|
function.
Each bitflag is described using a |MemberDescriptor-api| defining the bitflag name using the :code:`name` property.
The underlying primitive type related to bitflags must be of kind :code:`TK_BOOLEAN` and must be set in
|MemberDescriptor-api| :code:`type` property.
The |MemberDescriptor-api| :code:`id` property might be used to indicate the bitflag position within the bitmask.
This behavior is the same as setting the :code:`@position` :ref:`builtin annotation<builtin_annotations>`.
If the position is not specified, sequential order is followed.

The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_bitmask_type| to
facilitate the creation of bitmask types.

Bitmask types can be manipulated either using the |DynamicData::get_boolean_value|/|DynamicData::set_boolean_value| in
order to set a specific bitflag, or by using the unsigned integer setter/getter corresponding to the bitmask bound.
In this latest case, only bitflags are going to be set (bits not named are always unset).

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_BITMASK
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_BITMASK<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_BITMASK
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Bitmask Types<xmldynamictypes_bitmask>`.


.. _xtypes_supportedtypes_alias:

Alias Types
^^^^^^^^^^^

Alias types provide an alternative name to an already existing type.
The :code:`TypeKind` used to identify aliases is :code:`TK_ALIAS`.
Besides defining the alias name, the underlying type must be set using |TypeDescriptor-api| :code:`base_type` property.
Alias recursion is supported by defining another alias type as the base type.

Once the |DynamicData-api| is created, information can be accessed as if working with the base type.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_TYPEDEF
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_TYPEDEF<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_TYPEDEF
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Alias Types<xmldynamictypes_typedef>`.

.. _xtypes_supportedtypes_sequence:

Sequence types
^^^^^^^^^^^^^^

Sequence types are one-dimensional collections of any type.
The :code:`TypeKind` used to identify sequences is :code:`TK_SEQUENCE`.
|TypeDescriptor-api| :code:`element_type` property must be set with the collection's type.
Additionally, :code:`bound` property must also be configured with the sequence's maximum length, or
:code:`LENGTH_UNLIMITED` in case of unbounded sequences.

|DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_sequence_type| to
facilitate the creation of this type.
This API requires the type stored in the collection and the collection's bound, using :code:`LENGTH_UNLIMITED` in case
of unbounded sequences.

|DynamicData-api| class provides specific :func:`get_values` and :func:`set_values` functions for each primitive
type, allowing users to easily work with sequences of primitive types.
Primitive type promotion is also applicable for these methods.
For sequences of more complex types, please refer to :ref:`xtypes_complextypes`.

If a specific range of values within the sequence are to be modified, passing the starting index to
:func:`get_values` / :func:`set_values` would only manage data from that element forward until the length of the given
input.
Specific collection's element can be also be modified using the :func:`get_value` / :func:`set_value` passing the index
of the element to be modified.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_SEQUENCES
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_SEQUENCES<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_SEQUENCES
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Sequence Types <xmldynamictypes_sequence>`.

.. _xtypes_supportedtypes_array:

Array types
^^^^^^^^^^^

Array types are multi-dimensional collections of any type.
The :code:`TypeKind` used to identify arrays is :code:`TK_ARRAY`.
|TypeDescriptor-api| :code:`element_type` property must be set with the collection's type.
Additionally, :code:`bound` property must be configured with the sequence containing the size of each dimension.
Bound sequence must have at least one dimension and it is not allowed for any dimension to have size :code:`0`.

|DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_array_type| to
facilitate the creation of this type.
This API requires the type stored in the collection and the sequence with the collection's dimensions.

|DynamicData-api| class provides specific :func:`get_values` and :func:`set_values` functions for each primitive
type, allowing users to easily work with arrays of primitives types.
For arrays of more complex types, please refer to :ref:`xtypes_complextypes`.

.. note::

    Multi-dimensional arrays *flatten* every dimension into a single-dimension array.

Primitive type promotion is also applicable for these methods.

If a specific range of values within the array are to be modified, passing the starting index to
:func:`get_values` / :func:`set_values` would only manage data from that element forward until the length of the given
input.
Specific collection's element can be also be modified using the :func:`get_value` / :func:`set_value` passing the index
of the element to be modified.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_ARRAYS
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_ARRAYS<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_ARRAYS
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Array Types<xmldynamictypes_array>`.

.. _xtypes_supportedtypes_map:

Map Types
^^^^^^^^^

Map types are a collection of key/value pair types.
Access to the value element is done through the key which is unique within the map type.
The :code:`TypeKind` used to identify maps is :code:`TK_MAP`.
|TypeDescriptor-api| :code:`element_type` property must be set with the map value type.
|TypeDescriptor-api| :code:`key_type` property must be set with the map key type.
Allowed key types are signed and unsigned integer types and string types.

.. note::

  Currently, wide string keys are not supported as map keys.

Additionally, :code:`bound` property must also be configured with the map's maximum length, or :code:`LENGTH_UNLIMITED`
in case of unbounded maps.

|DynamicTypeBuilderFactory-api| exposes the |DynamicTypeBuilderFactory::create_map_type| function to
facilitate the creation of this type.
This API requires the type of both the key and the value stored in the collection, and the collection's bound, using
:code:`LENGTH_UNLIMITED` in case of unbounded maps.

Manipulating map types data is more complex.
First the :code:`MemberId` corresponding to a specific :code:`key` must be retrieved using
|DynamicData::get_member_id_by_name| API.
This API either returns the :code:`MemberId` corresponding to the existing :code:`key` or, if the :code:`key` does not
exist yet, it creates the :code:`key` and returns the :code:`memberId` associated to the just created :code:`key`.
In order to call this method, the correct :code:`string` representation of the key value must be passed.
The map value can now be set using the API corresponding to the map value type.
For complex map values, please refer to :ref:`xtypes_complextypes`.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_MAPS
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_MAPS<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_MAPS
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Map Types<xmldynamictypes_map>`.

.. _xtypes_supportedtypes_structure:

Structure Types
^^^^^^^^^^^^^^^

Structure types are an aggregation of members of different types.
The :code:`TypeKind` used to identify structures is :code:`TK_STRUCTURE`.
Structure types have single inheritance support, so a structure type might extend one other already defined structure.
The structure type which is extended should be configured in the |TypeDescriptor-api| :code:`base_type` property.
:ref:`Structure extensibility<extensibility>` might be configured using |TypeDescriptor-api| :code:`extensibility_kind`
property.

.. note::

  Currently, :code:`@nested` builtin annotation is not supported.

Structure members must be configured using the |DynamicTypeBuilder-api| by calling the |DynamicTypeBuilder::add_member|
function with the corresponding |MemberDescriptor-api|.

.. note::

  Empty structures, with no members, are allowed.

Member name is configured using |MemberDescriptor-api| :code:`name` property and the member type is set using
:code:`type` property.
Structure members might be keyed to create :ref:`topic instances<dds_layer_topic_instances>` by setting the
|MemberDescriptor-api| :code:`is_key` property.
The behavior is the same as setting the :code:`@key` :ref:`builtin annotation<builtin_annotations>`.
Additionally, |MemberDescriptor-api| :code:`default_value` property might be set to configure the member default value,
and |MemberDescriptor-api| :code:`id` property sets explicitly the member ID.
This behavior is the same as setting the :code:`@default` and :code:`@id`
:ref:`builtin annotations<builtin_annotations>`.

.. note::

  Currently, Fast DDS-Gen does not support :code:`@default` builtin annotation.

.. note::

  Currently, Dynamic Language Binding API implementation does not support the following builtin annotations:

  * :code:`@optional`
  * :code:`@must_understand`
  * :code:`@external`
  * :code:`@try_construct`

Member data can be managed using the corresponding accessors for the underlying member type.
Member ID might be retrieved using |DynamicData::get_member_id_by_name| API.
For managing complex type members, please refer to :ref:`xtypes_complextypes`.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_STRUCT
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_STRUCT<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_STRUCT
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Structure Types<xmldynamictypes_struct>`.

.. _xtypes_supportedtypes_union:

Union Types
^^^^^^^^^^^

Union types are a special type of structure type where only one member exists.
The :code:`TypeKind` used to identify unions is :code:`TK_UNION`.
Member selection is performed by setting another special member called discriminator.
The discriminator type must be defined using |TypeDescriptor-api| :code:`discriminator_type` property.
Supported discriminator :code:`TypeKind` are the following:

* :code:`TK_BOOLEAN`
* :code:`TK_BYTE`
* :code:`TK_CHAR8`
* :code:`TK_CHAR16`
* :code:`TK_INT8`
* :code:`TK_UINT8`
* :code:`TK_INT16`
* :code:`TK_UINT16`
* :code:`TK_INT32`
* :code:`TK_UINT32`
* :code:`TK_INT64`
* :code:`TK_UINT64`
* :code:`TK_ENUM`
* :code:`TK_ALIAS` that resolves, directly or indirectly to one of the aforementioned types.

:ref:`Union extensibility<extensibility>` might be configured using |TypeDescriptor-api| :code:`extensibility_kind`
property.

.. note::

  Currently, :code:`@nested` builtin annotation is not supported.

Union members must be configured using the |DynamicTypeBuilder-api| by calling the |DynamicTypeBuilder::add_member|
function with the corresponding |MemberDescriptor-api|.
At least one union member must be added to the union type.
Union member name is configured using |MemberDescriptor-api| :code:`name` property and the member type is set using
:code:`type` property.
It is also mandatory to either set |MemberDescriptor-api| :code:`is_default_label` property or configure the
:code:`label` property.
This latest property indicates the discriminator values that select this specific member.
If no labels are configured, then the flag indicating the member to be the default one, must be set.
Only one union member must be configured as default.

Additionally, |MemberDescriptor-api| :code:`default_value` property might be set to configure the member default value,
and |MemberDescriptor-api| :code:`id` property sets explicitly the member ID.
This behavior is the same as setting the :code:`@default` and :code:`@id`
:ref:`builtin annotations<builtin_annotations>`.

.. note::

  Currently, Fast DDS-Gen does not support :code:`@default` builtin annotation.

.. note::

  Currently, Dynamic Language Binding API implementation does not support the following builtin annotations:

  * :code:`@optional`
  * :code:`@must_understand`
  * :code:`@external`
  * :code:`@try_construct`

Member data can be managed using the corresponding accessors for the underlying member type.
Setting a member automatically changes the discriminator value selecting the set member.
When reading a member, the discriminator must be selecting the member being read.
Member ID might be retrieved using |DynamicData::get_member_id_by_name| API.
For managing complex type members, please refer to :ref:`xtypes_complextypes`.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_UNION
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_UNION<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_UNION
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Union Types<xmldynamictypes_union>`.

.. _xtypes_supportedtypes_bitset:

Bitset Types
^^^^^^^^^^^^

Bitset types are an aggregation of bitfields.
The :code:`TypeKind` used to identify bitsets is :code:`TK_BITSET`.
:code:`bound` property contains the sequence with the bitfield's bitcount (number of bits).
In order to be consistent, the length of the :code:`bound` sequence must agree with the number of bitfields.
Bitset types have single inheritance support, so a bitset type might extend one other already defined bitset.
The bitset type which is extended should be configured in the |TypeDescriptor-api| :code:`base_type` property.

Bitfields must be configured using the |DynamicTypeBuilder-api| by calling the |DynamicTypeBuilder::add_member|
function with the corresponding |MemberDescriptor-api|.
At least one bitfield is required for the bitset to be consistent.
Bitfield name is configured using |MemberDescriptor-api| :code:`name` property, and the bitfield initial bit position
is set using |MemberDescriptor-api| :code:`id` property.

.. note::

  For derived bitsets, the first bitfield initial position must be after the bits defined by the parent bitset type.

A bitfield manages exclusively a set of bits, so no bitfield superposition is allowed.
Additionally, |MemberDescriptor-api| :code:`type` property might be set to configure an integer type to access bitfield
data.
If not set, the minimum unsigned integer type is used instead:

.. list-table::
    :header-rows: 1
    :align: left

    * - Number of bits
      - C++ holder type
    * - :code:`1`
      - :code:`bool`
    * - :code:`2-8`
      - :code:`uint8_t`
    * - :code:`9-16`
      - :code:`uint16_t`
    * - :code:`17-32`
      - :code:`uint32_t`
    * - :code:`33-64`
      - :code:`uint64_t`

Each bitfield (or member) works like its primitive type with the only difference that the internal storage only
modifies the involved bits instead of the full primitive value.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_BITSET
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_BITSET<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_BITSET
        :end-before: //!--
        :dedent: 8

For a detailed explanation about the XML definition of this type, please refer to
:ref:`XML Bitset Types<xmldynamictypes_bitset>`.

.. _xtypes_annotations:

Annotations
^^^^^^^^^^^

Custom annotations
""""""""""""""""""

Both types and type members might be annotated using |DynamicTypeBuilder::apply_annotation| and
|DynamicTypeBuilder::apply_annotation_to_member| API respectively.

Annotations are defined using an |AnnotationDescriptor-api| which provides two properties: :code:`type` and
:code:`value`.
The annotation type must be the DynamicType representing the annotation being applied.
The :code:`TypeKind` used to identify annotations is :code:`TK_ANNOTATION`.
The annotation name is set in |TypeDescriptor-api| :code:`name` property.

The annotation type might have any number of parameters.
Annotation parameters must be configured  using the |DynamicTypeBuilder-api| by calling the
|DynamicTypeBuilder::add_member| function with the corresponding |MemberDescriptor-api|.
Annotation parameters must define the annotation parameter name in |MemberDescriptor-api| :code:`name` property,
and the parameter type using :code:`type` property.
Only the following types can be used to define an annotation parameter:

* `Primitive types`_
* `String types`_
* `Enumeration types`_

.. note::

  Currently, wide string types are not supported as annotation parameters.

Annotation parameter values are defined with |AnnotationDescriptor-api| :code:`value` property using
|AnnotationDescriptor::set_value-api|.
The annotation parameter name provided to the API must coincide with the one defined with the annotation type.
The annotation parameter value must be converted to its string representation.

.. note::

  Currently, custom annotations are not supported with :ref:`XML DynamicTypes <xmldynamictypes>`.

.. tab-set-code::

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_CUSTOM_ANNOTATION
        :end-before: //!--

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_CUSTOM_ANNOTATION<-->
        :end-before: <!--><-->

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_CUSTOM_ANNOTATION
        :end-before: //!--
        :dedent: 8

.. _xtypes_builtin_annotations:

Builtin annotations
"""""""""""""""""""

Beside the user-defined custom annotations, there are a number of builtin annotations that have already mentioned
throughout this section.

The table below summarizes the builtin annotations that can be applied using the Dynamic Language Binding API.
Please, refer to :ref:`builtin annotations <builtin_annotations>` for the complete list and their behavior.

.. list-table::
    :header-rows: 1
    :align: left

    * - Builtin annotation
      - Dynamic Language Binding API
      - Dynamic Language Binding support
      - XML Dynamic Type profiles support
      - IDL Parsing Dynamic Type support
    * - :code:`@appendable`
      - |TypeDescriptor-api| :code:`extensibility_kind` property.
      - ✅
      - ❌
      - ❌
    * - :code:`@bit_bound`
      - |TypeDescriptor-api| :code:`bound` property for :ref:`xtypes_supportedtypes_bitset`. |br|
        |MemberDescriptor-api| :code:`type` property for :ref:`xtypes_supportedtypes_enumeration`.
      - ✅
      - ✅❌ (`Enumeration types`_ not configurable).
      - TODO (Eugenio)
    * - :code:`@default`
      - |MemberDescriptor-api| :code:`default_value` property.
      - ✅
      - ❌
      - TODO (Eugenio)
    * - :code:`default_literal`
      - |MemberDescriptor-api| :code:`is_default_label` property.
      - ✅
      - ❌
      - TODO (Eugenio)
    * - :code:`@extensibility`
      - |TypeDescriptor-api| :code:`extensibility_kind` property.
      - ✅
      - ❌
      - TODO (Eugenio)
    * - :code:`@external`
      - |MemberDescriptor-api| :code:`is_shared` property.
      - ❌
      - ❌
      - ❌
    * - :code:`@final`
      - |TypeDescriptor-api| :code:`extensibility_kind` property.
      - ✅
      - ❌
      - ❌
    * - :code:`@id`
      - |MemberDescriptor-api| :code:`id` property.
      - ✅
      - ❌
      - TODO (Eugenio)
    * - :code:`@key` / :code:`@Key`
      - |MemberDescriptor-api| :code:`is_key` property.
      - ✅
      - ❌
      - ❌
    * - :code:`@mutable`
      - |TypeDescriptor-api| :code:`extensibility_kind` property.
      - ✅
      - ❌
      - ❌
    * - :code:`@nested`
      - |TypeDescriptor-api| :code:`is_nested` property.
      - ❌
      - ❌
      - TODO (Eugenio)
    * - :code:`@optional`
      - |MemberDescriptor-api| :code:`is_optional` property.
      - ❌
      - ❌
      - ❌
    * - :code:`@position`
      - |MemberDescriptor-api| :code:`id` property.
      - ✅
      - ✅
      - TODO (Eugenio)
    * - :code:`@try_construct`
      - |MemberDescriptor-api| :code:`try_construct_kind` property.
      - ❌
      - ❌
      - TODO (Eugenio)
    * - :code:`@value`
      - |MemberDescriptor-api| :code:`default_value` property.
      - ✅
      - ✅
      - TODO (Eugenio)
    * - :code:`@verbatim`
      - |VerbatimTextDescriptor-api|
      - ❌
      - ❌
      - TODO (Eugenio)

.. _xtypes_complextypes:

Managing Complex Types Data
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Some |DynamicData-api| instances manage complex types that cannot be directly modified with the primitive getters and
setters.
Dynamic Language Binding provides two possible approaches for managing complex data types:

1. |DynamicData::get_complex_value| and |DynamicData::set_complex_value|: This API allows to get/set generic DynamicData.
   The main difference with the next approach is that this API performs always a copy.
2. |DynamicData::loan_value|: this API allows to loan a reference to a DynamicData to work with preventing the data copy.
   |DynamicData::return_loaned_value| must be called to return the loan.
   Calling |DynamicData::loan_value| for an already loaned value will fail.

The following snippet includes an example of managing complex data using the same structure as the one defined in
`Structure types`_:

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //!--CPP_COMPLEX_DATA
    :end-before: //!--
    :dedent: 8
