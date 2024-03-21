.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _xtypes_language_binding:

Dynamic Language Binding
========================

*eProsima Fast DDS* supports several methods to define |DynamicTypes|.
Manual creation using |DynamicTypeBuilderFactory-api| and |DynamicTypeBuilder-api| to create the types.

:ref:`XMLDynamicTypes <xmldynamictypes>` allows *eProsima Fast DDS* to create DynamicTypes by defining 
them directly through XML.

This section contains an small description of the interfaces used to work with |DynamicTypes| and a detailed
explanation of how to create |DynamicTypes| in the manual way, as well as the same examples using XML files.

.. _xtypes_intenfaces:

Dynamic Types Interfaces
------------------------

DynamicTypeBuilderFactory
^^^^^^^^^^^^^^^^^^^^^^^^^
The |DynamicTypeBuilderFactory-api| serves as a singleton wich isntance is responsible for creating |DynamicTypes| and
serves as the entry point for both creating and deleting |DynamicTypeBuilder-api| objects.
The simpler types can be directly instantiated without the need for a DynamicTypeBuilder.
Refer to the Supported Types documentation for details on which types support this direct creation option.

AnnotationDescriptor
^^^^^^^^^^^^^^^^^^^^
|AnnotationDescriptor-api| encapsulates the state of an annotation when applied to an element (not an annotation type).

TypeDescriptor
^^^^^^^^^^^^^^
|TypeDescriptor-api| stores the state of a type, including its structure, relationships, and constraints.
As the class responsible for describing the inner structure of a DynamicType,
When the DynamicType is created, DynamicTypeBuilderFactory utilizes the information from the TypeDescriptor to
instantiate the DynamicType.
This information is then copied to the DynamicType during creation, gaining independence from the DynamicTypeBuilder

so the builder can be reused for another type.

DynamicTypeMember
^^^^^^^^^^^^^^^^^
|DynamicTypeMember-api| represents a data member of a DynamicType that itself is a DynamicType.
Dynamic types that are composed of other dynamic types have a DynamicTypeMember for every child
DynamicType added to it.
Functioning as the representation of a "member" of a type, it may be a member of an aggregate type,
or constants within enumerations, or other types of substructure.

MemberDescriptor
^^^^^^^^^^^^^^^^
|MemberDescriptor-api| encapsulates the state of a DynamicTypeMember.
Analogous to TypeDescriptor's role in describing the inner structure of a DynamicType, MemberDescriptor
stores all essential information for managing a DynamicTypeMember.
This information, including member name, unique ID, and default value, is copied to the DynamicData
upon its creation.

DynamicType
^^^^^^^^^^^
|DynamicType-api| objects represent a type's schema, including its name, type kind, and member definitions.
As the base class for all dynamically declared types, it is the representation of a dynamic data type used
for creating DynamicData values.
Once created, the structure of a DynamicType cannot be modified.

DynamicTypeBuilder
^^^^^^^^^^^^^^^^^^
|DynamicTypeBuilder-api| represents the transitional state of a type defined in the Type System.
It enables the instantiation of concrete DynamicType objects and serves as an intermediary for configuring a
DynamicType before its creation.
A DynamicType's composition must be defined before its creation, with the builder facilitating this setup,
and its structure cannot be modified once the object is created.
Upon definition, DynamicTypeBuilderFactory leverages the information contained in the builder to create
the DynamicType.
The builder offers a :func:`build` function for convenient creation of a fully constructed DynamicType,
utilizing DynamicTypeBuilderFactory internally.
Builders remain reusable after DynamicType creation, ensuring changes to the builder do not affect previously
created types.

DynamicDataFactory
^^^^^^^^^^^^^^^^^^
|DynamicDataFactory-api|, logically a singleton, oversees the creation of DynamicData objects.
As the sole entry point for creating and deleting DynamicData and objects, it operates similarly to the singleton
DomainParticipantFactory.
This singleton class manages every DynamicData instance, capable of generating an instance corresponding to a specified
DynamicType.

DynamicData
^^^^^^^^^^^
|DynamicData-api| represents a data instance of a DynamicType, providing functionalities to access
and modify data values.
Each DynamicData object corresponds to an object of the type represented by its DynamicType.
Offering reflective getters and setters, DynamicData enables manipulation of individual data samples.

.. _xtypes_supportedtypes:

Supported Types
---------------

.. _xtypes_supportedtypes_primitive:

Primitives
^^^^^^^^^^

By definition, primitive types are self-describing and can be created without configuration parameters.
Therefore, the |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::get_primitive_type| 
to allow users to create the type bypassing the |DynamicTypeBuilder-api| step.
The |DynamicData-api| class has a specific :func:`get_value` and :func:`set_value` functions for each primitive 
type in the list.

The following table shows the supported primitive types and their corresponding ``TypeKind``.
The ``TypeKind`` is used to identify the type when it is created.

+------------------+-------------------+
| C++ Type         | TypeKind          |
+------------------+-------------------+
| ``BOOLEAN``      | ``TK_BOOLEAN``    |
+------------------+-------------------+
| ``CHAR8``        | ``TK_CHAR8``      |
+------------------+-------------------+
| ``CHAR16``       | ``TK_CHAR16``     |
+------------------+-------------------+
| ``OCTET``        | ``TK_BYTE``       |
+------------------+-------------------+
| ``INT8``         | ``TK_INT8``       |
+------------------+-------------------+
| ``UINT8``        | ``TK_UINT8``      |
+------------------+-------------------+
| ``INT16``        | ``TK_INT16``      |
+------------------+-------------------+
| ``UINT16``       | ``TK_UINT16``     |
+------------------+-------------------+
| ``INT32``        | ``TK_INT32``      |
+------------------+-------------------+
| ``UINT32``       | ``TK_UINT32``     |
+------------------+-------------------+
| ``INT64``        | ``TK_INT64``      |
+------------------+-------------------+
| ``UINT64``       | ``TK_UINT64``     |
+------------------+-------------------+
| ``FLOAT32``      | ``TK_FLOAT32``    |
+------------------+-------------------+
| ``FLOAT64``      | ``TK_FLOAT64``    |
+------------------+-------------------+
| ``FLOAT128``     | ``TK_FLOAT128``   |
+------------------+-------------------+

This example shows how to create members of each primitive type using |DynamicTypeBuilderFactory-api|.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_PRIMITIVES
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_PRIMITIVES<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_PRIMITIVES
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Primitives <xmldynamictypes_primivites>`.

.. _xtypes_supportedtypes_string:

Strings
^^^^^^^

Strings are quite similar to primitive types, the main difference being that they need to set their size limit.

The |DynamicTypeBuilderFactory-api| exposes the functions |DynamicTypeBuilderFactory::create_string_type| and 
|DynamicTypeBuilderFactory::create_wstring_type| to allow users to create them directly.
These functions take a parameter to set the maximum length of the string, using ``LENGTH_UNLIMITED`` 
to set it to unlimited.
The |DynamicData-api| class has a specific :func:`get_string_value` and :func:`set_string_value`.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_STRINGS
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_STRINGS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_STRINGS
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Primitives <xmldynamictypes_primivites>`.

Constructed Types
^^^^^^^^^^^^^^^^^

Enumerated Types
""""""""""""""""

.. _xtypes_supportedtypes_enumeration:

Enumerations
************

An enumeration contains a set of supported values and a selected value among those supported.
The ``TypeKind`` used to identify Enumerations is ``TK_ENUM``.

The supported values must be configured using the |DynamicTypeBuilder-api|, using the |DynamicTypeBuilder::add_member| 
function for the respective supported values.
The type to which the enumeration will be bound to is determined the first time
|DynamicTypeBuilder::add_member| is invoked by taking the type of that member.
Subsequently added members must follow the same type.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_ENUM
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_ENUM<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_ENUM
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Enumerations <xmldynamictypes_enums>`.

.. _xtypes_supportedtypes_bitmask:

Bitmasks
********

Bitmasks are similar to |Enumeration| types, but their members act as bit flags that can be turned on and
off individually.
The ``TypeKind`` used to identify Bitmasks is ``TK_BITMASK``.

These flags might use the ``id`` attribute to set their position in the bitmask.
The ``bound`` attribute specifies the number of bits that the bitmask type will manage.
Bitmasks can be bound to any number of bits up to 64.
The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_bitmask_type| to 
facilitate the creation of this type.
The members added to a bitmask using |DynamicTypeBuilder::add_member| must be of type ``TK_BOOLEAN``.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_BITMASK
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITMASK<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITMASK
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Bitmask <xmldynamictypes_bitmask>`.

.. _xtypes_supportedtypes_alias:

Alias
"""""

Alias types provide an alternative name to an already existing type.
The ``TypeKind`` used to identify Alias is ``TK_ALIAS``.

Once the |DynamicData-api| is created, users can access its information as if they were working with the base type.
To create an alias type, users must use the |TypeDescriptor::base_type| function from the |TypeDescriptor-api| with 
the existing type they want the alias to represent.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_TYPEDEF
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_TYPEDEF<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_TYPEDEF
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Typedef <xmldynamictypes_typedef>`.

.. _xtypes_complextypes_aliasalias:

Alias types support recursion, simply by using an alias as base type for :|TypeDescriptor::base_type|.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_RECURSIVE_TYPEDEF
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_RECURSIVE_TYPEDEF<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_RECURSIVE_TYPEDEF
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Primitives <xmldynamictypes_primivites>`.

Collections
"""""""""""

.. _xtypes_supportedtypes_sequence:

Sequences
*********

A complex type that manages its members as a list of elements allowing users to insert, 
remove or access to a member of the list.

The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_sequence_type| to 
facilitate the creation of this type.
To create this type, the user must specify the type to be stored.
Additionally, the size limit of the list. Users can use ``LENGTH_UNLIMITED`` to create unbounded sequences.

The |DynamicData-api| class has a specific :func:`get_values` and :func:`set_values` functions for each primitive 
type, allowing users to work with multiple values at once.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_SEQUENCES
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_SEQUENCES<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_SEQUENCES
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Sequences <xmldynamictypes_sequence>`.

.. _xtypes_supportedtypes_array:

Arrays
******

Arrays are very similar to sequences with two main differences:
They can have multiple dimensions and their elements are initialized to the default value at the start.

An array needs to know the number of dimensions it maanges.
The user must provide a vector with as many elements as there are dimensions in the array.
Each element in the vector represents the size of the given dimension.

The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_array_type| to 
facilitate the creation of this type.

The |DynamicData-api| class has a specific :func:`get_values` and :func:`set_values` functions for each 
primitive type that allow users to work with multiple values at once.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_ARRAYS
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_ARRAYS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_ARRAYS
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Arrays <xmldynamictypes_array>`.

.. _xtypes_supportedtypes_map:

Maps
****

Maps contain a list of 'key-value' pair types, allowing users to insert, remove or modify the element types of the map.
The main difference with sequences is that the map works with pairs of elements and creates copies of the key element 
to block the access to these elements.

The |DynamicTypeBuilderFactory-api| exposes the |DynamicTypeBuilderFactory::create_map_type| function to 
facilitate the creation of this type.
To create a map, users must set the types of the key and the value elements.
Additionally, the size limit of the map. Users can use ``LENGTH_UNLIMITED`` to create unbounded sequences.

The |DynamicData-api| class has a specific :func:`get__values` and :func:`set___values` functions for each primitive 
type that allow users to work with multiple values at once.

To access the members of the map using the keys, users can use the |DynamicData::get_member_id_by_name| function to get
the ``MemberId`` of the desired element key, and use it to modify the value of said element.
Note that parameter used in |DynamicData::get_member_id_by_name| as the key name, must be the correct ``string``
representation of the value of the key.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_MAPS
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_MAPS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_MAPS
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Maps <xmldynamictypes_map>`.

Aggregated Types
""""""""""""""""

.. _xtypes_supportedtypes_structure:

Structures
**********

Structures are the most common aggregated types, they allow any kind of members to be added inside them.
They do not have any value, they are only used to contain other types.

The function |DynamicTypeBuilderFactory::create_type| is used to create a new structure type.
The ``TypeKind`` used to identify structures is ``TK_STRUCTURE``.

To manage the types inside the structure, users can call the :func:`get` and :func:`set` functions according to the 
kind of the type inside the structure using its ``id``.
If the structure contains a complex value, |DynamicData::loan_value| should be used to access it and 
|DynamicData::return_loaned_value| should be used to release that loan.
|DynamicData-api| manages the loaned values and users cannot loan a previously loaned value 
without calling |DynamicData::return_loaned_value| before.


.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_STRUCT
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_STRUCT<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_STRUCT
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Struct <xmldynamictypes_struct>`.

.. _xtypes_supportedtypes_union:

Unions
******

Unions are a special type of structure where only one of the members is active at a time.
The ``TypeKind`` used to identify unions is ``TK_UNION``.

To control these members, users must set the ``discriminator`` type that is going to be used to select the current 
member by calling the |TypeDescriptor::discriminator_type| function of the union type.
The ``discriminator`` itself is a DynamicType that must be of any of the following types:
- Boolean.
- Byte.
- Char8, Char16.
- Int8, UInt8, Int16, UInt16, Int32, UInt32, Int64, UInt64.
- An enumerated type.
- Any alias type that resolves, directly or indirectly, to one of the aforementioned types.

In addition, users can use the |MemberDescriptor::label| function with one or more values to set the labels of each 
member of the union, and |MemberDescriptor::is_default_label| to set the default label of the union.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_UNION
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_UNION<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_UNION
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Union <xmldynamictypes_union>`.

.. _xtypes_supportedtypes_bitset:

Bitset
******

Bitset types are similar to `structure` types, but their members are merely `bitfields`, which are stored optimally.
In the static version of bitsets, each bit uses just one bit in memory (with platform limitations) without alignment
considerations.
A bitfield can be anonymous (cannot be addressed) to skip unused bits within a bitset.

Each bitfield in a bitset can be modified through their minimal needed primitive representation.

+--------------------------+--------------------------+
| Number of bits           | Primitive                |
+--------------------------+--------------------------+
| ``1``                    | ``BOOLEAN``              |
+--------------------------+--------------------------+
| ``2-8``                  | ``UINT8``                |
+--------------------------+--------------------------+
| ``9-16``                 | ``UINT16``               |
+--------------------------+--------------------------+
| ``17-32``                | ``UINT32``               |
+--------------------------+--------------------------+
| ``33-64``                | ``UINT64``               |
+--------------------------+--------------------------+

Each bitfield (or member) works like its primitive type with the only difference that the internal storage only 
modifies the involved bits instead of the full primitive value.
The ``TypeKind`` used to identify Bitsets is ``TK_BITSET``.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_BITSET
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITSET<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITSET
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Bitset <xmldynamictypes_bitset>`.

.. _xtypes_complextypes:

Complex types
^^^^^^^^^^^^^

In complex data models, combinations of basic types :ref:`basic types<xtypes_supportedtypes>`
can be used to create intricate structures, including nested compositions such as structures within structures.
Additionally, types can be extended using inheritance to enhance flexibility.

The following subsections describe these *complex types* and their use.

Inheritance
"""""""""""

.. _xtypes_structure_inheritance:

Structures allow inheritance, exactly with the same OOP meaning.
To inherit from another structure, users must create the parent structure calling the 
|DynamicTypeBuilderFactory::create_type| normally.
After the parent type is created, use the |TypeDescriptor::base_type| function from the |TypeDescriptor-api| 
when creating the child structure, using the parent type as the base type. 

The resultant type contains all members from the base class and the new ones added to the child.
Structures support several levels of inheritance, so the base class can be another derived type itself.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_STRUCT_INHERITANCE
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_STRUCT_INHERITANCE<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_STRUCT_INHERITANCE
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Struct <xmldynamictypes_struct_inheritance>`.

.. _xtypes_bitset_inheritance:

Bitsets allows inheritance aswell, exactly with the same OOP meaning.
To inherit from another bitset, users must follow the same process as with 
:ref:`structures <xtypes_structure_inheritance>`, but using bitset types.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_BITSET_INHERITANCE
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITSET_INHERITANCE<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITSET_INHERITANCE
            :end-before: //!--

For a detailed explanation about the XML definition of this type,
please refer to :ref:`Bitset <xmlxtypes_bitset_inheritance>`.

Nested Types
""""""""""""

.. _xtypes_nested_structures:

Structures can contain other structures as members.
The access to these compound members is restricted and managed by the |DynamicData-api| instance.
Users must request access calling |DynamicData::loan_value| before using them, 
and release them with |DynamicData::return_loaned_value| once they finished.
The loan operation will fail if the member is already loaned and has not been released yet.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_COMPLEX_STRUCTS
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_COMPLEX_STRUCTS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_COMPLEX_STRUCTS
            :end-before: //!--

.. _xtypes_nested_unions:

Unions support complex type fields.
The access to these complex type fields is restricted and managed by the |DynamicData-api| instance.
Users must request access calling |DynamicData::loan_value| before using them, 
and release them with |DynamicData::return_loaned_value| once they finished.
The loan operation will fail if the fields is already loaned and has not been released yet.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_COMPLEX_UNIONS
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_COMPLEX_UNIONS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_COMPLEX_UNIONS
            :end-before: //!--


For a detailed explanation about the XML definition of this types,
please refer to :ref:`Complex types <xmlxtypes_complextypes>`.

Annotations
^^^^^^^^^^^

|DynamicTypeBuilder-api| allows applying an annotation to both current type and inner members with the functions.
To add the annotations to the types, the function |DynamicTypeBuilder::apply_annotation| is used, and 
|DynamicTypeBuilder::apply_annotation_to_member| to add them to the members.

Both functions take the :class:`AnnotationDescriptor` of the annotation to be added, 
and |DynamicTypeBuilder::apply_annotation_to_member| additionally receives the ``MemberId`` of the inner member.


Custom annotations
""""""""""""""""""

|DynamicTypes| allows users to create custom annotations to add extra information to the types.

:ref:`XMLDynamicTypes <xmldynamictypes>` does not currently support custom annotations.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: idl
            :start-after: //!--IDL_CUSTOM_ANNOTATION
            :end-before: //!--

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_CUSTOM_ANNOTATION
            :end-before: //!--


Builtin annotations
"""""""""""""""""""

Beside the custom annotations created by the users, there are a number of builtin annotations.

Currently, |DynamicTypes| supports the following builtin annotations:

+-------------------------+-------------------------------------------------------------------------------------------+
| Annotation              | Implemented behavior                                                                      |
+=========================+===========================================================================================+
| ``@extensibility``      | Applied to any element which is constructed. Allow specifying how the |br|                |
|                         | element is allowed to evolve. More info in :ref:`extensibility`.                          |
+-------------------------+-------------------------------------------------------------------------------------------+
| ``@final``              | Shortcut for `@extensibility(FINAL)`                                                      |
+-------------------------+-------------------------------------------------------------------------------------------+
| ``@appendable``         | Shortcut for `@extensibility(APPENDABLE)`                                                 |
+-------------------------+-------------------------------------------------------------------------------------------+
| ``@mutable``            | Shortcut for `@extensibility(MUTABLE)`                                                    |
+-------------------------+-------------------------------------------------------------------------------------------+
| ``@key``                | Alias for eProsima's @Key annotation. Indicate that a data member is part of the key |br| |
|                         | (please refer to :ref:`dds_layer_topic_instances` for more information).                  |
+-------------------------+-------------------------------------------------------------------------------------------+

To apply the @extensibility annotation (and its shortcuts) the |TypeDescriptor-api| exposes the function
|TypeDescriptor::extensibility_kind|, that recives a :class:`ExtensibilityKind`.

For the @key annotation the function |MemberDescriptor::is_key| can be fount in |MemberDescriptor-api|.

Type promotions
---------------

|DynamicTypes| supports type promotion, enabling implicit promotion of types during both :func:`get` and :func:`set` 
operations.
This means that a smaller type can be implicitly promoted to a larger type, but not the other way around.

The following promotions are supported:

+--------------+-------------------------------------------------------------------------------------------+
| Type         | Promotions                                                                                |
+==============+===========================================================================================+
| `Int8`       | Int16, Int32, Int64, Float32, Float64, Float128                                           |
+--------------+-------------------------------------------------------------------------------------------+
| `Int16`      | Int32, Int64, Float32, Float64, Float128                                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `Int32`      | Int64, Float64, Float128                                                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `Int64`      | Float128                                                                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `UInt8`      | Int16,Int32, Int64, UInt16, UInt32, UInt64, Float32, Float64, Float128                    |
+--------------+-------------------------------------------------------------------------------------------+
| `UInt16`     | Int32, Int64, UInt32, UInt64, Float32, Float64, Float128                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `UInt32`     | Int64, UInt64, Float64, Float128                                                          |
+--------------+-------------------------------------------------------------------------------------------+
| `UInt64`     | Float128                                                                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `Float32`    | Float64, Float128                                                                         |
+--------------+-------------------------------------------------------------------------------------------+
| `Float64`    | Float128                                                                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `Float128`   | (none)                                                                                    |
+--------------+-------------------------------------------------------------------------------------------+
| `Char8`      | Char16, Int16, Int32, Int64, Float32, Float64, Float128                                   |
+--------------+-------------------------------------------------------------------------------------------+
| `Char16`     | Int32, Int64, Float32, Float64, Float128                                                  |
+--------------+-------------------------------------------------------------------------------------------+
| `Byte`       | (any)                                                                                     |
+--------------+-------------------------------------------------------------------------------------------+
| `Boolean`    | Int8, Int16, Int32, Int64, UInt8, UInt16, UInt32, UInt64, Float32, Float64, Float128      |
+--------------+-------------------------------------------------------------------------------------------+
