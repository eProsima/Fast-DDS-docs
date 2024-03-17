.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dynamictypes_language_binding:

Dynamic Language Binding
========================

*eProsima Fast DDS* supports several methods for defining |DynamicTypes|.
Manual creation using |DynamicTypeBuilderFactory-api| and |DynamicTypeBuilder-api| to create the types.

:ref:`XMLDynamicTypes <xmldynamictypes>` allows *eProsima Fast DDS* to create DynamicTypes directly defining 
them through XML.

:ref:`Fast DDS-Gen <fastddsgen_intro>` provides a way to automatically generate the code to define the |DynamicTypes| 
from the IDL files.

This section contains a detailed explanation of how to create |DynamicTypes| using the manual way, as well as the 
same examples using XML and IDL files.

.. _dynamictypes_supportedtypes:

Supported Types
---------------

.. _dynamictypes_supportedtypes_primitive:

Primitives
^^^^^^^^^^

By definition, primitive types are self-described and can be created without configuration parameters.
Therefore, |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::get_primitive_type| 
to allow users to create the type avoiding the |DynamicTypeBuilder-api| step.
The |DynamicData-api| class has a specific :func:`get__value` and :func:`set__value` functions for each primitive 
type of the list.

The following table shows the supported primitive types and their corresponding ``TypeKind``.
The ``TypeKind`` is used to identify the type when creating it.

+------------------+-------------------+
| Type             | TypeKind          |
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

This examples show how to create members of each primitive type using |DynamicTypeBuilderFactory-api|.

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

.. _dynamictypes_supportedtypes_string:

Strings
^^^^^^^

Strings are pretty similar to primitive types, the main difference being that they need to set their size limit.

|DynamicTypeBuilderFactory-api| exposes the functions |DynamicTypeBuilderFactory::create_string_type| and 
|DynamicTypeBuilderFactory::create_wstring_type| to allow users create them directly.
This functions receive a parameter to set the maximum length of the string, using ``LENGTH_UNLIMITED`` 
to set it as unlimited.
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

.. _dynamictypes_supportedtypes_enumeration:

Enumerations
************

An enumeration contains a set of supported values and a selected value among those supported.
The ``TypeKind`` used to identify Enumerations is ``TK_ENUM``.

The supported values must be configured using the |DynamicTypeBuilder-api|, using the |DynamicTypeBuilder::add_member| 
function for each supported value.

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

.. _dynamictypes_supportedtypes_bitmask:

Bitmasks
********

Bitmasks are similar to |Enumeration| types, but their members work as bit flags that can be individually turned on and
off.
The ``TypeKind`` used to identify Bitmasks is ``TK_BITMASK``.

Those flags might use the ``positition`` attribute to set their position in the bitmask.
The ``bit_bound`` attribute specifies the number of bits that the bitmask type will manage.
Bitmasks can be bound to any number of bits up to 64.
The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_bitmask_type| to 
facilitate the creation of this type.

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

.. _dynamictypes_supportedtypes_alias:

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

.. _dynamictypes_complextypes_aliasalias:

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

.. _dynamictypes_supportedtypes_sequence:

Sequences
*********

A complex type that manages its members as a list of items allowing users to insert, 
remove or access to a member of the list.

The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_sequence_type| to 
facilitate the creation of this type.
To create this type users need to specify the type that it is going to store.
Additionally, the size limit of the list. Users can use ``LENGTH_UNLIMITED`` to create unbounded sequences.

The |DynamicData-api| class has a specific :func:`get__values` and :func:`set__values` functions for each primitive 
type that allow users to work with multiple values at once.

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

.. _dynamictypes_supportedtypes_array:

Arrays
******

Arrays are pretty similar to sequences with two main differences:
They can have multiple dimensions and they do not need their elements to be stored consecutively.

An array needs to know the number of dimensions it is managing.
For that, users must provide a vector with as many elements as dimensions in the array.
Each element in the vector represents the size of the given dimension.

The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_array_type| to 
facilitate the creation of this type.

The |DynamicData-api| class has a specific :func:`get__values` and :func:`set___values` functions for each 
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

.. _dynamictypes_supportedtypes_map:

Maps
****

Maps contain a list of 'key-value' pair types, allowing users to insert, remove or modify the element types of the map.
The main difference with sequences is that the map works with pairs of elements and creates copies of the key element 
to block the access to these elements.

The |DynamicTypeBuilderFactory-api| exposes the function |DynamicTypeBuilderFactory::create_map_type| to 
facilitate the creation of this type.
To create a map, users must set the types of the key and the value elements.
Additionally, the size limit of the map. Users can use ``LENGTH_UNLIMITED`` to create unbounded sequences.

The |DynamicData-api| class has a specific :func:`get__values` and :func:`set___values` functions for each primitive 
type that allow users to work with multiple values at once.

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

.. _dynamictypes_supportedtypes_structure:

Structures
**********

Structures are the most common complex types, they allow to add any kind of members inside them.
They do not have any value, they are only used to contain other types.

The function |DynamicTypeBuilderFactory::create_type| is used to create a new structure type.
The ``TypeKind`` used to identify Structures is ``TK_STRUCTURE``.

To manage the types inside the structure, users can call the :func:`get` and :func:`set` functions according to the 
kind of the type inside the structure using their ``ids``.
If the structure contains a complex value, |DynamicData::loan_value| should be used to access it and 
|DynamicData::return_loaned_value| to release that loan.
|DynamicData-api| manages the loaned values and users can not loan a value that has been loaned previously 
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

.. _dynamictypes_supportedtypes_union:

Unions
******

Unions are a special kind of structures where only one of the members is active at the same time.
The ``TypeKind`` used to identify Unions is ``TK_UNION``.

To control these members, users must set the ``discriminator`` type that is going to be used to select the current 
member calling the |TypeDescriptor::discriminator_type| function of the union type.
The ``discriminator`` itself is a DynamicType of any primitive type, string type or union type.

Additionally users can use the function |MemberDescriptor::label| with one or more values to set the labels of each 
member of the union, and |MemberDescriptor::is_default_label| to specify the default label of the union.

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

.. _dynamictypes_supportedtypes_bitset:

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

.. _dynamictypes_complextypes:

Complex types
^^^^^^^^^^^^^

In complex data models, combinations of basic types :ref:`basic types<dynamictypes_supportedtypes>`
can be used to create intricate structures, including nested compositions such as structures within structures.
Additionally, types can be extended using inheritance to enhance flexibility.

The following subsections describe these *complex types* and their use.

Inheritance
"""""""""""

.. _dynamictypes_structure_inheritance:

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

.. _dynamictypes_bitset_inheritance:

Bitsets allows inheritance aswell, exactly with the same OOP meaning.
To inherit from another bitset, users must follow the same process as with 
:ref:`structures <dynamictypes_structure_inheritance>`, but using bitset types.

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
please refer to :ref:`Bitset <xmldynamictypes_bitset_inheritance>`.

Nested Types
""""""""""""

.. _dynamictypes_nested_structures:

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

.. _dynamictypes_nested_unions:

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
please refer to :ref:`Complex types <xmldynamictypes_complextypes>`.

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
