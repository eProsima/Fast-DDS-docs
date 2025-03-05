.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _fastddsgen_idl_datatypes:

Defining a data type via IDL
=============================

This section describes the data types that can be defined using IDL files, as well as other mechanisms for building
data types using IDL files.

.. contents::
    :local:
    :backlinks: none
    :depth: 2


Supported IDL types
-------------------

Be aware that *Fast DDS-Gen* is not case sensitive as it is specified in the
`IDL specification <https://www.omg.org/spec/IDL/4.2/PDF>`_.
To activate case sensitivity use option :code:`-cs` when running *Fast DDS-Gen* (see
:ref:`fastddsgen_supported_options`).

.. warning::

  Note that IDL files created by ROS 2 are not necessarily compatible with Fast DDS applications, since they are processed in a different manner and can lead to incompatible type definitions.
  For a detailed explanation on how to ensure compatibility between ROS 2 and Fast DDS applications, please refer to this `Vulcanexus tutorial <https://docs.vulcanexus.org/en/jazzy/rst/tutorials/core/deployment/dds2vulcanexus/dds2vulcanexus_ros2idl.html>`_.

.. _idl_primitive_types:

Primitive types
^^^^^^^^^^^^^^^

The following table shows the basic IDL types supported by *Fast DDS-Gen* and how they are mapped to C++11.

+--------------------+-----------------+
| IDL                | C++11           |
+====================+=================+
| char               | ``char``        |
+--------------------+-----------------+
| octet              | ``uint8_t``     |
+--------------------+-----------------+
| short              | ``int16_t``     |
+--------------------+-----------------+
| unsigned short     | ``uint16_t``    |
+--------------------+-----------------+
| long               | ``int32_t``     |
+--------------------+-----------------+
| unsigned long      | ``uint32_t``    |
+--------------------+-----------------+
| long long          | ``int64_t``     |
+--------------------+-----------------+
| unsigned long long | ``uint64_t``    |
+--------------------+-----------------+
| float              | ``float``       |
+--------------------+-----------------+
| double             | ``double``      |
+--------------------+-----------------+
| long double        | ``long double`` |
+--------------------+-----------------+
| boolean            | ``bool``        |
+--------------------+-----------------+
| string             | ``std::string`` |
+--------------------+-----------------+

Arrays
^^^^^^

*Fast DDS-Gen* supports unidimensional and multidimensional arrays.
Arrays are always mapped to ``std::array`` containers.
The following table shows the array types supported and their mapping.

+-------------------------+------------------------------+
| IDL                     | C++11                        |
+=========================+==============================+
| char a[5]               | ``std::array<char,5> a``     |
+-------------------------+------------------------------+
| octet a[5]              | ``std::array<uint8_t,5> a``  |
+-------------------------+------------------------------+
| short a[5]              | ``std::array<int16_t,5> a``  |
+-------------------------+------------------------------+
| unsigned short a[5]     | ``std::array<uint16_t,5> a`` |
+-------------------------+------------------------------+
| long a[5]               | ``std::array<int32_t,5> a``  |
+-------------------------+------------------------------+
| unsigned long a[5]      | ``std::array<uint32_t,5> a`` |
+-------------------------+------------------------------+
| long long a[5]          | ``std::array<int64_t,5> a``  |
+-------------------------+------------------------------+
| unsigned long long a[5] | ``std::array<uint64_t,5> a`` |
+-------------------------+------------------------------+
| float a[5]              | ``std::array<float,5> a``    |
+-------------------------+------------------------------+
| double a[5]             | ``std::array<double,5> a``   |
+-------------------------+------------------------------+

Sequences
^^^^^^^^^

*Fast DDS-Gen* supports sequences, which map into the ``std::vector`` container.
The following table represents how the map between IDL and C++11 is handled.

+------------------------------+---------------------------+
| IDL                          | C++11                     |
+==============================+===========================+
| sequence<char>               | ``std::vector<char>``     |
+------------------------------+---------------------------+
| sequence<octet>              | ``std::vector<uint8_t>``  |
+------------------------------+---------------------------+
| sequence<short>              | ``std::vector<int16_t>``  |
+------------------------------+---------------------------+
| sequence<unsigned short>     | ``std::vector<uint16_t>`` |
+------------------------------+---------------------------+
| sequence<long>               | ``std::vector<int32_t>``  |
+------------------------------+---------------------------+
| sequence<unsigned long>      | ``std::vector<uint32_t>`` |
+------------------------------+---------------------------+
| sequence<long long>          | ``std::vector<int64_t>``  |
+------------------------------+---------------------------+
| sequence<unsigned long long> | ``std::vector<uint64_t>`` |
+------------------------------+---------------------------+
| sequence<float>              | ``std::vector<float>``    |
+------------------------------+---------------------------+
| sequence<double>             | ``std::vector<double>``   |
+------------------------------+---------------------------+

Maps
^^^^

*Fast DDS-Gen* supports maps, which are equivalent to the ``std::map`` container.
The equivalence between types is handled in the same way as for sequences_.

+-------------------------------+---------------------------------+
| IDL                           | C++11                           |
+===============================+=================================+
| map<char, unsigned long long> |  ``std::map<char, uint64_T>``   |
+-------------------------------+---------------------------------+

.. note::

    Only :ref:`idl_primitive_types` are currently supported.

Structures
^^^^^^^^^^

You can define an IDL structure with a set of members with multiple types.
It will be converted into a C++ class in which the members of the structure defined via IDL are mapped to private data
members of the class.
Furthermore, :func:`set` and :func:`get` member functions are created to access these private data members.

The following IDL structure:

.. code-block:: omg-idl

    struct Structure
    {
        octet octet_value;
        long long_value;
        string string_value;
    };

Would be converted to:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // STRUCTURES_DATA_TYPE
   :end-before: //!

Structures can inherit from other structures, extending their member set.

.. code-block:: omg-idl

    struct ParentStruct
    {
        octet parent_member;
    };

    struct ChildStruct : ParentStruct
    {
        long child_member;
    };

In this case, the resulting C++ code will be:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // STRUCTURE_INHERITANCE
   :end-before: //!

.. _optional_members:

Optional members
""""""""""""""""

A member of a structure can be optional.
This is achieved by writing the ``@optional`` annotation before the member.

.. code-block:: omg-idl

    struct StructWithOptionalMember
    {
        @optional octet octet_opt;
    };

An optional member is converted into a template class ``eprosima::fastcdr::optional<T>``, where ``T`` is the member's
type.

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // STRUCTURE_WITH_OPTIONAL
   :end-before: //!

Before reading the value of the optional member, it should be checked the optional contains a value using
``has_value()`` function.
Accessing a *null* optional throws a ``eprosima::fastcdr::exception::BadOptionalAccessException`` exception.

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // ACCESSING_OPTIONAL_VALUE
   :dedent: 4
   :end-before: //!

.. _extensibility:

Extensibility
"""""""""""""

In order to support evolving types without breaking interoperability, the concept of type extensibility is supported by
*Fast DDS-Gen*.
There are three extensibility kinds: *final*, *appendable* and *mutable*.

* *FINAL* extensibility indicates that the type is strictly defined. It is not possible to add members while maintaining
  type assignability.
* *APPENDABLE* extensibility indicates that two types, where one contains all of the members of the other plus
  additional members appended to the end, may remain assignable.
* *MUTABLE* extensibility indicates that two types may differ from one another in the additional, removal, and/or
  transposition of members while remaining assignable.

.. code-block:: omg-idl

    @extensibility(FINAL)
    struct FinalStruct
    {
        octet octet_opt;
    };

    @extensibility(APPENDABLE)
    struct AppendableStruct
    {
        octet octet_opt;
    };

    @extensibility(MUTABLE)
    struct MutableStruct
    {
        octet octet_opt;
    };

.. note::

    XCDRv1 encoding algorithm is not able to manage correctly the deserialization of an appendable structure when it is
    used as a member of another one.

Unions
^^^^^^

In IDL, a union is defined as a sequence of members with their own types and a discriminant that specifies which member
is in use.
An IDL union type is mapped as a C++ class with member functions to access the union members and the discriminant.

The following IDL union:

.. code-block:: omg-idl

    union Union switch(long)
    {
       case 1:
        octet octet_value;
      case 2:
        long long_value;
      case 3:
        string string_value;
    };

Would be converted to:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // UNION_DATA_TYPE
   :end-before: //!

Bitsets
^^^^^^^

Bitsets are a special kind of structure, which encloses a set of bits.
A bitset can represent up to 64 bits.
Each member is defined as *bitfield* and eases the access to a part of the bitset.

For example:

.. code-block:: omg-idl

    bitset MyBitset
    {
        bitfield<3> a;
        bitfield<10> b;
        bitfield<12, long> c;
    };

The type :class:`MyBitset` will store a total of 25 bits (3 + 10 + 12) and will require 32 bits in memory
(lowest primitive type to store the bitset's size).

- The bitfield 'a' allows us to access to the first 3 bits (0..2).
- The bitfield 'b' allows us to access to the next 10 bits (3..12).
- The bitfield 'c' allows us to access to the next 12 bits (13..24).

The resulting C++ code will be similar to:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // BITSET_DATA_TYPE
   :end-before: //!

Internally, it is stored as a ``std::bitset``.
For each bitfield, :func:`get` and :func:`set` member functions are generated with the smaller possible primitive
unsigned type to access it.
In the case of bitfield 'c', the user has established that this accessing type will be ``long``, so the generated code
uses ``int32_t`` instead of automatically use ``uint16_t``.

Bitsets can inherit from other bitsets, extending their member set.

.. code-block:: omg-idl

    bitset ParentBitset
    {
        bitfield<3> parent_member;
    };

    bitset ChildBitset : ParentBitset
    {
        bitfield<10> child_member;
    };

In this case, the resulting C++ code will be:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // BITSET_INHERITANCE
   :end-before: //!

Note that in this case, :class:`ChildBitset` will have two ``std::bitset`` data members, one belonging to
:class:`ParentBitset` and the other belonging to :class:`ChildBitset`.

Enumerations
^^^^^^^^^^^^

An enumeration in IDL format is a collection of identifiers that have an associated numeric value.
An IDL enumeration type is mapped directly to the corresponding C++11 enumeration definition.

The following IDL enumeration:

.. code-block:: omg-idl

    enum Enumeration
    {
        RED,
        GREEN,
        @value(3)
        BLUE
    };

Would be converted to:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // ENUMERATION_DATA_TYPE
   :end-before: //!

Bitmasks
^^^^^^^^

Bitmasks are a special kind of Enumeration to manage masks of bits.
It allows defining bit masks based on their position.

The following IDL bitmask:

.. code-block:: omg-idl

    @bit_bound(8)
    bitmask MyBitMask
    {
        @position(0) flag0,
        @position(1) flag1,
        @position(4) flag4,
        @position(6) flag6,
        flag7
    };

Would be converted to:

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // BITMASK_DATA_TYPE
   :end-before: //!

The annotation ``bit_bound`` defines the width of the associated enumeration.
It must be a positive number between 1 and 64.
If omitted, it will be 32 bits.
For each ``flag``, the user can use the annotation ``position`` to define the position of the flag.
If omitted, it will be auto incremented from the last defined flag, starting at 0.

Modules
^^^^^^^

In order to avoid collision between variable names, modules can be defined within the IDL file.
A module would be converted into a namespace in C++.

Data types with a key
^^^^^^^^^^^^^^^^^^^^^

In order to use keyed topics, the user should define some key members inside the structure.
This is achieved by writing the ``@key`` annotation before the members of the structure that are used as keys.
For example in the following IDL file the *id* and *type* field would be the keys:

.. code-block:: omg-idl

    struct MyType
    {
        @key long id;
        @key string type;
        long positionX;
        long positionY;
    };

*Fast DDS-Gen* automatically detects these tags and correctly generates the serialization methods for the key generation
function in TopicDataType (:func:`getKey`).
This function will obtain the 128-bit MD5 digest of the big-endian serialization of the Key Members.

Including other IDL files
-------------------------

Other IDL files can be included in addition to the current IDL file.
*Fast DDS-Gen* uses a C/C++ preprocessor for this purpose, and ``#include`` directive can be used to include an IDL
file.
Preprocessor directives guarding against multiple inclusion of the same IDL file are also advisable.

.. literalinclude:: /../code/FastDDSGenCodeTester.cpp
   :language: c++
   :start-after: // INCLUDE_MORE_IDL_FILES
   :end-before: //!

If *Fast DDS-Gen* does not find a C/C++ preprocessor in default system paths, the preprocessor path can be specified
using parameter ``-ppPath``.
The parameter ``-ppDisable`` can be used to disable the usage of the C/C++ preprocessor.

Annotations
--------------

The application allows the user to define and use their own annotations as defined in the
`OMG IDL specification <https://www.omg.org/spec/IDL/4.2/>`_.

.. code-block:: omg-idl

    @annotation MyAnnotation
    {
        long value;
        string name;
    };

.. _builtin_annotations:

Additionally, the following standard annotations are defined by the specification and considered builtin (these
annotations might be applied without the need of defining them).

.. list-table::
    :header-rows: 1
    :align: left

    * - Builtin Annotation
      - Behavior
      - Supported
    * - :code:`@ami`
      - Asynchronous interface or operation.
      - ❌
    * - :code:`@appendable`
      - Shortcut for :code:`@extensibility(APPENDABLE)`.
      - ✅
    * - :code:`@autoid`
      - Member ID algorithm configuration if there is no member ID explicitly set using :code:`@id` annotation. |br|
        Possible values are :code:`SEQUENTIAL` (member ID is assigned sequentially. Default value) or |br|
        :code:`HASH` (member ID is calculated with an algorithm involving hashing the member name). |br|
        This annotation might be defined in module, structure or union declarations.
      - ✅
    * - :code:`@bit_bound`
      - Set number of bits on `bitmasks`_ and `enumerations`_ underlying primitive type. |br|
        Currently, :code:`@bit_bound` can only be applied to bitmask types.
      - ✅❌
    * - :code:`@data_representation`
      - Specify the |DataRepresentationId-api| required for a specific type.
      - ❌
    * - :code:`@default`
      - Set constant default value to a member.
      - ❌
    * - :code:`@default_literal`
      - Mark an `enumerations`_ literal as default.
      - ❌
    * - :code:`@default_nested`
      - Use in module declaration to mark as :code:`@nested` every element defined within the module.
      - ❌
    * - :code:`@extensibility`
      - Applicable to any constructed element.
        Specify how the element is allowed to evolve. |br|
        Please, refer to Extensibility_ for more information.
      - ✅
    * - :code:`@external`
      - Member is stored in external storage.
      - ✅
    * - :code:`@final`
      - Shortcut for :code:`@extensibility(FINAL)`
      - ✅
    * - :code:`@hashid`
      - Calculate the member ID with the string provided or, if empty, with the member name.
      - ✅
    * - :code:`@id`
      - Assign member ID to a structure or union member.
      - ✅
    * - :code:`@ignore_literal_names`
      - When checking evolved type compatibility, take or not into account member names.
      - ❌
    * - :code:`@key`
      - Mark a structure member as part of the key. :code:`@Key` is also supported. |br|
        Please, refer to :ref:`dds_layer_topic_instances` for more information.
      - ✅
    * - :code:`@max`
      - Set a maximum constant value to the member.
      - ❌
    * - :code:`@min`
      - Set a minimum constant value to the member.
      - ❌
    * - :code:`@must_understand`
      - Mark a structure member as essential for the structure cohesion.
      - ❌
    * - :code:`@mutable`
      - Shortcut for :code:`@extensibility(MUTABLE)`
      - ✅
    * - :code:`@nested`
      - Type is always used within another one.
      - ❌
    * - :code:`@non_serialized`
      - Omit member during serialization.
      - ❌
    * - :code:`@oneway`
      - One-way operation, flowing the information only on one direction.
      - ❌
    * - :code:`@optional`
      - Configure a structure member as optional. Please refer to `Optional Members`_ for more information.
      - ✅
    * - :code:`@position`
      - Set a bitflag position in `bitmasks`_.
      - ✅
    * - :code:`@range`
      - Set a range of allowed values for the member.
      - ❌
    * - :code:`@service`
      - Interface is to be treated as a service.
      - ❌
    * - :code:`@topic`
      - Structure or union is meant to be used as Topic Data Type.
      - ❌
    * - :code:`@try_construct`
      - When checking evolved type compatibility, configure the behavior for |br|
        collection/aggregated types construction and what to do in case of failure.
      - ❌
    * - :code:`@unit`
      - Specify a unit of measurement for the member.
      - ❌
    * - :code:`@value`
      - Set constant value to `enumerations`_ literal.
      - ✅
    * - :code:`@verbatim`
      - Add comment or text to the element.
      - ❌

Forward declaration
-------------------

*Fast DDS-Gen* supports forward declarations.
This allows declaring inter-dependant structures, unions, etc.

.. code-block:: omg-idl

    struct ForwardStruct;

    union ForwardUnion;

    struct ForwardStruct
    {
        ForwardUnion fw_union;
    };

    union ForwardUnion switch (long)
    {
        case 0:
            ForwardStruct fw_struct;
        default:
            string empty;
    };

IDL 4.2 aliases
------------------

IDL 4.2 allows using the following names for primitive types:

+------------------------+
| int8                   |
+------------------------+
| uint8                  |
+------------------------+
| int16                  |
+------------------------+
| uint16                 |
+------------------------+
| int32                  |
+------------------------+
| uint32                 |
+------------------------+
| int64                  |
+------------------------+
| uint64                 |
+------------------------+

IDL 4.2 comments
-----------------

There are two ways to write IDL comments:

* The characters ``/*`` start a comment, which terminates with the characters ``*/``.
* The characters ``//`` start a comment, which terminates at the end of the line on which they occur.

Please refer to the `IDL 4.2 specification <https://www.omg.org/spec/IDL/4.2/PDF>`_ (*Section 7.2 Lexical Conventions*)
for more information on IDL conventions.

.. code-block:: omg-idl

    /* MyStruct definition */
    struct MyStruc
    {
        string mymessage;   // mymessage data member.
    };
