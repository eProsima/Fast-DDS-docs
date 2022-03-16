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

.. code-block:: idl

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

.. code-block:: idl

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

Unions
^^^^^^

In IDL, a union is defined as a sequence of members with their own types and a discriminant that specifies which member
is in use.
An IDL union type is mapped as a C++ class with member functions to access the union members and the discriminant.

The following IDL union:

.. code-block:: idl

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

.. code-block:: idl

    bitset MyBitset
    {
        bitfield<3> a;
        bitfield<10> b;
        bitfield<12, int> c;
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
In the case of bitfield 'c', the user has established that this accessing type will be ``int``, so the generated code
uses ``int32_t`` instead of automatically use ``uint16_t``.

Bitsets can inherit from other bitsets, extending their member set.

.. code-block:: idl

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

.. code-block:: idl

    enum Enumeration
    {
        RED,
        GREEN,
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

.. code-block:: idl

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

Data types with a key
^^^^^^^^^^^^^^^^^^^^^

In order to use keyed topics, the user should define some key members inside the structure.
This is achieved by writing the ``@Key`` annotation before the members of the structure that are used as keys.
For example in the following IDL file the *id* and *type* field would be the keys:

.. code-block:: idl

    struct MyType
    {
        @Key long id;
        @Key string type;
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
`OMG IDL 4.2 specification <https://www.omg.org/spec/IDL/4.2/>`_.
User annotations will be passed to TypeObject generated code if the ``-typeobject`` argument was used.

.. code-block:: idl

    @annotation MyAnnotation
    {
        long value;
        string name;
    };

Additionally, the following standard annotations are builtin (recognized and passed to TypeObject when unimplemented).

+-------------------------+-------------------------------------------------------------------------+
| Annotation              | Implemented behavior                                                    |
+=========================+=========================================================================+
| @id                     | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @autoid                 | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @optional               | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @extensibility          | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @final                  | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @appendable             | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @mutable                | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @position               | Used by bitmasks_.                                                      |
+-------------------------+-------------------------------------------------------------------------+
| @value                  | Allows to set a constant value to any element.                          |
+-------------------------+-------------------------------------------------------------------------+
| @key                    | Alias for eProsima's @Key annotation.                                   |
+-------------------------+-------------------------------------------------------------------------+
| @must_understand        | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @default_literal        | Allows selecting one member as the default within a collection.         |
+-------------------------+-------------------------------------------------------------------------+
| @default                | Allows specifying the default value of the annotated element.           |
+-------------------------+-------------------------------------------------------------------------+
| @range                  | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @min                    | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @max                    | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @unit                   | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @bit_bound              | Allows setting a size to a bitmasks_.                                   |
+-------------------------+-------------------------------------------------------------------------+
| @external               | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @nested                 | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @verbatim               | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @service                | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @oneway                 | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @ami                    | Unimplemented.                                                          |
+-------------------------+-------------------------------------------------------------------------+
| @non_serialized         | The annotated member will be omitted from serialization.                |
+-------------------------+-------------------------------------------------------------------------+

Most unimplemented annotations are related to Extended Types.

Forward declaration
---------------------

*Fast DDS-Gen* supports forward declarations.
This allows declaring inter-dependant structures, unions, etc.

.. code-block:: idl

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

.. code-block:: idl

    /* MyStruct definition */
    struct MyStruc
    {
        string mymessage;   // mymessage data member.
    };
