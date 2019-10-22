Execution and IDL Definition
============================

Building publisher/subscriber code
----------------------------------
This section guides you through the usage of this Java application and briefly describes the generated files.

Once you added ``scripts`` folder to your ``PATH``,
the Java application can be executed using the following scripts depending on if you are on Windows or Linux: ::

    > fastrtpsgen.bat
    $ fastrtpsgen

In case you didn't modified your ``PATH`` you can find these scripts in your ``<fastrtpsgen_directory>/scripts`` folder.

The expected argument list of the application is: ::

    fastrtpsgen [<options>] <IDL file> [<IDL file> ...]

Where the option choices are:

    +---------------------+-----------------------------------------------------------------------------------------+
    | Option              | Description                                                                             |
    +=====================+=========================================================================================+
    | -help               | Shows the help information.                                                             |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -version            | Shows the current version of eProsima FASTRTPSGEN.                                      |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -d <directory>      | Sets the output directory where the generated files are created.                        |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -I <directory>      | Add directory to preprocessor include paths.                                            |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -t <directory>      | Sets a specific directory as a temporary directory.                                     |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -example <platform> | Generates an example and a solution to compile the generated source code for a specific |
    |                     | platform. The help command shows the supported platforms.                               |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -replace            | Replaces the generated source code files even if they exist.                            |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -ppDisable          | Disables the preprocessor.                                                              |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -ppPath             | Specifies the preprocessor path.                                                        |
    +---------------------+-----------------------------------------------------------------------------------------+
    | -typeobject         | Generates `TypeObject` files for the IDL provided and modifies MyType constructor to    |
    |                     | register the TypeObject representation into the factory.                                |
    +---------------------+-----------------------------------------------------------------------------------------+

For more information about TypeObject representation read :ref:`dynamic-types`.

.. _idl-types:

Defining a data type via IDL
----------------------------

The following table shows the basic IDL types supported by *fastrtpsgen* and how they are mapped to C++11.

    +--------------------+-------------+
    | IDL                | C++11       |
    +====================+=============+
    | char               | char        |
    +--------------------+-------------+
    | octet              | uint8_t     |
    +--------------------+-------------+
    | short              | int16_t     |
    +--------------------+-------------+
    | unsigned short     | uint16_t    |
    +--------------------+-------------+
    | long               | int32_t     |
    +--------------------+-------------+
    | unsigned long      | uint32_t    |
    +--------------------+-------------+
    | long long          | int64_t     |
    +--------------------+-------------+
    | unsigned long long | uint64_t    |
    +--------------------+-------------+
    | float              | float       |
    +--------------------+-------------+
    | double             | double      |
    +--------------------+-------------+
    | long double        | long double |
    +--------------------+-------------+
    | boolean            | bool        |
    +--------------------+-------------+
    | string             | std::string |
    +--------------------+-------------+

Arrays
^^^^^^

*fastrtpsgen* supports unidimensional and multidimensional arrays.
Arrays are always mapped to std::array containers.
The following table shows the array types supported and how they map.

    +-------------------------+--------------------------+
    | IDL                     | C++11                    |
    +=========================+==========================+
    | char a[5]               | std::array<char,5> a     |
    +-------------------------+--------------------------+
    | octet a[5]              | std::array<uint8_t,5> a  |
    +-------------------------+--------------------------+
    | short a[5]              | std::array<int16_t,5> a  |
    +-------------------------+--------------------------+
    | unsigned short a[5]     | std::array<uint16_t,5> a |
    +-------------------------+--------------------------+
    | long a[5]               | std::array<int32_t,5> a  |
    +-------------------------+--------------------------+
    | unsigned long a[5]      | std::array<uint32_t,5> a |
    +-------------------------+--------------------------+
    | long long a[5]          | std::array<int64_t,5> a  |
    +-------------------------+--------------------------+
    | unsigned long long a[5] | std::array<uint64_t,5> a |
    +-------------------------+--------------------------+
    | float a[5]              | std::array<float,5> a    |
    +-------------------------+--------------------------+
    | double a[5]             | std::array<double,5> a   |
    +-------------------------+--------------------------+

Sequences
^^^^^^^^^

*fastrtpsgen* supports sequences, which map into the STD vector container.
The following table represents how the map between IDL and C++11 is handled.

    +------------------------------+--------------------------+
    | IDL                          | C++11                    |
    +==============================+==========================+
    | sequence<char>               |    std::vector<char>     |
    +------------------------------+--------------------------+
    | sequence<octet>              |    std::vector<uint8_t>  |
    +------------------------------+--------------------------+
    | sequence<short>              |    std::vector<int16_t>  |
    +------------------------------+--------------------------+
    | sequence<unsigned short>     |    std::vector<uint16_t> |
    +------------------------------+--------------------------+
    | sequence<long>               |    std::vector<int32_t>  |
    +------------------------------+--------------------------+
    | sequence<unsigned long>      |    std::vector<uint32_t> |
    +------------------------------+--------------------------+
    | sequence<long long>          |    std::vector<int64_t>  |
    +------------------------------+--------------------------+
    | sequence<unsigned long long> |    std::vector<uint64_t> |
    +------------------------------+--------------------------+
    | sequence<float>              |    std::vector<float>    |
    +------------------------------+--------------------------+
    | sequence<double>             |    std::vector<double>   |
    +------------------------------+--------------------------+

Maps
^^^^

*fastrtpsgen* supports maps, which are equivalent to the STD map container.
The equivalence between types is handled in the same way as for sequences_.

    +-------------------------------+---------------------------------+
    | IDL                           | C++11                           |
    +===============================+=================================+
    | map<char, unsigned long long> |    std::map<char, uint64_T>     |
    +-------------------------------+---------------------------------+

Structures
^^^^^^^^^^

You can define an IDL structure with a set of members with multiple types.
It will be converted into a C++ class with each member mapped as an attribute plus methods to *get* and *set* each
member.

The following IDL structure: ::

    struct Structure
    {
        octet octet_value;
        long long_value;
        string string_value;
    };

Would be converted to: ::

    class Structure
    {
    public:
       Structure();
       ~Structure();
       Structure(const Structure &x);
       Structure(Structure &&x);
       Structure& operator=( const Structure &x);
       Structure& operator=(Structure &&x);

       void octet_value(uint8_t _octet_value);
       uint8_t octet_value() const;
       uint8_t& octet_value();
       void long_value(int64_t _long_value);
       int64_t long_value() const;
       int64_t& long_value();
       void string_value(const std::string
          &_string_value);
       void string_value(std::string &&_string_value);
       const std::string& string_value() const;
       std::string& string_value();

    private:
       uint8_t m_octet_value;
       int64_t m_long_value;
       std::string m_string_value;
    };

Structures can inherit from other structures, extending their member set. ::

    struct ParentStruct
    {
        octet parent_member;
    };

    struct ChildStruct : ParentStruct
    {
        long child_member;
    };

In this case, the resulting C++ code will be: ::

    class ParentStruct
    {
        ...
    };

    class ChildStruct : public ParentStruct
    {
        ...
    };

Unions
^^^^^^

In IDL, a union is defined as a sequence of members with their own types and a discriminant that specifies which member
is in use.
An IDL union type is mapped as a C++ class with access functions to the union members and the discriminant.

The following IDL union: ::

    union Union switch(long)
    {
       case 1:
        octet octet_value;
      case 2:
        long long_value;
      case 3:
        string string_value;
    };

Would be converted to: ::

    class Union
    {
    public:
       Union();
       ~Union();
       Union(const Union &x);
       Union(Union &&x);
       Union& operator=(const Union &x);
       Union& operator=(Union &&x);

       void d(int32t __d);
       int32_t _d() const;
       int32_t& _d();

       void octet_value(uint8_t _octet_value);
       uint8_t octet_value() const;
       uint8_t& octet_value();
       void long_value(int64_t _long_value);
       int64_t long_value() const;
       int64_t& long_value();
       void string_value(const std::string
          &_string_value);
       void string_value(std:: string &&_string_value);
       const std::string& string_value() const;
       std::string& string_value();

    private:
       int32_t m__d;
       uint8_t m_octet_value;
       int64_t m_long_value;
       std::string m_string_value;
    };

Bitsets
^^^^^^^

Bitsets are a special kind of structure, which encloses a set of bits. A bitset can represent up to 64 bits.
Each member is defined as *bitfield* and eases the access to a part of the bitset.

For example: ::

    bitset MyBitset
    {
        bitfield<3> a;
        bitfield<10> b;
        bitfield<12, int> c;
    };

The type MyBitset will store a total of 25 bits (3 + 10 + 12) and will require 32 bits in memory
(lowest primitive type to store the bitset's size).

- The bitfield 'a' allows us to access to the first 3 bits (0..2).

- The bitfield 'b' allows us to access to the next 10 bits (3..12).

- The bitfield 'c' allows us to access to the next 12 bits (13..24).

The resulting C++ code will be similar to: ::

    class MyBitset
    {
    public:
        void a(char _a);
        char a() const;

        void b(uint16_t _b);
        uint16_t b() const;

        void c(int32_t _c);
        int32_t c() const;
    private:
        std::bitset<25> m_bitset;
    };

Internally is stored as a std::bitset. For each bitfield, getter and setter methods are generated with the
smaller possible primitive unsigned type to access it. In the case of bitfield 'c', the user has established
that this accessing type will be **int**, so the generated code uses **int32_t** instead of automatically
use **uint16_t**.

Bitsets can inherit from other bitsets, extending their member set. ::

    bitset ParentBitset
    {
        bitfield<3> parent_member;
    };

    bitset ChildBitset : ParentBitset
    {
        bitfield<10> child_member;
    };

In this case, the resulting C++ code will be: ::

    class ParentBitset
    {
        ...
    };

    class ChildBitset : public ParentBitset
    {
        ...
    };

Note that in this case, ChildBitset will have two ``std::bitset`` members, one belonging to ParentBitset and the
other belonging to ChildBitset.

Enumerations
^^^^^^^^^^^^

An enumeration in IDL format is a collection of identifiers that have a numeric value associated.
An IDL enumeration type is mapped directly to the corresponding C++11 enumeration definition.

The following IDL enumeration: ::

    enum Enumeration
    {
        RED,
        GREEN,
        BLUE
    };

Would be converted to: ::

    enum Enumeration : uint32_t
    {
        RED,
        GREEN,
        BLUE
    };

Bitmasks
^^^^^^^^

Bitmasks are a special kind of Enumeration to manage masks of bits. It allows defining bit masks based on their
position.

The following IDL bitmask: ::

    @bit_bound(8)
    bitmask MyBitMask
    {
        @position(0) flag0,
        @position(1) flag1,
        @position(4) flag4,
        @position(6) flag6,
        flag7
    };

Would be converted to: ::

    enum MyBitMask : uint8_t
    {
        flag0 = 0x01 << 0,
        flag1 = 0x01 << 1,
        flag4 = 0x01 << 4,
        flag6 = 0x01 << 6,
        flag7 = 0x01 << 7
    };

The annotation *bit_bound* defines the width of the associated enumeration. It must be a positive number between
1 and 64. If omitted, it will be 32 bits.
For each *flag*, the user can use the annotation *position* to define the position of the flag. If omitted, it will
be auto incremented from the last defined flag, starting at 0.

Keyed Types
^^^^^^^^^^^

In order to use keyed topics, the user should define some key members inside the structure.
This is achieved by writing “@Key” before the members of the structure you want to use as keys.
For example in the following IDL file the *id* and *type* field would be the keys: ::

    struct MyType
    {
        @Key long id;
        @Key string type;
        long positionX;
        long positionY;
    };

*fastrtpsgen* automatically detects these tags and correctly generates the serialization methods for the key generation
function in TopicDataType (`getKey`).
This function will obtain the 128-bit MD5 digest of the big-endian serialization of the Key Members.

Including other IDL files
^^^^^^^^^^^^^^^^^^^^^^^^^

You can include another IDL files in yours in order to use data types defined in them. *fastrtpsgen* uses a C/C++
preprocessor for this purpose, and you can use ``#include`` directive to include an IDL file.

.. code-block:: c

    #include "OtherFile.idl"
    #include <AnotherFile.idl>

If *fastrtpsgen* doesn't find a C/C++ preprocessor in default system paths, you could specify the preprocessor path
using parameter ``-ppPath``.
If you want to disable the usage of the preprocessor, you could use the parameter ``-ppDisable``.


Annotations
^^^^^^^^^^^

The application allows the user to define and use their own annotations as defined in the IDL 4.2 standard.
User annotations will be passed to TypeObject generated code if the ``-typeobject`` argument was used.

::

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

IDL 4.2 aliases
^^^^^^^^^^^^^^^

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

Forward declaration
^^^^^^^^^^^^^^^^^^^

The application allows forward declarations: ::

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

As the example shows, this allows declaring inter-dependant structures, unions, etc.
