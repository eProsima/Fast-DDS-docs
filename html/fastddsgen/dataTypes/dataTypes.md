<!-- {{{ X-Types API -->
<!-- }}} -->

<a id="fastddsgen-idl-datatypes"></a>

<a id="defining-a-data-type-via-idl"></a>

# Defining a data type via IDL

This section describes the data types that can be defined using IDL files, as well as other mechanisms for building
data types using IDL files.

<a id="fastddsgen-supported-idl-types"></a>

<a id="supported-idl-types"></a>

## Supported IDL types

This section describes the IDL types that *Fast DDS-Gen* supports when generating code.
For the types that can be built at runtime, either with the Dynamic Language Binding API, XML Dynamic Type profiles or
IDL parsing, please refer to [Supported Types](../../fastdds/xtypes/language_binding.md#xtypes-supportedtypes).

Be aware that *Fast DDS-Gen* is not case sensitive as it is specified in the
[IDL specification](https://www.omg.org/spec/IDL/4.2/PDF).
To activate case sensitivity use option `-cs` when running *Fast DDS-Gen* (see
[Supported options](../usage/usage.md#fastddsgen-supported-options)).

#### WARNING
Note that IDL files created by ROS 2 are not necessarily compatible with Fast DDS applications, since they are processed in a different manner and can lead to incompatible type definitions.
For a detailed explanation on how to ensure compatibility between ROS 2 and Fast DDS applications, please refer to this [Vulcanexus tutorial](https://docs.vulcanexus.org/en/jazzy/rst/tutorials/core/deployment/dds2vulcanexus/dds2vulcanexus_ros2idl.html).

<a id="idl-primitive-types"></a>

<a id="primitive-types"></a>

### Primitive types

The following table shows the basic IDL types supported by *Fast DDS-Gen* and how they are mapped to C++11.

| IDL                | C++11         |
|--------------------|---------------|
| char               | `char`        |
| octet              | `uint8_t`     |
| short              | `int16_t`     |
| unsigned short     | `uint16_t`    |
| long               | `int32_t`     |
| unsigned long      | `uint32_t`    |
| long long          | `int64_t`     |
| unsigned long long | `uint64_t`    |
| float              | `float`       |
| double             | `double`      |
| long double        | `long double` |
| boolean            | `bool`        |
| string             | `std::string` |

<a id="arrays"></a>

### Arrays

*Fast DDS-Gen* supports unidimensional and multidimensional arrays.
Arrays are always mapped to `std::array` containers.
The following table shows the array types supported and their mapping.

| IDL                     | C++11                      |
|-------------------------|----------------------------|
| char a[5]               | `std::array<char,5> a`     |
| octet a[5]              | `std::array<uint8_t,5> a`  |
| short a[5]              | `std::array<int16_t,5> a`  |
| unsigned short a[5]     | `std::array<uint16_t,5> a` |
| long a[5]               | `std::array<int32_t,5> a`  |
| unsigned long a[5]      | `std::array<uint32_t,5> a` |
| long long a[5]          | `std::array<int64_t,5> a`  |
| unsigned long long a[5] | `std::array<uint64_t,5> a` |
| float a[5]              | `std::array<float,5> a`    |
| double a[5]             | `std::array<double,5> a`   |

<a id="sequences"></a>

### Sequences

*Fast DDS-Gen* supports sequences, which map into the `std::vector` container.
The following table represents how the map between IDL and C++11 is handled.

| IDL                          | C++11                   |
|------------------------------|-------------------------|
| sequence<char>               | `std::vector<char>`     |
| sequence<octet>              | `std::vector<uint8_t>`  |
| sequence<short>              | `std::vector<int16_t>`  |
| sequence<unsigned short>     | `std::vector<uint16_t>` |
| sequence<long>               | `std::vector<int32_t>`  |
| sequence<unsigned long>      | `std::vector<uint32_t>` |
| sequence<long long>          | `std::vector<int64_t>`  |
| sequence<unsigned long long> | `std::vector<uint64_t>` |
| sequence<float>              | `std::vector<float>`    |
| sequence<double>             | `std::vector<double>`   |

<a id="maps"></a>

### Maps

*Fast DDS-Gen* supports maps, which are equivalent to the `std::map` container.
The equivalence between types is handled in the same way as for [sequences](#sequences).

| IDL                           | C++11                      |
|-------------------------------|----------------------------|
| map<char, unsigned long long> | `std::map<char, uint64_T>` |

#### NOTE
Only [Primitive types](#idl-primitive-types) are currently supported.

<a id="structures"></a>

### Structures

You can define an IDL structure with a set of members with multiple types.
It will be converted into a C++ class in which the members of the structure defined via IDL are mapped to private data
members of the class.
Furthermore, `set()` and `get()` member functions are created to access these private data members.

The following IDL structure:

```omg-idl
struct Structure
{
    octet octet_value;
    long long_value;
    string string_value;
};
```

Would be converted to:

```c++
class Structure
{
public:

    Structure();
    ~Structure();
    Structure(
            const Structure& x);
    Structure(
            Structure&& x);
    Structure& operator =(
            const Structure& x);
    Structure& operator =(
            Structure&& x);

    void octet_value(
            uint8_t _octet_value);
    uint8_t octet_value() const;
    uint8_t& octet_value();
    void long_value(
            int64_t _long_value);
    int64_t long_value() const;
    int64_t& long_value();
    void string_value(
            const std::string
            & _string_value);
    void string_value(
            std::string&& _string_value);
    const std::string& string_value() const;
    std::string& string_value();

private:

    uint8_t m_octet_value;
    int64_t m_long_value;
    std::string m_string_value;
};
```

Structures can inherit from other structures, extending their member set.

```omg-idl
struct ParentStruct
{
    octet parent_member;
};

struct ChildStruct : ParentStruct
{
    long child_member;
};
```

In this case, the resulting C++ code will be:

```c++
class ParentStruct
{
    octet parent_member;
};

class ChildStruct : public ParentStruct
{
    long child_member;
};
```

<a id="optional-members"></a>

<a id="id1"></a>

#### Optional members

A member of a structure can be optional.
This is achieved by writing the `@optional` annotation before the member.

```omg-idl
struct StructWithOptionalMember
{
    @optional octet octet_opt;
};
```

An optional member is converted into a template class `eprosima::fastcdr::optional<T>`, where `T` is the member’s
type.

```c++
class StructWithOptionalMember
{
    eprosima::fastcdr::optional<octet> octet_opt;
};
```

Before reading the value of the optional member, it should be checked the optional contains a value using
`has_value()` function.
Accessing a *null* optional throws a `eprosima::fastcdr::exception::BadOptionalAccessException` exception.

```c++
if (octet_opt.has_value())
{
    octet oc = octet_opt.value();
}
```

<a id="extensibility"></a>

<a id="id2"></a>

#### Extensibility

In order to support evolving types without breaking interoperability, the concept of type extensibility is supported by
*Fast DDS-Gen*.
There are three extensibility kinds: *final*, *appendable* and *mutable*.

* *FINAL* extensibility indicates that the type is strictly defined. It is not possible to add members while maintaining
  type assignability.
* *APPENDABLE* extensibility indicates that two types, where one contains all of the members of the other plus
  additional members appended to the end, may remain assignable.
* *MUTABLE* extensibility indicates that two types may differ from one another in the additional, removal, and/or
  transposition of members while remaining assignable.

```omg-idl
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
```

#### NOTE
XCDRv1 encoding algorithm is not able to manage correctly the deserialization of an appendable structure when it is
used as a member of another one.

<a id="unions"></a>

### Unions

In IDL, a union is defined as a sequence of members with their own types and a discriminant that specifies which member
is in use.
An IDL union type is mapped as a C++ class with member functions to access the union members and the discriminant.

The following IDL union:

```omg-idl
union Union switch(long)
{
   case 1:
    octet octet_value;
  case 2:
    long long_value;
  case 3:
    string string_value;
};
```

Would be converted to:

```c++
class Union
{
public:

    Union();
    ~Union();
    Union(
            const Union& x);
    Union(
            Union&& x);
    Union& operator =(
            const Union& x);
    Union& operator =(
            Union&& x);

    void d(
            int32_t __d);
    int32_t _d() const;
    int32_t& _d();

    void octet_value(
            uint8_t _octet_value);
    uint8_t octet_value() const;
    uint8_t& octet_value();
    void long_value(
            int64_t _long_value);
    int64_t long_value() const;
    int64_t& long_value();
    void string_value(
            const std::string
            & _string_value);
    void string_value(
            std:: string&& _string_value);
    const std::string& string_value() const;
    std::string& string_value();

private:

    int32_t m__d;
    uint8_t m_octet_value;
    int64_t m_long_value;
    std::string m_string_value;
};
```

<a id="bitsets"></a>

### Bitsets

Bitsets are a special kind of structure, which encloses a set of bits.
A bitset can represent up to 64 bits.
Each member is defined as *bitfield* and eases the access to a part of the bitset.

For example:

```omg-idl
bitset MyBitset
{
    bitfield<3> a;
    bitfield<10> b;
    bitfield<12, long> c;
};
```

The type `MyBitset` will store a total of 25 bits (3 + 10 + 12) and will require 32 bits in memory
(lowest primitive type to store the bitset’s size).

- The bitfield ‘a’ allows us to access to the first 3 bits (0..2).
- The bitfield ‘b’ allows us to access to the next 10 bits (3..12).
- The bitfield ‘c’ allows us to access to the next 12 bits (13..24).

The resulting C++ code will be similar to:

```c++
class MyBitset
{
public:

    void a(
            char _a);
    char a() const;

    void b(
            uint16_t _b);
    uint16_t b() const;

    void c(
            int32_t _c);
    int32_t c() const;

private:

    std::bitset<25> m_bitset;
};
```

Internally, it is stored as a `std::bitset`.
For each bitfield, `get()` and `set()` member functions are generated with the smaller possible primitive
unsigned type to access it.
In the case of bitfield ‘c’, the user has established that this accessing type will be `long`, so the generated code
uses `int32_t` instead of automatically use `uint16_t`.

Bitsets can inherit from other bitsets, extending their member set.

```omg-idl
bitset ParentBitset
{
    bitfield<3> parent_member;
};

bitset ChildBitset : ParentBitset
{
    bitfield<10> child_member;
};
```

In this case, the resulting C++ code will be:

```c++
class ParentBitset
{
    std::bitset<3> parent_member;
};

class ChildBitset : public ParentBitset
{
    std::bitset<10> child_member;
};
```

Note that in this case, `ChildBitset` will have two `std::bitset` data members, one belonging to
`ParentBitset` and the other belonging to `ChildBitset`.

<a id="enumerations"></a>

### Enumerations

An enumeration in IDL format is a collection of identifiers that have an associated numeric value.
An IDL enumeration type is mapped directly to the corresponding C++11 enumeration definition.

The following IDL enumeration:

```omg-idl
enum Enumeration
{
    RED,
    GREEN,
    @value(3)
    BLUE
};
```

Would be converted to:

```c++
enum Enumeration : int32_t
{
    RED,
    GREEN,
    BLUE = 3
};
```

<a id="bitmasks"></a>

### Bitmasks

Bitmasks are a special kind of Enumeration to manage masks of bits.
It allows defining bit masks based on their position.

The following IDL bitmask:

```omg-idl
@bit_bound(8)
bitmask MyBitMask
{
    @position(0) flag0,
    @position(1) flag1,
    @position(4) flag4,
    @position(6) flag6,
    flag7
};
```

Would be converted to:

```c++
enum MyBitMask : uint8_t
{
    flag0 = 0x01 << 0,
    flag1 = 0x01 << 1,
    flag4 = 0x01 << 4,
    flag6 = 0x01 << 6,
    flag7 = 0x01 << 7
};
```

The annotation `bit_bound` defines the width of the associated enumeration.
It must be a positive number between 1 and 64.
If omitted, it will be 32 bits.
For each `flag`, the user can use the annotation `position` to define the position of the flag.
If omitted, it will be auto incremented from the last defined flag, starting at 0.

<a id="modules"></a>

### Modules

In order to avoid collision between variable names, modules can be defined within the IDL file.
A module would be converted into a namespace in C++.

<a id="data-types-with-a-key"></a>

### Data types with a key

In order to use keyed topics, the user should define some key members inside the structure.
This is achieved by writing the `@key` annotation before the members of the structure that are used as keys.
For example in the following IDL file the *id* and *type* field would be the keys:

```omg-idl
struct MyType
{
    @key long id;
    @key string type;
    long positionX;
    long positionY;
};
```

*Fast DDS-Gen* automatically detects these tags and correctly generates the serialization methods for the key generation
function in TopicDataType (`getKey()`).
This function will obtain the 128-bit MD5 digest of the big-endian serialization of the Key Members.

<a id="including-other-idl-files"></a>

## Including other IDL files

Other IDL files can be included in addition to the current IDL file.
*Fast DDS-Gen* uses a C/C++ preprocessor for this purpose, and `#include` directive can be used to include an IDL
file.
Preprocessor directives guarding against multiple inclusion of the same IDL file are also advisable.

```c++
 #include "OtherFile.idl"
 #include <AnotherFile.idl>
 #include <IncludedIDL.idl>
```

If *Fast DDS-Gen* does not find a C/C++ preprocessor in default system paths, the preprocessor path can be specified
using parameter `-ppPath`.
The parameter `-ppDisable` can be used to disable the usage of the C/C++ preprocessor.

The parameter `-extrastg` can also be used to apply custom templates to included IDL files.
To enable this feature, the output file name passed to `-extrastg` must include the character `@`, which will be
replaced by the name of the included file to generate the output file.

The following command will generate two custom templates, one for the main IDL file and another for the included one:

- `MainIDL_Custom.cpp`
- `IncludedIDL_Custom.cpp`

Where `IncludedIDL.idl` is included in `MainIDL.idl` file.

```bash
<path/to/Fast DDS-Gen>/scripts/fastddsgen MainIDL.idl -I <path/to/idls> -extrastg <path/to/template>/Custom.stg @_Custom.cpp
```

Check [Supported options](../usage/usage.md#fastddsgen-supported-options) for more information about the `-extrastg` option.

<a id="annotations"></a>

## Annotations

The application allows the user to define and use their own annotations as defined in the
[OMG IDL specification](https://www.omg.org/spec/IDL/4.2/).

```omg-idl
@annotation MyAnnotation
{
    long value;
    string name;
};
```

<a id="builtin-annotations"></a>

Additionally, the following standard annotations are defined by the specification and considered builtin (these
annotations might be applied without the need of defining them).

| Builtin Annotation      | Behavior                                                                                                                                                                                                                                                                                                                                                                 | Supported   |
|-------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------|
| `@ami`                  | Asynchronous interface or operation.                                                                                                                                                                                                                                                                                                                                     | ❌           |
| `@appendable`           | Shortcut for `@extensibility(APPENDABLE)`.                                                                                                                                                                                                                                                                                                                               | ✅           |
| `@autoid`               | Member ID algorithm configuration if there is no member ID explicitly set using `@id` annotation.<br/>Possible values are `SEQUENTIAL` (member ID is assigned sequentially. Default value) or<br/>`HASH` (member ID is calculated with an algorithm involving hashing the member name).<br/>This annotation might be defined in module, structure or union declarations. | ✅           |
| `@bit_bound`            | Set number of bits on [bitmasks](#bitmasks) and [enumerations](#enumerations) underlying primitive type.<br/>Currently, `@bit_bound` can only be applied to bitmask types.                                                                                                                                                                                               | ⚠️          |
| `@data_representation`  | Specify the [`DataRepresentationId`](../../fastdds/api_reference/dds_pim/core/policy/datarepresentationid.md#_CPPv4N8eprosima7fastdds3dds20DataRepresentationIdE) required for a specific type.                                                                                                                                                                          | ❌           |
| `@default`              | Set constant default value to a member.                                                                                                                                                                                                                                                                                                                                  | ❌           |
| `@default_literal`      | Mark an [enumerations](#enumerations) literal as default.                                                                                                                                                                                                                                                                                                                | ❌           |
| `@default_nested`       | Use in module declaration to mark as `@nested` every element defined within the module.                                                                                                                                                                                                                                                                                  | ❌           |
| `@extensibility`        | Applicable to any constructed element.<br/>Specify how the element is allowed to evolve.<br/>Please, refer to [Extensibility](#extensibility) for more information.                                                                                                                                                                                                      | ✅           |
| `@external`             | Member is stored in external storage.                                                                                                                                                                                                                                                                                                                                    | ✅           |
| `@final`                | Shortcut for `@extensibility(FINAL)`                                                                                                                                                                                                                                                                                                                                     | ✅           |
| `@hashid`               | Calculate the member ID with the string provided or, if empty, with the member name.                                                                                                                                                                                                                                                                                     | ✅           |
| `@id`                   | Assign member ID to a structure or union member.                                                                                                                                                                                                                                                                                                                         | ✅           |
| `@ignore_literal_names` | When checking evolved type compatibility, take or not into account member names.                                                                                                                                                                                                                                                                                         | ❌           |
| `@key`                  | Mark a structure member as part of the key. `@Key` is also supported.<br/>Please, refer to [Topics, keys and instances](../../fastdds/dds_layer/topic/instances.md#dds-layer-topic-instances) for more information.                                                                                                                                                      | ✅           |
| `@max`                  | Set a maximum constant value to the member.                                                                                                                                                                                                                                                                                                                              | ❌           |
| `@min`                  | Set a minimum constant value to the member.                                                                                                                                                                                                                                                                                                                              | ❌           |
| `@must_understand`      | Mark a structure member as essential for the structure cohesion.                                                                                                                                                                                                                                                                                                         | ❌           |
| `@mutable`              | Shortcut for `@extensibility(MUTABLE)`                                                                                                                                                                                                                                                                                                                                   | ✅           |
| `@nested`               | Type is always used within another one.                                                                                                                                                                                                                                                                                                                                  | ❌           |
| `@non_serialized`       | Omit member during serialization.                                                                                                                                                                                                                                                                                                                                        | ❌           |
| `@oneway`               | One-way operation, flowing the information only on one direction.                                                                                                                                                                                                                                                                                                        | ❌           |
| `@optional`             | Configure a structure member as optional. Please refer to [Optional Members](#id1) for more information.                                                                                                                                                                                                                                                                 | ✅           |
| `@position`             | Set a bitflag position in [bitmasks](#bitmasks).                                                                                                                                                                                                                                                                                                                         | ✅           |
| `@range`                | Set a range of allowed values for the member.                                                                                                                                                                                                                                                                                                                            | ❌           |
| `@service`              | Interface is to be treated as a service.                                                                                                                                                                                                                                                                                                                                 | ❌           |
| `@feed`                 | Used in interface operations to indicate data streaming (see [Defining an IDL interface](../interfaces/interfaces.md#fastddsgen-interfaces-definition)).                                                                                                                                                                                                                 | ✅           |
| `@topic`                | Structure or union is meant to be used as Topic Data Type.                                                                                                                                                                                                                                                                                                               | ❌           |
| `@try_construct`        | When checking evolved type compatibility, configure the behavior for<br/>collection/aggregated types construction and what to do in case of failure.                                                                                                                                                                                                                     | ❌           |
| `@unit`                 | Specify a unit of measurement for the member.                                                                                                                                                                                                                                                                                                                            | ❌           |
| `@value`                | Set constant value to [enumerations](#enumerations) literal.                                                                                                                                                                                                                                                                                                             | ✅           |
| `@verbatim`             | Add comment or text to the element.                                                                                                                                                                                                                                                                                                                                      | ❌           |

<a id="forward-declaration"></a>

## Forward declaration

*Fast DDS-Gen* supports forward declarations.
This allows declaring inter-dependant structures, unions, etc.

```omg-idl
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
```

<a id="idl-4-2-aliases"></a>

## IDL 4.2 aliases

IDL 4.2 allows using the following names for primitive types:

* `int8`
* `uint8`
* `int16`
* `uint16`
* `int32`
* `uint32`
* `int64`
* `uint64`

<a id="idl-4-2-comments"></a>

## IDL 4.2 comments

There are two ways to write IDL comments:

* The characters `/*` start a comment, which terminates with the characters `*/`.
* The characters `//` start a comment, which terminates at the end of the line on which they occur.

Please refer to the [IDL 4.2 specification](https://www.omg.org/spec/IDL/4.2/PDF) (*Section 7.2 Lexical Conventions*)
for more information on IDL conventions.

```omg-idl
/* MyStruct definition */
struct MyStruc
{
    string mymessage;   // mymessage data member.
};
```
