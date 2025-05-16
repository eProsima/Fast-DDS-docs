#include <stdint.h>
#include <string>
#include <bitset>

#include <fastcdr/config.h>

#if FASTCDR_VERSION_MAJOR > 1

#include <fastcdr/xcdr/optional.hpp>

using octet = unsigned char;

// STRUCTURES_DATA_TYPE
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
//!

// STRUCTURE_INHERITANCE
class ParentStruct
{
    octet parent_member;
};

class ChildStruct : public ParentStruct
{
    long child_member;
};
//!

// STRUCTURE_WITH_OPTIONAL
class StructWithOptionalMember
{
    eprosima::fastcdr::optional<octet> octet_opt;
};
//!

void accessing_optional_value()
{
    eprosima::fastcdr::optional<octet> octet_opt;

    // ACCESSING_OPTIONAL_VALUE
    if (octet_opt.has_value())
    {
        octet oc = octet_opt.value();
    }
    //!
}

// UNION_DATA_TYPE
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
//!

// BITSET_DATA_TYPE
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
//!

// BITSET_INHERITANCE
class ParentBitset
{
    std::bitset<3> parent_member;
};

class ChildBitset : public ParentBitset
{
    std::bitset<10> child_member;
};
//!

// ENUMERATION_DATA_TYPE
enum Enumeration : uint32_t
{
    RED,
    GREEN,
    BLUE
};
//!

// BITMASK_DATA_TYPE
enum MyBitMask : uint8_t
{
    flag0 = 0x01 << 0,
    flag1 = 0x01 << 1,
    flag4 = 0x01 << 4,
    flag6 = 0x01 << 6,
    flag7 = 0x01 << 7
};
//!

#endif // FASTCDR_VERSION_MAJOR > 1

/*
   // INCLUDE_MORE_IDL_FILES
 #include "OtherFile.idl"
 #include <AnotherFile.idl>
 #include <IncludedIDL.idl>
   //!
   /**/
