Execution and IDL Definition
============================

Building publisher/subscriber code
----------------------------------
This section guides you through the usage of this Java application and briefly describes the generated files.

The Java application can be executed using the following scripts depending on if you are on Windows or Linux: ::

    > fastrtpsgen.bat
    $ fastrtpsgen

The expected argument list of the application is: ::

    fastrtpsgen [<options>] <IDL file> [<IDL file> ...]

Where the option choices are:

    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | Option              | Description                                                                                                                                       |
    +=====================+===================================================================================================================================================+
    | -help               | Shows the help information.                                                                                                                       |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -version            | Shows the current version of eProsima FASTRTPSGEN.                                                                                                |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -d <directory>      | Sets the output directory where the generated files are created.                                                                                  |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -I <directory>      | Add directory to preprocessor include paths.                                                                                                      |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -t <directory>      | Sets a specific directory as a temporary directory.                                                                                               |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -example <platform> | Generates an example and a solution to compile the generated source code for a specific platform. The help command shows the supported platforms. |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -replace            | Replaces the generated source code files even if they exist.                                                                                      |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -ppDisable          | Disables the preprocessor.                                                                                                                        |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+
    | -ppPath             | Specifies the preprocessor path.                                                                                                                  |
    +---------------------+---------------------------------------------------------------------------------------------------------------------------------------------------+

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
    |  long long         | int64_t     |
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

*fastrtpsgen* supports unidimensional and multidimensional arrays. Arrays are always mapped to std::array containers. The following table shows the array types supported and how they map.

	+--------------------------+--------------------------+
	| IDL                      | C++11                    |
	+==========================+==========================+
	| char a[5]                | std::array<char,5> a     |
	+--------------------------+--------------------------+
	| octet a[5]               | std::array<uint8_t,5> a  |
	+--------------------------+--------------------------+
	| short a[5]               | std::array<int16_t,5> a  |
	+--------------------------+--------------------------+
	| unsigned short a[5]      | std::array<uint16_t,5> a |
	+--------------------------+--------------------------+
	| long long a[5]           | std::array<int64_t,5> a  |
	+--------------------------+--------------------------+
	| unsigned long long a[5]  | std::array<uint64_t,5> a |
	+--------------------------+--------------------------+
	| float a[5]               | std::array<float,5> a    |
	+--------------------------+--------------------------+
	| double a[5]              | std::array<double,5> a   |
	+--------------------------+--------------------------+

Sequences
^^^^^^^^^

*fastrtpsgen* supports sequences, which map into the STD vector container. The following table represents how the map between IDL and C++11 is handled.

	+-------------------------------+--------------------------+
	| IDL                           | C++11                    |
	+===============================+==========================+
	| sequence<char>                |    std::vector<char>     |
	+-------------------------------+--------------------------+
	| sequence<octet>               |    std::vector<uint8_t>  |
	+-------------------------------+--------------------------+
	| sequence<short>               |    std::vector<int16_t>  |
	+-------------------------------+--------------------------+
	| sequence<unsigned short>      |    std::vector<uint16_t> |
	+-------------------------------+--------------------------+
	| sequence<long long>           |    std::vector<int64_t>  |
	+-------------------------------+--------------------------+
	| sequence<unsigned long long>  |    std::vector<uint64_t> |
	+-------------------------------+--------------------------+
	| sequence<float>               |    std::vector<float>    |
	+-------------------------------+--------------------------+
	| sequence<double>              |    std::vector<double>   |
	+-------------------------------+--------------------------+

Structures
^^^^^^^^^^

You can define an IDL structure with a set of members with multiple types. It will be converted into a C++ class with each member mapped as an attribute plus methods to *get* and *set* each member.

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

Unions
^^^^^^

In IDL, a union is defined as a sequence of members with their own types and a discriminant that specifies which member is in use. An IDL union type is mapped as a C++ class with access functions to the union members and the discriminant.

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

Enumerations
^^^^^^^^^^^^

An enumeration in IDL format is a collection of identifiers that have a numeric value associated. An IDL enumeration type is mapped directly to the corresponding C++11 enumeration definition. 

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

Keyed Types
^^^^^^^^^^^

In order to use keyed topics the user should define some key members inside the structure. This is achieved by writing “@Key” before the members of the structure you want to use as keys. 
For example in the following IDL file the *id* and *type* field would be the keys: ::

	struct MyType
	{
	    @Key long id;
	    @Key string type;
	    long positionX;
	    long positionY;
	};

*fastrtpsgen* automatically detects these tags and correctly generates the serialization methods for the key generation function in TopicDataType (getKey). This function will obtain the 128-bit MD5 digest of the big-endian serialization of the Key Members.

Including other IDL files
^^^^^^^^^^^^^^^^^^^^^^^^^

You can include another IDL files in yours in order to use data types defined in them. *fastrtpsgen* uses a C/C++
preprocessor for this purpose, and you can use ``#include`` directive to include an IDL file.

.. code-block:: c++

    #include "OtherFile.idl"
    #include <AnotherFile.idl>

If *fastrtpsgen* doesn't find a C/C++ preprocessor in default system paths, you could specify the preprocessor path using
parameter ``-ppPath``. If you want to disable the usage of preprocessor, you could use the parameter ``-ppDisable``.
