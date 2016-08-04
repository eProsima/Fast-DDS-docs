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

	+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
	| Option              | Description																	 |
	+=====================+==================================================================================================================================================+
	| -help               | Shows the help information.															 |
	+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
	| -version            | Shows the current version of eProsima FASTRTPSGEN.												 |
	+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
	| -d <directory>      | Output directory where the generated files are created.												 |
	+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
	| -example <platform> |Generates an example and a solution to compile the generated source code for a specific platform. The help command shows the supported platforms. |
	+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+
	| -replace            |Replaces the generated source code files whether they exist.											 |
	+---------------------+--------------------------------------------------------------------------------------------------------------------------------------------------+

Defining a data type via IDL
----------------------------

The following table shows the basic IDL types supported by *fastrtpsgen* and how they are mapped to C++11.

	+---------------------+-------------+
	| IDL                 | C++11       |
	+=====================+=============+
	| char                | char        |
	+---------------------+-------------+
	| octet               | uint8_t     |
	+---------------------+-------------+
	| short               | int16_t     |
	+---------------------+-------------+
	| unsigned short      | uint16_t    |
	+---------------------+-------------+
	|  long long          | int64_t     |
	+---------------------+-------------+
	| unsigned long long  | uint64_t    |
	+---------------------+-------------+
	| float               | float       |
	+---------------------+-------------+
	| double              | double      |
	+---------------------+-------------+
	| boolean             | bool        |
	+---------------------+-------------+
	| string              | std::string |
	+---------------------+-------------+

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
	|  long long a[5]          | std::array<int64_t,5> a  |
	+--------------------------+--------------------------+
	| unsigned long long a[5]  | std::array<uint64_t,5> a |
	+--------------------------+--------------------------+
	| float a[5]               | std::array<float,5> a    |
	+--------------------------+--------------------------+
	| double a[5]              | std::array<double,5> a   |
	+--------------------------+--------------------------+

Sequences
^^^^^^^^^

*fastrtpsgen* fupports sequences, which map into the STD vector container. The following table represents how the map between IDL and C++11 is handled.

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
	|  sequence<long long>          |    std::vector<int64_t>  |
	+-------------------------------+--------------------------+
	| sequence<unsigned long long>  |    std::vector<uint64_t> |
	+-------------------------------+--------------------------+
	| sequence<float>               |    std::vector<float>    |
	+-------------------------------+--------------------------+
	| sequence<double>              |    std::vector<double>   |
	+-------------------------------+--------------------------+

Structures
^^^^^^^^^^

You can define an IDL structure with a set of members with multiple types. It will be converted into a C++ class with each member mapped as an attributes plus method to *get* and *set* each member.

The following IDL structure: ::

	struct Structure
	{
    	octet octet_value;
   	long long_value;
    	string string_value;
	};

Would be converted: ::

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


