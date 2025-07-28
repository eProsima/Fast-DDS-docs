.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dynamic-types-idl-parsing:

Dynamic Types IDL Parsing
=========================

*Fast DDS* supports the implementation of |DynamicTypes| for parsing IDL files at runtime to generate dynamic
data types.

This feature enables applications to dynamically load type definitions from IDL files instead of relying solely on
pre-generated types or XML profiles.
This enhances flexibility, as new data types can be introduced without modifying and recompiling the application.

Supported Types
---------------

Below, the types supported by *eProsima Fast DDS* IDL parsing are presented.
For further information about the supported |DynamicTypes|, please, refer to :ref:`xtypes_supportedtypes`.

* :ref:`xtypes_supportedtypes_primitive`
* :ref:`xtypes_supportedtypes_string`
* :ref:`xtypes_supportedtypes_structure`
* :ref:`xtypes_supportedtypes_alias`
* :ref:`xtypes_supportedtypes_array`
* :ref:`xtypes_supportedtypes_sequence`
* :ref:`xtypes_supportedtypes_union`
* :ref:`xtypes_supportedtypes_enumeration`
* Arithmetic expressions
* Union/struct forward declarations
* Modules/scoped types

The following types are currently not supported by the IDL parsing feature:

* :ref:`xtypes_supportedtypes_bitmask`
* :ref:`xtypes_supportedtypes_map`
* :ref:`xtypes_supportedtypes_bitset`
* :ref:`xtypes_annotations`
* Inheritance
* Member ID

Create a Dynamic Type from a IDL file
-------------------------------------

*Fast DDS* application can use dynamic types generated in runtime just by loading the corresponding IDL files into the
|DynamicTypeBuilderFactory-api| using |DynamicTypeBuilderFactory::create_type_w_uri|.
After getting the DynamicType, objects of |DynamicPubSubType-api| class might be instantiated and used to write/read
data.

.. warning::

    |DynamicTypeBuilderFactory::create_type_w_uri| requires the fully qualified name of the
    type defined in the IDL file. If no scope is provided, global scope is assumed.

.. note::

    The preprocessor can be manually selected using |DynamicTypeBuilderFactory::set_preprocessor|

Example
^^^^^^^

The following snippet shows the previously explained steps:

.. literalinclude:: /../code/DDSCodeTester.cpp
     :language: c++
     :dedent: 8
     :start-after: //DDS_DYNAMIC_TYPES_IDL_PARSING_CREATE_TYPE
     :end-before: //!--


Iterate over the parsed IDL types
---------------------------------

*Fast DDS* application supports a simple way to iterate over each aggregated type declared and parsed in the IDL file,
*i.e*:

* :ref:`xtypes_supportedtypes_union`
* :ref:`xtypes_supportedtypes_enumeration`
* :ref:`xtypes_supportedtypes_structure`
* :ref:`xtypes_supportedtypes_alias`

using |DynamicTypeBuilderFactory::for_each_type_w_uri| method.
User can customize the runtime behavior by providing a lambda expression, which will be triggered
when a new aggregated type is parsed.

The callback provided by the user receives the |DynamicTypeBuilder-api| instance related to the parsed type
and returns a boolean value, indicating whether the iteration should continue or not.
In case of returning false for a given type,
the parsing process will stop and no further types will be processed.

Example
^^^^^^^

The following snippet shows how to iterate over the aggregated types declared in the IDL file,
storing their names in a vector:

.. literalinclude:: /../code/DDSCodeTester.cpp
     :language: c++
     :dedent: 8
     :start-after: //DDS_DYNAMIC_TYPES_IDL_PARSING_ITERATE_OVER_TYPES
     :end-before: //!--
