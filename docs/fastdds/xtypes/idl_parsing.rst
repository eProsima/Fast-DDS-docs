.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _dynamic-types-idl-parsing:

Dynamic Types IDL Parsing
=========================

*Fast DDS* supports the implementation of |DynamicTypes| for parsing IDL files at runtime to generate dynamic
data types.
The |DynamicTypeBuilderFactory::create_type_w_uri| API allows users to provide a URI pointing to an IDL file.
Fast DDS will process the file and return the corresponding DynamicTypeBuilder, which can then be used to create a
DynamicType.

This feature enables applications to dynamically load type definitions from IDL files instead of relying solely on
pre-generated types or XML profiles.
This enhances flexibility, as new data types can be introduced without modifying and recompiling the application.

Type definition
^^^^^^^^^^^^^^^
Below, the types supported by *eProsima Fast DDS* IDL parsing are presented.
For further information about the supported |DynamicTypes|, please, refer to :ref:`xtypes_supportedtypes`.

* :ref:`xtypes_supportedtypes_primitive`
* :ref:`xtypes_supportedtypes_string`
* :ref:`xtypes_supportedtypes_structure`
* :ref:`xtypes_supportedtypes_alias`
* :ref:`xtypes_supportedtypes_array`
* :ref:`xtypes_supportedtypes_union`
* :ref:`xtypes_supportedtypes_enumeration`
* :ref:`xtypes_annotations`
* Arithmetic expressions
* Union/struct forward declarations

The following types are currently not supported by the IDL parsing feature:

* :ref:`xtypes_supportedtypes_bitmask`
* :ref:`xtypes_supportedtypes_sequence`
* :ref:`xtypes_supportedtypes_map`
* :ref:`xtypes_supportedtypes_bitset`
* :ref:`xtypes_builtin_annotations`
* Module
* Inheritance
* Member ID

Example
^^^^^^^

*Fast DDS* application can use dynamic types generated in runtime just by loading the corresponding IDL files into the
|DynamicTypeBuilderFactory-api| using |DynamicTypeBuilderFactory::create_type_w_uri|.
After getting the DynamicType, objects of |DynamicPubSubType-api| class might be instantiated and used to write/read
data.

.. note::

    The preprocessor can be manually selected using |DynamicTypeBuilderFactory::set_preprocessor|

The following snippet shows the previously explained steps:

.. literalinclude:: /../code/DDSCodeTester.cpp
     :language: c++
     :start-after: //DDS_DYNAMIC_TYPES_IDL_PARSING
     :end-before: //!--
