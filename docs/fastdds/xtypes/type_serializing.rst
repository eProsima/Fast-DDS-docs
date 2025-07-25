.. include:: ../../03-exports/aliases-api.include

.. _xtypes_serialization_utilities:

Serialization Utilities
=======================

Fast DDS provides methods to serialize `XTypes` objects, enabling efficient data exchange and manipulation
within distributed systems.
Serialization is a crucial process in data distribution services, as it converts complex data structures into a
format that can be easily transmitted and reconstructed across different platforms and programming environments.

.. _xtypes_serialization_utilities_idl:

Dynamic Type to IDL
-------------------

The method |XTypesUtils-idl_serialize-api| serializes a |DynamicType-api| object to its IDL representation.

.. note::

    The conversion to IDL only supports the following :ref:`builtin annotations<builtin_annotations>`:
    :code:`@bit_bound`, :code:`@extensibility`, :code:`@key`, and :code:`@position`.

.. warning::

    The conversion to IDL of a :ref:`Bitset<xtypes_supportedtypes_bitset>` with inheritance merges derived
    :ref:`Bitsets<xtypes_supportedtypes_bitset>` with their base :ref:`Bitset<xtypes_supportedtypes_bitset>`.

.. warning::

    The conversion to IDL dismisses values explicitly set to their default value.
    For example, the default :code:`@bit_bound` value of a :ref:`Bitmask<xtypes_supportedtypes_bitmask>` is 32.
    If a user were to explicitly set the :code:`@bit_bound` value of a
    :ref:`Bitmask<xtypes_supportedtypes_bitmask>` to 32 and then serialize the |DynamicType-api| to IDL, the
    :code:`@bit_bound` would not be included in the IDL.

.. _xtypes_serialization_utilities_idl_example:

Example: Convert a discovered type to IDL format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following example demonstrates how to use the |XTypesUtils-idl_serialize-api| method in Fast DDS to convert
discovered types to IDL format.
Each time the subscriber discovers a new |DataReader-api| or |DataWriter-api|, it uses the
|DynamicTypeBuilderFactory-api| to build a |DynamicType-api| and serialize it to IDL format.
Please refer to :ref:`use-case-remote-type-discovery-and-matching` section for more details on how to implement
remote type discovery.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //!--DYNTYPE_IDL_SERIALIZATION
    :end-before: //!--
    :dedent: 4

.. _xtypes_serialization_utilities_dyndata_json:

DynamicData to JSON
-------------------

In the context of DDS (Data Distribution Service), |DynamicType-api| represents the structure of the data being
distributed across the system.
Each |DynamicData-api| object corresponds to an object of the type represented by its |DynamicType-api|,
providing functionalities to access and modify data values.
To enhance interoperability and readability, it is often useful to serialize |DynamicData-api| into a more
manageable format, to enable easier data processing and analysis across different systems and applications.
The method |XTypesUtils-json_serialize-api| converts a |DynamicData-api| object into a JSON object, then
dumped into a ``std::ostream``.
The inverse conversion is also possible using the method |XTypesUtils-json_deserialize-api|, as described in the
:ref:`corresponding section<xtypes_serialization_utilities_json_dyndata>`.

Supported Types
^^^^^^^^^^^^^^^

This section provides the serialization of |DynamicData-api| to JSON ostream for all supported types.

.. _xtypes_serialization_utilities_primitive:

Primitives
""""""""""

Primitive types are the basic building blocks for every |DynamicType-api|.
Below is an example of the definition of primitive types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_PRIMITIVES
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Primitives.json
    :language: json

.. _xtypes_serialization_utilities_string:

Strings
"""""""

String types are used to represent sequences of characters, which are essential for handling textual
data within the system.
The following example shows the definition of string types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_STRINGS
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Strings.json
    :language: json

.. _xtypes_serialization_utilities_enumeration:

Enumerations
""""""""""""

Enumeration types represent a fixed set of named values, making it easier to work with a predefined
list of options.
Below is an example of the definition of enumeration types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_ENUM
    :end-before: //!--

The previous DynamicData object can be serialized in two different formats, `eProsima` and `OMG`, providing
flexibility depending on the required interoperability and compatibility with other systems.
The previous |DynamicData-api| object would be serialized as follows in the different formats:

.. tab-set::

    .. tab-item:: eProsima
       :sync: eprosima

       .. literalinclude:: /../code/json/Enum_EPROSIMA.json
           :language: json

    .. tab-item:: OMG
       :sync: omg

       .. literalinclude:: /../code/json/Enum_OMG.json
           :language: json

.. _xtypes_serialization_utilities_bitmask:

Bitmasks
""""""""

Bitmask types allow the representation of a set of flags in a single value, which can be useful
for storing multiple boolean options compactly.
Here is an example of the definition of bitmask types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_BITMASK
    :end-before: //!--

Bitmask also present different serialized structures in the different formats `eProsima` and `OMG`.
The previous |DynamicData-api| object would be serialized as follows in the different formats:

.. tab-set::

    .. tab-item:: eProsima
       :sync: eprosima

       .. literalinclude:: /../code/json/Bitmask_EPROSIMA.json
           :language: json

    .. tab-item:: OMG
       :sync: omg

       .. literalinclude:: /../code/json/Bitmask_OMG.json
           :language: json

.. _xtypes_serialization_utilities_sequence:

Sequences
"""""""""

Sequence types are used to represent ordered collections of elements, similar to arrays but with
dynamic length.
Below is an example of the definition of sequence types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_SEQUENCES
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Sequences.json
    :language: json

.. _xtypes_serialization_utilities_array:

Arrays
""""""

Array types are used to represent fixed-size collections of elements.
The following example shows the definition of array types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_ARRAYS_JSON
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Arrays.json
    :language: json

.. _xtypes_serialization_utilities_map:

Maps
""""

Map types represent collections of key-value pairs, allowing for the efficient lookup of values based
on unique keys.
Below is an example of the definition of map types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_MAPS
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Maps.json
    :language: json

.. _xtypes_serialization_utilities_structure:

Structures
""""""""""

Structure types are used to group different types of data together.
Here is an example of the definition of structure types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_STRUCT
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Structs.json
    :language: json

.. _xtypes_serialization_utilities_union:

Unions
""""""

Union types are a special type of structure type where only one member exists.
Below is an example of the definition of union types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_UNION
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Unions.json
    :language: json

.. _xtypes_serialization_utilities_bitsets:

Bitsets
"""""""

Bitset types are a specialized type of structure where individual bits can be accessed and manipulated.
Below is an example of the definition of bitset types in IDL:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_BITSET_JSON
    :end-before: //!--

A |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Bitsets.json
    :language: json

.. _xtypes_serialization_utilities_dyndata_json_example:

Example: Convert received data into JSON format
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following code demonstrates how to use the |XTypesUtils-json_serialize-api| function in Fast DDS to serialize
received data into a more manageable and understandable JSON format.
Each time the subscriber receives new data, the corresponding |DynamicData-api| can be obtained from the
|DynamicDataFactory-api| and serialized into a JSON string format.
Please refer to :ref:`use-case-remote-type-discovery-and-matching` section for more details on how to implement
remote type discovery.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 4
    :start-after: //!--DYNDATA_JSON_SERIALIZATION
    :end-before: //!--

.. _xtypes_serialization_utilities_json_dyndata:

JSON to DynamicData
-------------------

Apart from having the possibility to serialize :ref:`DynamicData to JSON<xtypes_serialization_utilities_dyndata_json>`,
Fast DDS also provides a way to perform the inverse conversion.
The method |XTypesUtils-json_deserialize-api| is able to convert a JSON object into a |DynamicData-api| instance,
by only providing its associated |DynamicType-api|.
This method is useful for injecting data from external sources into a DDS network, allowing for the integration of data
from various systems and applications.

.. _xtypes_serialization_utilities_json_dyndata_example:

Example: Convert JSON data into DynamicData
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following code demonstrates how to use the |XTypesUtils-json_deserialize-api| function in Fast DDS to convert
JSON data into a |DynamicData-api| object.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :dedent: 4
    :start-after: //!--JSON_DYNDATA_DESERIALIZATION
    :end-before: //!--
