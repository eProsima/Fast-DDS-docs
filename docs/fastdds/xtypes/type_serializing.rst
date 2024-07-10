.. include:: ../../03-exports/aliases-api.include

.. _xtypes_serialization_utilities:

Serialization Utilities
=======================

Fast DDS provides methods to serialize `XTypes` objects, enabling efficient data exchange and manipulation
within distributed systems.
Serialization is a crucial process in data distribution services, as it converts complex data structures into a
format that can be easily transmitted and reconstructed across different platforms and programming environments.

DynamicData to JSON
--------------------

In the context of DDS (Data Distribution Service), |DynamicType-api| represents the structure of the data being
distributed across the system.
Each |DynamicData-api| object corresponds to an object of the type represented by its |DynamicType-api|,
providing functionalities to access and modify data values.
To enhance interoperability and readability, it is often useful to serialize |DynamicData-api| into a more
manageable format, to enable easier data processing and analysis across different systems and applications.
The method |XTypesUtils-json_serialize-api| converts a |DynamicData-api| object into a JSON object, then
dumped into a ``std::ostream``.

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

The previous |DynamicData-api| object corresponding to the type represented above
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

The previous |DynamicData-api| object corresponding to the type represented above
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

.. tabs::

    .. tab:: eProsima

        .. literalinclude:: /../code/json/Enum_EPROSIMA.json
            :language: json

    .. tab:: OMG

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

.. tabs::

    .. tab:: eProsima

        .. literalinclude:: /../code/json/Bitmask_EPROSIMA.json
            :language: json

    .. tab:: OMG

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

The previous |DynamicData-api| object corresponding to the type represented above
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

The previous |DynamicData-api| object corresponding to the type represented above
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

The previous |DynamicData-api| object corresponding to the type represented above
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

The previous |DynamicData-api| object corresponding to the type represented above
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

The previous |DynamicData-api| object corresponding to the type represented above
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

The previous |DynamicData-api| object corresponding to the type represented above
would be serialized as follows:

.. literalinclude:: /../code/json/Bitsets.json
    :language: json

.. _xtypes_serialization_utilities_example:

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
