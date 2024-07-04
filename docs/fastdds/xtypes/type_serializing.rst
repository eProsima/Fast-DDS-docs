.. include:: ../../03-exports/aliases-api.include

.. _xtypes_type_serializing_json:

Dynamic Type Serializing
========================

Fast-DDS provides methods to serialize `DynamicType` objects.

Dynamic Data to JSON
--------------------

The method `json_serialize` converts a `DynamicData` object into an JSON string.
The following `DynamicData` object would be serialized as follows:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
    :language: omg-idl
    :start-after: //!--IDL_TYPE_INTROSPECTION_EXAMPLE
    :end-before: //!--

.. literalinclude:: /../code/DynamicDataJSONExamples.json
    :language: json

Example: Remote discovered types introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following code demonstrates how to implement remote type introspection using FastDDS in C++.
This feature allows a subscriber to introspect the remotly discovered type and serialize it into a more manageable
and understandable format.
Once a type is discovered, it can be registered and every time the subscriber receives new data related to this type,
the corresponding DynamicData can be obtained from the DynamicDataFactory and serialized into a JSON string format.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //!--REMOTE_TYPE_INTROSPECTION
    :end-before: //!--
