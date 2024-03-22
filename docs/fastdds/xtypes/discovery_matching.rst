.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/aliases.include

.. _xtypes_discovery_matching:

Discovery and Endpoint Matching
===============================

.. _DDS-XTypes V1.3: https://www.omg.org/spec/DDS-XTypes/1.3

*eProsima Fast DDS* incorporates an advanced type discovery service used on endpoint discovery and matching.
Discovery process leverages the optional |TypeInformation-api| value to ensure compatibility between communicating endpoints.
Upon the discovery of a remote endpoint, *eProsima Fast DDS* examines the |TypeInformation-api| to determine
the availability of essential data type information.

The process of checking the types is as follows:

1. **Evaluation of TypeInformation**: If the discovered enpoint has transmitted |TypeInformation-api|,
     *eProsima Fast DDS* evaluates whether all required types are already known.
     In cases where some types are missing, an automatic process to discover the necessary types is initiated.
     |TypeObject-api| is used to communicate with other participants and to discover the topic types asynchronously.
     Matching of the discovered enpoint is deferred until all necessary types are known.

2. **Fallback Mechanism**: In scenarios where the discovered endpoint has not transmitted |TypeInformation-api|,
     *eProsima Fast DDS* resorts to a fallback mechanism.
     This mechanism relys on the type name to verify if the type is already known within the system.
     If the type name is recognized, matching occurs.

.. _xtypes_discovery_typeinformation:

TypeInformation
---------------

|TypeInformation-api| is a fundamental component of the *XTypes 1.3* specification, facilitating the efficient exchange
of essential data type information between DDS endpoints.
Instead of transmitting the entire |TypeObject|, endpoints share a compact representation containing the |TypeIdV1-api|
of the respective data type.
The exchanged |TypeIdV1-api| serves as unique identifiers for the associated data types, allowing endpoints to verify
compatibility and also to facilitate subsequent retrieval of the necessary |TypeObject-api| for the data types needed,
This mechanism enhances interoperability without burdening non-interested endpoints with unnecessary data.

This approach significantly reduces network overhead, especially when communicating with endpoints that may
not require the complete type definition.

The contents of TypeInformation include:

* |TypeIdV1-api|: This is the core component of TypeInformation.
  It represents the unique identifier of the data type associated with the endpoint and
  enables endpoints to identify and verify the compatibility of data types during communication.

* Dependency Information: Optionally, TypeInformation may include dependency information, which
  indicates other data types on which the current data type depends.
  This information helps endpoints understand the data type hierarchy and its dependencies,
  which facilitates comprehensive compatibility checks.
  The decision as to which types to include in the dependency information is left to the implementation.

|TypeInformation-api| is described in the annexes of `DDS-XTypes V1.3`_ document,
please refer to this document for details.

.. _xtypes_discovery_typeobject:

TypeObject
----------

The *XTypes* type system uniquely identifies each possible type with a TypeIdentifier.
For simple types, such as primitive types, strings, or certain sequences of primitives,
the TypeIdentifier fully describes the type.
For more complex types, however, the TypeIdentifier only serves to identify the type,
and its full description requires a TypeObject.

|TypeObject-api| provides a thorough description of a data type, similar to how it's represented in IDL.
There are two kinds of TypeObjects: :class:`CompleteTypeObject` and :class:`MinimalTypeObject`.

- :class:`CompleteTypeObject` mirrors the expressiveness of types represented in IDL or XML.
     It can accurately represent any non-plain type from IDL without information loss,
     except for formatting differences.
     This representation is well-suited for programming and tooling purposes.

- :class:`MinimalTypeObject` provides a simplified representation of a data type,
     that contains only the information necessary for remote endpoints to interpret the data correctly.
     This representation excludes details such as user-defined annotations or member order for types with
     extensibility kind MUTABLE, which do not affect the type assignability.
     By minimizing redundant information, MinimalTypeObject reduces network overhead during type compatibility
     checks between DataWriters and DataReaders.

Both :class:`CompleteTypeObject` and :class:`MinimalTypeObject` are defined in the annexes of the `DDS-XTypes V1.3`_
document, providing detailed specifications and IDL representations.

Fast DDS-Gen
------------

*Fast DDS-Gen* default behaviour generates code compatible with :ref:`XTypes1.3 <dynamic-types>`.
When processing an IDL file, it will automatically generate all the code files needed for the data types.

For instance, assuming the IDL file is named *"Mytype"*, the generated files include:

- *MyType.hpp*: This file contains the definition of the data type.
- *MyTypePubSubType.cxx/.h*: Serialization and deserialization source code for the data type.
- *MyTypeTypeObjectSupport.cxx/.hpp*: Generation and registration of |TypeObject-api|.

Detailed usage instructions and additional information about *Fast DDS-Gen* can be found in the
:ref:`Fast DDS-Gen <fastddsgen_intro>` section.
