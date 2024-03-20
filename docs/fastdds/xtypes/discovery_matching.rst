.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/aliases.include

.. _xtypes_discovery_matching:

Discovery and Endpoint Matching
===============================

.. _DDS-XTypes V1.3: https://www.omg.org/spec/DDS-XTypes/1.3

*eProsima Fast DDS* incorporates advanced type discovery mechanisms for endpoint discovery and matching,
leveraging the optional |TypeInformation-api| value to ensure compatibility between communicating Endpoints.
Upon the discovery of a participant, *eProsima Fast DDS* examines the |TypeInformation-api| to determine
the availability of essential data type information.

The process of checking the types is as follows:

1. **Evaluation of TypeInformation**: If the discovered participant has transmitted |TypeInformation-api|,
     *eProsima Fast DDS* evaluates whether all required types are already known.
     In cases where some types are missing, an automatic process to discover the necessary types is initiated.
     Utilizing |TypeObject-api| to communicate with other participants and discover the topic types asynchronously.
     Matching of the discovered participant is deferred until all necessary types are known.

2. **Fallback Mechanism**: In scenarios where the discovered participant has not transmitted |TypeInformation-api|,
     *eProsima Fast DDS* resorts to a fallback mechanism.
     It utilizes the type name to verify if the type is already known within the system.
     If the type name is recognized, matching occurs.

.. _xtypes_discovery_typeinformation:

TypeInformation
---------------

|TypeInformation-api| is a fundamental component of the *XTypes 1.3* specification, facilitating the efficient exchange
of essential data type information between DDS Endpoints.
Instead of transmitting the entire |TypeObject|, Endpoints share a compact representation containing the |TypeIdV1-api|
of the respective data type.
The exchanged |TypeIdV1-api| serve as unique identifiers for the associated data types, allowing Endpoints to verify
compatibility and also facilitate subsequent retrieval of the necessary |TypeObject-api| for the data types needed,
enhancing interoperability without burdening non-interested Endpoints with unnecessary data.

This approach significantly reduces network overhead, especially when communicating with Endpoints that may
not require the complete type definition.

The contents of TypeInformation include:

* |TypeIdV1-api|: This is the core component of TypeInformation,
  representing the unique identifier of the data type associated with the Endpoint.
  It enables Endpoints to identify and verify the compatibility of data types during communication.

* Dependency Information: Optionally, TypeInformation may include dependency information,
  indicating other data types on which the current data type depends.
  This information aids Endpoints in understanding the data type hierarchy and its dependencies,
  facilitating comprehensive compatibility checks. The decision regarding which types to include in the
  dependency information of TypeInformation is left to the implementation.

|TypeInformation-api| is described in the annexes of `DDS-XTypes V1.3`_ document,
please refer to this document for details.

.. _xtypes_discovery_typeobject:

TypeObject
----------

The *XTypes* type system uniquely identifies any possible type with a TypeIdentifier.
For simple types such as primitive types, strings, or certain sequences of primitives,
the TypeIdentifier fully describes the type.
However, for more complex types, the TypeIdentifier only serves to identify the type,
and its complete description requires a TypeObject.

|TypeObject-api| provides a thorough description of a data type, similar to how it's represented in IDL.
There are two kinds of TypeObjects: :class:`CompleteTypeObject` and :class:`MinimalTypeObject`.

- :class:`CompleteTypeObject` mirrors the expressiveness of types represented in IDL or XML.
     It can accurately represent any non-plain type from IDL without information loss,
     except for formatting differences.
     This representation is well-suited for programming and tooling purposes.

- :class:`MinimalTypeObject` offers a simplified representation of a data type,
     containing only essential information for remote Endpoints to interpret the data accurately.
     This representation excludes details such as user-defined annotations or member order for types with
     extensibility kind MUTABLE, which do not affect type assignability.
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
