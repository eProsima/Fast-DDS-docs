.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/aliases.include

.. _dynamictypes_discovery_matching:

Dynamic Types Discovery and Endpoint Matching
=============================================

.. _DDS-XTypes V1.3: http://www.omg.org/spec/DDS-XTypes/1.3

When using |DynamicTypes| support, *Fast DDS* checks the optional |TypeInformation-api|
value during endpoint matching.

The process of checking the types is as follows:

* If the discovered participant has sent |TypeInformation-api|, Fast DDS will check if all required types are known.
  If not, it will initiate an automatic process of discovering the necessary types, using |TypeObject-api|
  to communicate the information about the types with the other participants.
  This process occurs asynchronously, and the matching of the discovered participant will not occur
  until all necessary types are known, and they are fully compatible.
* If the discovered participant has not sent |TypeInformation-api|, Fast DDS will utilize a fallback mechanism,
  using the type name to check if the type is known. Matching will occur if the type is known.

.. _dynamictypes_discovery_typeinformation:

TypeInformation
---------------

|TypeInformation-api| is an extension of *XTypes 1.3* that allow Endpoints to share information about data types 
without sending the TypeObject.
Endpoints instead share a TypeInformation containing the TypeIdentifier of the data type.
Then each Endpoint can request the TypeObject for the data types it is interested in.
This avoids sending the complete data type to Endpoints that may not be interested.

|TypeInformation-api| is described in the annexes of `DDS-XTypes V1.3`_ document,
please refer to this document for details.

.. _dynamictypes_discovery_typeobject:

TypeObject
----------

|TypeObject-api| provides a thorough description of a data type, similar to how it's represented in IDL.
There are two kinds of TypeObjects: :class:`CompleteTypeObject` and :class:`MinimalTypeObject`.

 - :class:`CompleteTypeObject` is representation of a type with the same level of detail and expressiveness as in IDL.
      It serves as an alternative representation of types, suitable for programming and tooling purposes.

 - :class:`MinimalTypeObject` is a simplified version of the data type.
      It includes only the necessary information for remote Endpoints to interpret the data accurately.
      This simplified representation excludes details such as user-defined annotations or member order for types with
      extensibility kind MUTABLE, which are irrelevant for type assignability.
      By doing so, it reduces the amount of information applications need to transmit over the network.

TypeObject is an IDL union with both *Minimal* and *Complete* representation.
Both are described in the annexes of `DDS-XTypes V1.3`_ document, please refer to this document for details.

Fast DDS-Gen
------------
*Fast DDS-Gen* default behaviour creates the necessary code to work with *Dynamic Types*.
The generated code will include the `XXXTypeObjecSupport.h` and `XXXTypeObjecSupport.cxx` files,
taking :class:`XXX` as our IDL type.
These files provide a Type Factory for the type :class:`XXX`.
Generally, these files are not used directly, as the type :class:`XXX` will register itself.

*Fast DDS-Gen* usage information can be found on the :ref:`Fast DDS-Gen <fastddsgen_intro>` section.
