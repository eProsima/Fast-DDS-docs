.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/aliases.include

.. _dynamictypes_discovery:

Dynamic Types Discovery and Endpoint Matching
=============================================

.. _DDS-XTypes V1.3: http://www.omg.org/spec/DDS-XTypes/1.3

When using |DynamicTypes| support, *Fast DDS* checks the optional |TypeObject|
and |TypeIdentifier| values during endpoint matching.
Currently, the matching only verifies that both endpoints are using the same topic data type,
but will not negotiate about it.

The process of checking the types is as follows:

* It checks :class:`CompleteTypeObject` on TypeObject first.
* If one or both endpoints do not define the :class:`CompleteTypeObject`, it tries with :class:`MinimalTypeObject`.
* If one or both endpoints do not define :class:`MinimalTypeObject` either,
  it compares the TypeIdentifier.
* If none is defined, then just the type name is checked.

If one of the endpoints transmits a :class:`CompleteTypeObject`, :ref:`discovery-time-data-typing` can be performed.

.. _dynamictypes_discovery_typeobject:

TypeObject
----------

|TypeObjectV1-api| fully describes a data type, the same way as the IDL representation does.
There are two kinds of TypeObjects: :class:`CompleteTypeObject` and :class:`MinimalTypeObject` .

 - :class:`CompleteTypeObject` fully describes the type, the same way as the IDL representation does.
 - :class:`MinimalTypeObject` is a compact representation of the data type, that contains only the information relevant
   for the remote Endpoint to be able to interpret the data.

TypeObject is an IDL union with both *Minimal* and *Complete* representation.
Both are described in the annexes of `DDS-XTypes V1.3`_ document,
please refer to this document for details.


.. _dynamictypes_discovery_typeinformation:

TypeInformation
---------------

|TypeInformation-api| is an extension of *XTypes 1.3* that allow Endpoints to
share information about data types without sending the TypeObject.
Endpoints instead share a TypeInformation containing the
TypeIdentifier of the data type.
Then each Endpoint can request the complete TypeObject for the data
types it is interested in.
This avoids sending the complete data type to Endpoints that may not be interested.

|TypeInformation-api| is described in the annexes of `DDS-XTypes V1.3`_ document,
please refer to this document for details.


.. _dynamictypes_discovery_typeidentifier:

TypeIdentifier
--------------

|TypeIdV1-api| provides a unique way to identify each type.
For basic types, the information contained in the TypeIdentifier
completely describes the type, while for complex ones, it serves as a search key to
retrieve the complete TypeObject.

|TypeIdV1-api| is described in the annexes of `DDS-XTypes V1.3`_ document,
please refer to this document for details.

Fast DDS-Gen
------------

*Fast DDS-Gen* supports the generation of `XXXTypeObjecSupport.h` and `XXXTypeObjecSupport.cxx` files,
taking :class:`XXX` as our IDL type.
These files provide a small Type Factory for the type :class:`XXX`.
Generally, these files are not used directly, as now the type :class:`XXX` will register itself,
making it very easy to use static types with dynamic types.

.. _discovery-time-data-typing:

Discovery-Time Data Typing
--------------------------

Using the Fast DDS API, when a participant discovers a remote endpoint that sends a complete
TypeObject or a simple TypeIdentifier describing a type
that the participant does not know, the participant listener's function
is called with the received TypeObject or TypeIdentifier,
and, when possible, a pointer to a :ref:`dynamictypes_overview_dynamictype` ready to be used.

Discovery-Time Data Typing allows the discovering of simple DynamicTypes.
A TypeObject that depends on other TypeObjects, cannot be built locally using
Discovery-Time Data Typing and should use :ref:`TypeLookup-Service` instead.

.. _typelookup-service:

TypeLookup Service
------------------

Using the Fast DDS API, when a participant discovers an endpoint that sends a type information
describing a type that the participant doesn't know, the participant listener's function
is called with the received TypeInformation.
The user can then try to retrieve the full TypeObject hierarchy to build the remote type locally, using the
TypeLookup Service.

A participant can be enabled to act as a TypeLookup server, client, or both.

This function takes the name of the type, the type information, and a callback function.
Internally it uses the TypeLookup Service to retrieve the full TypeObject,
and, if successful, it will call the callback.

This callback has the following signature:

.. code-block:: c

    void(std::string& type_name, const DynamicType_ptr type)

* **type_name**: Is the name given to the type 
  to allow the same callback to be used across different calls.

* **type**: If unable to build and register a :ref:`dynamictypes_overview_dynamictype`, this parameter contains
  a pointer to the type.
  Otherwise it contains ``nullptr``.
  In the latter case, the user can still try to build the type manually using the factories, but it is very
  likely that the build process will fail.

:ref:`TopicDataType<dds_layer_definition_data_types>` contains a data member named
|TopicDataType::auto_fill_type_information-api|.
If set to true, the local participant will send the type information to the remote endpoint during discovery.


