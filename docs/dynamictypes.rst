.. _dynamic-types:

Dynamic Topic Types
===================

.. _DDS-XTypes V1.2: http://www.omg.org/spec/DDS-XTypes/1.2

*eProsima Fast RTPS* provides a dynamic way to define and use topic types and topic data.
Our implementation follows the *OMG Extensible and Dynamic Topic Types for DDS interface*.
For more information, you can read the specification for `DDS-XTypes V1.2`_.

The dynamic topic types offer the possibility to work over RTPS without the restrictions related to the IDLs.
Using them the users can declare the different types that they need and manage the information directly,
avoiding the additional step of updating the IDL file and the generation of *C++* classes.

The management of dynamic types is split into two main groups.
The first one manages the declaration of the types, building and
setting the configuration of every type and the second one is in charge of
the data instances and their information.

Concepts
--------

**Type Descriptor**

Stores the information about one type with its relationships and restrictions.
It's the minimum class needed to generate a Dynamic type and in case of the
complex ones, it stores information about its children or its parent types.

**Member Descriptor**

Several complex types need member descriptors to declare the relationship between types.
This class stores information about that members like their name, their unique ID,
the type that is going to be created and the default value after the creation.
Union types have special fields to identify each member by labels.

**Dynamic Type Builder Factory**

*Singleton* class that is in charge of the creation and the management of every
``DynamicTypes`` and ``DynamicTypeBuilders``.
It declares methods to create each kind of supported types, making easier the
management of the descriptors.
Every object created by the factory must be deleted calling the ``delete_type`` method.

**Dynamic Type Builder**

Intermediate class used to configure and create ``DynamicTypes``.
By design Dynamic types can't be modified, so the previous step to create a new one is to create a builder and apply
the settings that the user needs.
Users can create several types using the same builder, but the changes applied
to the builder don't affect to the types created previously.
Every object created by a builder must be deleted calling the ``delete_type`` method
of the Dynamic Type builder Factory.

**Dynamic Type**

Base class in the declaration of Dynamic types, it stores the information about
its type and every Member that is related to it.
It creates a copy of the descriptor on its creation and cannot be changed to keep the consistency.

**Dynamic Type Member**

A class that creates the relationship between a member descriptor with its parent type.
Dynamic Types have a one Dynamic type member for every child member added to it.

**Dynamic Data Factory**

*Singleton* class that is in charge of the creation and the management of every
``DynamicData``.
It creates them using the given ``DynamicType`` with its settings.
Every data object created by the factory must be deleted calling the ``delete_type`` method.
Allows creating a ``TypeIdentifier`` and a (Minimal and Complete) ``TypeObject`` from a ``TypeDescriptor``.

**Dynamic Data**

A class that manages the data of the Dynamic Types. It stores the information that is
sent and received.
There are two ways to work with DynamicDatas, the first one is the
most secured, activating the macro ``DYNAMIC_TYPES_CHECKING``, it creates a variable for
each primitive kind to help the debug process.
The second one reduces the size of the ``DynamicData`` class using only the minimum
values and making the code harder to debug.

**Dynamic PubSubType**

A class that inherits from ``TopicDataType`` and works as an intermediary between RTPS Domain and the Dynamic Types.
It implements the methods needed to create, serialize, deserialize and delete ``DynamicData`` instances when the
participants need to convert the received information from any transport to the registered dynamic type.


Supported Types
---------------

In order to provide maximum flexibility and capability to the defined dynamic types,
*eProsima Fast RTPS* supports several member types, ranging from simple primitives to nested structures.

Primitive Types
^^^^^^^^^^^^^^^

This section includes every simple kind:

+--------------------------+--------------------------+
| ``BOOLEAN``              | ``INT64``                |
+--------------------------+--------------------------+
| ``BYTE``                 | ``UINT16``               |
+--------------------------+--------------------------+
| ``CHAR8``                | ``UINT32``               |
+--------------------------+--------------------------+
| ``CHAR16``               | ``UINT64``               |
+--------------------------+--------------------------+
| ``INT16``                | ``FLOAT32``              |
+--------------------------+--------------------------+
| ``INT32``                | ``FLOAT64``              |
+--------------------------+--------------------------+
| ``FLOAT128``             |                          |
+--------------------------+--------------------------+

By definition, primitive types are self-described and can be created without configuration parameters.
Therefore, ``DynamicTypeBuilderFactory`` exposes several methods to allow users create
the dynamic type avoiding the ``DynamicTypeBuilder`` step.
The ``DynamicTypeBuilder`` can still be used to create dynamic data of primitive types,
as shown on the example below.
The ``DynamicData`` class has a specific ``get`` and ``set`` methods for each primitive
type of the list.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_PRIMITIVES
   :end-before: //!--

String and WString
^^^^^^^^^^^^^^^^^^

Strings are pretty similar to primitive types except that they need to set the size
of the ``buffer`` that they can manage.
By default this size is set to 255 characters.

``DynamicTypeBuilderFactory`` exposes the methods ``create_string_type`` and ``create_wstring_type``
to allow users create the Dynamic Types avoiding the ``DynamicTypeBuilder`` step.
The ``DynamicTypeBuilder`` can still be used to create String type dynamic data,
as shown on the example below.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRINGS
   :end-before: //!--

Alias
^^^^^

Alias types provide an alternative name to an already existing type.
Once the ``DynamicData`` is created, users can access its information as if
they were working with the base type.

``DynamicTypeBuilderFactory`` exposes the method ``create_alias_type`` to allow users
create the Alias types avoiding the ``DynamicTypeBuilder`` step.
The ``DynamicTypeBuilder`` can still be used to create Alias,
as shown on the example below.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ALIAS
   :end-before: //!--

Enumeration
^^^^^^^^^^^

An enumeration contains a set of supported values and a selected value among those supported.
The supported values must be configured using the ``DynamicTypeBuilder``, using the `add_member`` method
for each supported value.
The input to this method are the index and the name of the value we want to add.

The ``DynamicData`` class has methods ``get_enum_value`` and ``set_enum_value`` to work
with value index or value name name strings.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ENUMERATIONS
   :end-before: //!--

Bitmask
^^^^^^^

Bitmasks are similar to `enumeration` types, but their members work as bit flags that can be individually turned on and
off. Bit operations can be applied when testing or setting a bitmask value.
``DynamicData`` has the special methods ``get_bitmask_value`` and ``set_bitmask_value`` which allow to retrieve or
modify the full value instead of accessing each bit.

Bitmasks can be bound to any number of bits up to 64.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_BITMASKS
   :end-before: //!--

Structure
^^^^^^^^^

Structures are the common complex types, they allow to add any kind of members inside them.
They don't have any value, they are only used to contain other types.

To manage the types inside the structure, users can call the ``get`` and ``set`` methods
according to the kind of the type inside the structure using their ``ids``.
If the structure contains a complex value, it should be used with ``loan_value`` to
access to it and ``return_loaned_value`` to release that pointer.
``DynamicData`` manages the counter of loaned values and users can't loan a value that
has been loaned previously without calling ``return_loaned_value`` before.

The ``Ids`` must be consecutive starting by zero, and the ``DynamicType`` will change that
Id if it doesn't match with the next value.
If two members have the same Id, after adding the second one, the previous
will change its Id to the next value.
To get the Id of a member by name, ``DynamicData`` exposes the method ``get_member_id_by_name``.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRUCTS
   :end-before: //!--

Structures allow inheritance, exactly with the same OOP meaning. To inherit from another structure, we must create the
structure calling the ``create_child_struct_builder`` of the factory. This method is shared with bitsets and will
deduce our type depending on the parent's type.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRUCTS-INHERIT
   :end-before: //!--

Bitset
^^^^^^

Bitset types are similar to `structure` types but their members are only `bitfields`, which are stored optimally.
In the static version of bitsets, each bit uses just one bit in memory (with platform limitations) without alignment
considerations. A bitfield can be anonymous (cannot be addressed) to skip unused bits within a bitset.

Each bitfield in a bitset can be modified through their minimal needed primitive representation.

+--------------------------+--------------------------+
| Number of bits           | Primitive                |
+--------------------------+--------------------------+
| ``1``                    | ``BOOLEAN``              |
+--------------------------+--------------------------+
| ``2-8``                  | ``UINT8``                |
+--------------------------+--------------------------+
| ``9-16``                 | ``UINT16``               |
+--------------------------+--------------------------+
| ``17-32``                | ``UINT32``               |
+--------------------------+--------------------------+
| ``33-64``                | ``UINT64``               |
+--------------------------+--------------------------+

Each bitfield (or member) works like its primitive type with the only difference that the internal storage only
modifies the involved bits instead of the full primitive value.

Bit_bound and position of the bitfield can be set using annotations (useful when converting between static and dynamic
bitsets).

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_BITSETS
   :end-before: //!--

Bitsets allows inheritance, exactly with the same OOP meaning. To inherit from another bitset, we must create the
bitset calling the ``create_child_struct_builder`` of the factory. This method is shared with structures and will
deduce our type depending on the parent's type.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_BITSETS-INHERIT
   :end-before: //!--

Union
^^^^^

Unions are a special kind of structures where only one of the members is active
at the same time.
To control these members, users must set the ``discriminator`` type that is going to be used
to select the current member calling the ``create_union_builder`` method.
The ``discriminator`` itself is a Dynamic Type of any primitive type, string type or union type.

Every member that is going to be added needs at least one ``union_case_index`` to set
how it is going to be selected and, optionally, if it is the default value of the union.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_UNIONS
   :end-before: //!--

Sequence
^^^^^^^^

A complex type that manages its members as a list of items allowing users to
insert, remove or access to a member of the list. To create this type users
need to specify the type that it is going to store and optionally the size
limit of the list.

To ease the memory management of this type, ``DynamicData`` has these methods:

 - ``insert_sequence_data``: Creates a new element at the end of the list and returns
   the ``id`` of the new element.
 - ``remove_sequence_data``: Removes the element of the given index and refreshes the ids
   to keep the consistency of the list.
 - ``clear_data``: Removes all the elements of the list.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_SEQUENCES
   :end-before: //!--

Array
^^^^^

Arrays are pretty similar to sequences with two main differences:
they can have multiple dimensions and they don't need that the elements
are stored consecutively.

An array needs to know the number of dimensions it is managing.
For that, users must provide a vector with as many elements as dimensions in the array.
Each element in the vector represents the size of the given dimension.
If the value of an element is set to zero, the default value applies ( :class:`100` ).

Id values on the ``set`` and ``get`` methods of ``DynamicData`` correspond to the array index.
To ease the management of array elements, every ``set`` method in ``DynamicData`` class creates
the item if the given index is empty.

To ease the memory management of this type, ``DynamicData`` has these methods:
 - ``insert_array_data``: Creates a new element at the end of the array and returns
   the ``id`` of the new element.
 - ``remove_array_data``: Clears the element of the given index.
 - ``clear_data``: Removes all the elements of the array.
 - ``get_array_index``: Returns the position id giving a vector of indexes on every dimension
   that the arrays support, which is useful in multidimensional arrays.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ARRAYS
   :end-before: //!--

Map
^^^

Maps contain a list of pairs 'key-value' types, allowing users to insert, remove or
modify the element types of the map.
The main difference with sequences is that the map works with pairs of elements and
creates copies of the key element to block the access to these elements.

To create a map, users must set the types of the key and the value elements, and,
optionally, the size limit of the map.

To ease the memory management of this type, ``DynamicData`` has these methods:
 - ``insert_map_data``: Inserts a new key value pair and returns the ids of the newly
   created key and value elements.
 - ``remove_map_data``: Uses the given id to find the key element and removes the key
   and the value elements from the map.
 - ``clear_data``: Removes all the elements from the map.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_MAPS
   :end-before: //!--

Complex examples
----------------

If the application's data model is complex, it is possible to combine the basic types
described above to create complex types, including nested composed types (structures
within structures within unions).
Types can also be extended using inheritance, improving the flexibility of the definition
of the data types to fit the model.

Nested structures
^^^^^^^^^^^^^^^^^

Structures can contain other structures as members.
The access to these compound members is restricted and managed by the ``DynamicData`` instance.
Users must request access calling ``loan_value`` before using them, and release
them with ``return_loaned_value`` once they finished.
The loan operation will fail if the member is already loaned and has not been released yet.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_STRUCTS
   :end-before: //!--

Structures inheritance
^^^^^^^^^^^^^^^^^^^^^^

To inherit a structure from another one, use the ``create_child_struct_type`` method from
``DynamicTypeBuilderFactory``.
The resultant type contains all members from the base class and the new ones added to the child.

Structures support several levels of inheritance, so the base class can be another derived type itself.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_INHERITANCE_STRUCTS
   :end-before: //!--

Alias of an alias
^^^^^^^^^^^^^^^^^

Alias types support recursion, simply use an alias name as base type for ``create_alias_type``.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_ALIAS
   :end-before: //!--

Unions with complex types
^^^^^^^^^^^^^^^^^^^^^^^^^

Unions support complex type fields.
The access to these complex type fields is restricted and managed by the ``DynamicData`` instance.
Users must request access calling ``loan_value`` before using them, and release
them with ``return_loaned_value`` once they finished.
The loan operation will fail if the fields is already loaned and has not been released yet.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_UNIONS
   :end-before: //!--

Annotations
^^^^^^^^^^^

DynamicTypeBuilder allows applying an annotation to both current type and inner members with the methods:

- ``apply_annotation``

- ``apply_annotation_to_member``

Both methods take the name,  the key and the value of the annotation.
``apply_annotation_to_member`` additionally receives the ``MemberId`` of the inner member.

For example, if we define an annotation like:

.. code-block:: idl

    @annotation MyAnnotation
    {
        long value;
        string name;
    };

And then we apply it through IDL to a struct:

.. code-block:: idl

    @MyAnnotation(5, "length")
    struct MyStruct
    {
    ...

The equivalent code using DynamicTypes will be:

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ANNOTATION
   :end-before: //!--

Builtin annotations
~~~~~~~~~~~~~~~~~~~

The following annotations modifies the behavior of DynamicTypes:

- | ``@position``: When applied to Bitmask_, sets the position of the flag, as expected in the IDL annotation.
  | If applied to Bitset_, sets the base position of the bitfield, useful to identify unassigned bits.

- | ``@bit_bound``: Applies to Bitset_. Sets the size in bits of the bitfield.

- | ``@key``: Alias for ``@Key``. See :ref:`topics-and-keys` section for more details.

- | ``@default``: Sets a default value for the member.

- | ``@non_serialized``: Excludes a member from being serialized.

Serialization
-------------

Dynamic Types have their own :class:`pubsub` type like any class generated with an IDL, and
their management is pretty similar to them.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_SERIALIZATION
   :end-before: //!--

A member can be marked to be ignored by serialization with the annotation ``@non_serialized``.

Important Notes
---------------

Memory management is critical for Dynamic Types,  because
every dynamic type and dynamic data is managed with pointers.
Every object stored inside of a dynamic object is managed by its owner, and users
must delete every object they create using the factories

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_NOTES_1
   :end-before: //!--

To ease this management, the library defines smart pointers (``DynamicTypeBuilder_ptr``,
``DynamicType`` and  ``DynamicData_ptr``) that will delete the objects automatically when they are not
needed anymore.
``DynamicType`` will always be returned as ``DynamicType_ptr`` because there is no internal management of its memory.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_NOTES_2
   :end-before: //!--

The only case where these smart pointers cannot be used is with methods ``loan_value`` and ``return_loaned_value``.
Raw pointers should be used with these methods, because the returned value should not be deleted, and using
a smart pointer with them will cause a crash.


Dynamic Types Discovery and Endpoint Matching
---------------------------------------------

When using Dynamic Types support, *Fast RTPS* checks the optional ``TypeObjectV1`` and ``TypeIdV1`` values
during endpoint matching.
Currently, the matching only verifies that both endpoints are using the same topic data type,
but will not negotiate about it.

The matching checks ``CompleteTypeObject`` first.
If one or both endpoints do not define the ``CompleteTypeObject``, it tries with ``MinimalTypeObject``.
If one or both endpoints do not define ``MinimalTypeObject`` either, it compares the ``TypeIdentifier``.
If none is defined, then just the type name is checked.

If one of the endpoints  transmits a ``CompleteTypeObject``, :ref:`discovery-time-data-typing` can be done.

TypeObject (TypeObjectV1)
^^^^^^^^^^^^^^^^^^^^^^^^^

There are two kinds of ``TypeObject``: ``MinimalTypeObject`` and ``CompleteTypeObject``.

 - ``MinimalTypeObject`` is used to check compatibility between types.
 - ``CompleteTypeObject`` fully describes the type.

``TypeObject`` is an IDL union with both representation, *Minimal* and *Complete*.
Both are described in the annexes of `DDS-XTypes V1.2`_ document,
please refer to this document for details.

TypeIdentifier (TypeIdV1)
^^^^^^^^^^^^^^^^^^^^^^^^^

``TypeIdentifier`` represents a full description of basic types and has an :class:`EquivalenceKind` for complex ones.

``TypeIdentifier`` is described in the annexes of `DDS-XTypes V1.2`_ document,
please refer to this document for details.

TypeObjectFactory
^^^^^^^^^^^^^^^^^

*Singleton* class that manages the creation and access for all registered ``TypeObjects`` and ``TypeIdentifiers``.
It can generate a full ``DynamicType`` from a basic ``TypeIdentifier`` (i.e., a ``TypeIdentifier`` whose discriminator
isn't:class:`EK_MINIMAL` or :class:`EK_COMPLETE`) can generate a full ``DynamicType``.

Fastrtpsgen
^^^^^^^^^^^

*FastRTPSGen* has been upgraded to generate :class:`XXXTypeObject.h` and :class:`XXXTypeObject.cxx` files,
taking :class:`XXX` as our IDL type.
These files provide a small Type Factory for the type :class:`XXX`.
Generally, these files are not used directly, as now the type :class:`XXX` will register itself through its factory to
``TypeObjectFactory`` in its constructor, making it very easy to use static types with dynamic types.

.. _discovery-time-data-typing:

Discovery-Time Data Typing
^^^^^^^^^^^^^^^^^^^^^^^^^^

When using fastdds API, when a participant discovers a remote endpoint that sends a complete ``TypeObject`` or a simple
``TypeIdentifier`` describing a type that the participant doesn't know, the participant listener's
method ``on_type_discovery`` is called with the received ``TypeObject`` or ``TypeIdentifier provided``,
and, when possible, a ``DynamicType_ptr`` ready to be used.

Discovery-Time Data Typing allows the discovering of simple DynamicTypes. A TypeObject that depends on other
TypeObjects, cannot be built locally using Discovery-Time Data Typing and should use :ref:`TypeLookup-Service` instead.

To ease the sharing of the TypeObject and TypeIdentifier used by Discovery-Time Data Typing, there exists
an attribute in ``TopicAttributes`` named ``auto_fill_type_object``. If set to true, on discovery time,
the local participant will try to fill ``type`` and ``type_id`` fields in the correspond ``ReaderProxyData``
or ``WriterProxyData`` to be sent to the remote endpoint.


.. _typelookup-service:

TypeLookup Service
^^^^^^^^^^^^^^^^^^

When using fastdds API, when a participant discovers an endpoint that sends a TypeInformation
describing a type that the participant doesn't know, the participant listener's
method ``on_type_information_received`` is called with the received TypeInformation.
The user can then try to retrieve the full TypeObject hierarchy to build the remote type locally, using the
TypeLookup Service.

To enable this builtin TypeLookup Service, the user must enable it in the Participant's RTPS builtin attributes:

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //TYPELOOKUP_SERVICE_ENABLING
   :end-before: //!--

A participant can be enabled to act as a TypeLookup server, client, or both.

The process of retrieving the remote type from its TypeInformation, and then registering it, can be simplified
using the0  ``register_remote_type`` method on the :ref:`dds_layer_domainParticiant`.
This method takes the name of the type, the TypeInformation, and a callback function.
Internally it uses the TypeLookup Service to retrieve the full TypeObject, and, if successful, it will
call the callback.

This callback has the following signature:

.. code-block:: c

    void(std::string& type_name, const DynamicType_ptr type)

- | type_name: Is the name given to the type when calling ``register_remote_type`` function,
  | to allow the same callback to be used across different calls.

- | type: If the ``register_remote_type`` was able to build and register a DynamicType,
  | this parameter contains a pointer to the type.
  | Otherwise it contains ``nullptr``.
  | In the latter case, the user can still try to build the type manually using the factories, but it is very
  | likely that the build process will fail.

``TopicAttributes`` contains an  attribute named ``auto_fill_type_information``.
If set to true, the local participant will try to fill the ``type_information`` field in the correspond
``ReaderProxyData`` or ``WriterProxyData`` to be sent to the remote endpoint during discovery.


XML Dynamic Types
-----------------

:ref:`XMLDynamicTypes` allows *eProsima Fast RTPS* to create Dynamic Types directly defining them through XML.
This allows any application to change ``TopicDataTypes`` without the need to change its source code.


Dynamic HelloWorld Examples
---------------------------

DynamicHelloWorldExample
^^^^^^^^^^^^^^^^^^^^^^^^

This example is located on the folder ``examples/C++/DynamicHelloWorldExample``.
It shows the use of DynamicType generation to provide the TopicDataType.
This example is compatible with the classic HelloWorldExample.

As a quick reference, the following piece of code shows how the HelloWorld type is created using DynamicTypes:

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_HELLO_WORLD_API
   :end-before: //!--


DDSDynamicHelloWorldExample
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This example uses the DDS API, and is located in the ``examples/C++/DDS/DynamicHelloWorldExample`` folder.
It shows a publisher that loads a type from an XML file, and shares it during discovery.
The subscriber discovers the type using :ref:`discovery-time-data-typing`, and registers the
discovered type on the ``on_type_discovery`` listener method.

TypeLookupService
^^^^^^^^^^^^^^^^^

This example uses the DDS API, and is located in the ``examples/C++/DDS/TypeLookupService`` folder.
It's very similar to DDSDynamicHelloWorldExample, but the shared type is complex enough to require the
TypeLookup Service due to the dependency of inner struct types.
Specifically, it uses the ``register_remote_type`` approach with a callback.
