.. _dynamic-types:

Dynamic Topic Types
===================

.. _link: http://www.omg.org/spec/DDS-XTypes/1.2

*eProsima Fast RTPS* provides a dynamic way to define and use topic types and topic data.
Our implementation follows the *OMG Extensible and Dynamic Topic Types for DDS interface*.
For more information, you can read the document (DDS-XTypes V1.2) in this link_.

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

Primitive types don't need a specific configuration to create the type. Because of that
``DynamicTypeBuilderFactory`` has got exposed several methods to allow users to create
the Dynamic Types avoiding the ``DynamicTypeBuilder`` step. The example below shows the two
ways to create dynamic data of a primitive type.
The ``DynamicData`` class has a specific ``get`` and ``set`` Methods for each primitive
type of the list.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_PRIMITIVES
   :end-before: //!--

String and WString
^^^^^^^^^^^^^^^^^^

Strings are pretty similar to primitive types with one exception, they need to set the size
of the ``buffer`` that they can manage.
To do that, ``DynamicTypeBuilderFactory`` exposes the methods ``create_string_type`` and ``create_wstring_type``.
By default, its size is set to 255 characters.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRINGS
   :end-before: //!--

Alias
^^^^^

Alias types have been implemented to rename an existing type, keeping the rest of properties
of the given type.
``DynamicTypeBuilderFactory`` exposes the method ``create_alias_type`` to create alias types
taking the base type and the new name that the alias is going to set.
After the creation of the ``DynamicData``, users can access its information like
they were working with the base type.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ALIAS
   :end-before: //!--

Enumeration
^^^^^^^^^^^

The enum type is managed as complex in Dynamic Types because it allows adding members
to set the different values that the enum is going to manage.
Internally, it works with a ``UINT32`` to store what value is selected.

To use enumerations users must create a Dynamic Type builder calling to ``create_enum_type``
and after that, they can call to ``add_member`` given the index and the name of the
different values that the enum is going to support.

The `DynamicData` class has got methods ``get_enum_value`` and ``set_enum_value`` to work
with ``UINT32`` or with strings using the names of the members added to the builder.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ENUMERATIONS
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
will change its id to the next value.
To get the id of a member by name, ``DynamicData`` exposes the method ``get_member_id_by_name``.

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

Union
^^^^^

Unions are a special kind of structures where only one of the members is active
at the same time.
To control these members, users must set the :class:`discriminator` type that is going to be used
to select the current member calling the ``create_union_type`` method.
After the creation of the Dynamic Type, every member that is going to be added
needs at least one ``union_case_index`` to set how it is going to be selected and
optionally if it is the default value of the union.

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
- ``remove_sequence_data``: Removes the element of the given index and refresh the ids
to keep the consistency of the list.
- ``clear_data``: Removes all the elements of the list.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_SEQUENCES
   :end-before: //!--

Array
^^^^^

Arrays are pretty similar to sequences with two main differences. The first one is
that they can have multiple dimensions and the other one is that they don't need
that the elements are stored consecutively.
The method to create arrays needs a vector of sizes to set how many dimensions are
going to be managed, if users don't want to set a limit can set the value as zero
on each dimension and it applies the default value ( :class:`100` ).
To ease the management of arrays every ``set`` method in ``DynamicData`` class creates
the item if there isn't any in the given ``Id``.
Arrays also have methods to handle the creation and deletion of elements like
sequences, they are ``insert_array_data``, ``remove_array_data`` and ``clear_data``.
Additionally, there is a special method ``get_array_index`` that returns the position id
giving a vector of indexes on every dimension that the arrays support, that is
useful in multidimensional arrays.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ARRAYS
   :end-before: //!--

Map
^^^

Maps contain a list of pairs 'key-value' types, allowing users to insert, remove or
modify the element types of the map. The main difference with sequences is that the map
works with pairs of elements and creates copies of the key element to block the access
to these elements.

To create a map, users must set the types of the key and the value elements and
optionally the size limit of the map. To add a new element to the map, ``DynamicData``
has the method ``insert_map_data`` that returns the ids of the key and the value
elements inside the map.
To remove an element of the map there is the method ``remove_map_data`` that uses the
given id to find the key element and removes the key and the value elements from the map.
The method ``clear_data`` removes all the elements from the map.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_MAPS
   :end-before: //!--

Complex examples
----------------

Nested structures
^^^^^^^^^^^^^^^^^

Structures allow to add other structures inside them, but users must take care that
to access to these members they need to call ``loan_value`` to get a pointer to the
data and release it calling ``return_loaned_value``.
``DynamicDatas`` manages the counter of loaned values and users can't loan a value that
has been loaned previously without calling ``return_loaned_value`` before.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_STRUCTS
   :end-before: //!--

Structures inheritance
^^^^^^^^^^^^^^^^^^^^^^

Structures can inherit from other structures. To do that ``DynamicTypeBuilderFactory``
has the method ``create_child_struct_type`` that relates the given struct type with
the new one. The resultant type contains the members of the base class and the ones
that users have added to it.

Structures support several levels of inheritance, creating recursively the members
of all the types in the hierarchy of the struct.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_INHERITANCE_STRUCTS
   :end-before: //!--

Alias of an alias
^^^^^^^^^^^^^^^^^

Alias types support recursion, so if users need to create an alias of another alias,
it can be done calling ``create_alias_type`` method giving the alias as a base type.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_ALIAS
   :end-before: //!--

Unions with complex types
^^^^^^^^^^^^^^^^^^^^^^^^^

Unions support complex types, the available interface to access to them is calling
``loan_value`` to get a pointer to the data and set this field as the active one and
release it calling ``return_loaned_value``.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_NESTED_UNIONS
   :end-before: //!--

Annotations
^^^^^^^^^^^

DynamicTypeBuilder allows applying an annotation to both current type and inner members with the methods:

- ``apply_annotation``

- ``apply_annotation_to_member``

``apply_annotation_to_member`` receives the ``MemberId`` to apply plus the same parameters than ``apply_annotation``.
The common parameters are the name of the annotation, the key and the value.

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

.. code-block:: cpp

    // Create the annotation


    // Apply the annotation
    DynamicTypeBuilder_ptr builder = DynamicTypeBuilderFactory::get_instance()->create_struct_builder();
    ...
    builder->apply_annotation("MyAnnotation", "value", "5");
    builder->apply_annotation("MyAnnotation", "name", "length");

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

The most important part of Dynamic Types is memory management because
every dynamic type and dynamic data are managed with pointers. Every object stored
inside of other dynamic object is managed by its owner, so users only must take care
of the objects that they have created calling to the factories.
These two factories in charge to manage these objects, and they must create and delete every object.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_NOTES_1
   :end-before: //!--

To ease this management, the library incorporates a special kind of shared pointers to call
to the factories to delete the object directly ( ``DynamicTypeBuilder_ptr`` and  ``DynamicData_ptr``).
The only restriction on using this kind of pointers are
the methods ``loan_value`` and ``return_loaned_value``, because they return a pointer
to an object that is already managed by the library and using a ``DynamicData_ptr``
with them will cause a crash.
``DynamicType`` will always be returned as ``DynamicType_ptr`` because there is no internal management of its memory.

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_NOTES_2
   :end-before: //!--

Dynamic Types Discovery and Endpoint Matching
---------------------------------------------

When using Dynamic Types support, *Fast RTPS* make use of an optional ``TypeObjectV1`` and ``TypeIdV1``.
At its current state, the matching will only verify that both endpoints are using the same topic type,
but will not negotiate about it.

This verification is done by ``TypeIdentifier``, then ``MinimalTypeObject``, and finally
``CompleteTypeObject``.

If one endpoints uses a ``CompleteTypeObject`` instead, it makes possible :ref:`discovery-time-data-typing`.

TypeObject (TypeObjectV1)
^^^^^^^^^^^^^^^^^^^^^^^^^

There are two kinds of ``TypeObject``: ``MinimalTypeObject`` and ``CompleteTypeObject``.

 - ``MinimalTypeObject`` is used to check compatibility between types.
 - ``CompleteTypeObject`` fully describes the type.

Both are defined in the annexes of DDS-XTypes V1.2 document so its details will not be covered in this document.

 - ``TypeObject`` is an IDL union with both representation, *Minimal* and *Complete*.

TypeIdentifier (TypeIdV1)
^^^^^^^^^^^^^^^^^^^^^^^^^

``TypeIdentifier`` is described too in the annexes of *DDS-XTypes V1.2 document*.
It represents a full description of basic types and has an :class:`EquivalenceKind` for complex ones.
An :class:`EquivalenceKind` is a hash code of 14 octets, as described by the *DDS-XTypes V1.2 document*.

TypeObjectFactory
^^^^^^^^^^^^^^^^^

*Singleton* class that manages the creation and access for all registered ``TypeObjects`` and ``TypeIdentifiers``.
From a basic ``TypeIdentifier`` (in other words, a ``TypeIdentifier`` whose discriminator isn't
:class:`EK_MINIMAL` or :class:`EK_COMPLETE`) can generate a full ``DynamicType``.

Fastrtpsgen
^^^^^^^^^^^

*FastRTPSGen* has been upgraded to generate :class:`XXXTypeObject.h` and :class:`XXXTypeObject.cxx` files,
taking :class:`XXX` as our IDL type. These files provide a small Type Factory for the type :class:`XXX`.
Generally, these files are not used directly, as now the type :class:`XXX` will register itself through its factory to
``TypeObjectFactory`` in its constructor, making very easy the use of static types with dynamic types.

.. _discovery-time-data-typing:

Discovery-Time Data Typing
^^^^^^^^^^^^^^^^^^^^^^^^^^

When using fastdds API, if a participant discovers an endpoint which sends a complete TypeObject or a simple
TypeIdentifier describing a type that the participant doesn't know, it will be called by its listener's
method ``on_type_discovery`` with the TypeInformation provided, and a ``DynamicType_ptr`` ready to be used.


XML Dynamic Types
-----------------

:ref:`XMLDynamicTypes` allows *eProsima Fast RTPS* to create Dynamic Types directly defining them through XML.
This allows any application to change ``TopicDataTypes`` without the need to change its source code.


Dynamic HelloWorld Examples
---------------------------

DynamicHelloWorldExample
^^^^^^^^^^^^^^^^^^^^^^^^

Using some of the functionality described in this document, there exists an example at
the ``examples/C++/DynamicHelloWorldExample`` folder named
:class:`DynamicHelloWorldExample` that uses DynamicType generation to provide the TopicDataType.

This example is compatible with classic HelloWorldExample.

As a quick reference, it is shown how the HelloWorld type is created using DynamicTypes:

.. literalinclude:: ../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_HELLO_WORLD_API
   :end-before: //!--


DDSDynamicHelloWorldExample
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Another example located in the ``examples/C++/DDS/DynamicHelloWorldExample`` folder shows a publisher that shares
a type loaded from an XML file, and a subscriber that discovers the type using discovery-time-data-typing_, showing
the received data after introspecting it.
