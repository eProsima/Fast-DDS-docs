.. _dynamictypes_supportedtypes:

Supported Types
===============

In order to provide maximum flexibility and capability to the defined dynamic types,
eProsima Fast DDS supports several member types, ranging from simple primitives to nested structures.

This section describes the basic (not nested) supported types. For more complex structures and
examples, please, refer to :ref:`dynamictypes_complextypes`.

.. _dynamictypes_supportedtypes_primitive:

Primitive Types
---------------

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
Therefore, ``DynamicTypeBuilderFactory`` exposes several functions to allow users create
the dynamic type avoiding the ``DynamicTypeBuilder`` step.
The ``DynamicTypeBuilder`` can still be used to create dynamic data of primitive types,
as shown on the example below.
The ``DynamicData`` class has a specific ``get`` and ``set`` functions for each primitive
type of the list.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_PRIMITIVES
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_string:

String and WString
------------------

Strings are pretty similar to primitive types, the main difference being
that they need to set the size of the ``buffer`` that they can manage.
By default this size is set to 255 characters.

``DynamicTypeBuilderFactory`` exposes the functions ``create_string_type`` and ``create_wstring_type``
to allow users create the Dynamic Types avoiding the ``DynamicTypeBuilder`` step.
The ``DynamicTypeBuilder`` can still be used to create String type dynamic data,
as shown on the example below.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRINGS
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_alias:

Alias
-----

Alias types provide an alternative name to an already existing type.
Once the ``DynamicData`` is created, users can access its information as if
they were working with the base type.

``DynamicTypeBuilderFactory`` exposes the function ``create_alias_type`` to allow users
create the Alias types avoiding the ``DynamicTypeBuilder`` step.
The ``DynamicTypeBuilder`` can still be used to create Alias,
as shown on the example below.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ALIAS
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_enumeration:

Enumeration
-----------

An enumeration contains a set of supported values and a selected value among those supported.
The supported values must be configured using the ``DynamicTypeBuilder``, using the ``add_member`` function
for each supported value.
The input to this function is the index and the name of the value we want to add.

The ``DynamicData`` class has functions ``get_enum_value`` and ``set_enum_value`` to work
with value index or value name name strings.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ENUMERATIONS
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_bitmask:

Bitmask
-------

Bitmasks are similar to `enumeration` types, but their members work as bit flags that can be individually turned on and
off. Bit operations can be applied when testing or setting a bitmask value.
``DynamicData`` has the special functions ``get_bitmask_value`` and ``set_bitmask_value`` which allow to retrieve or
modify the full value instead of accessing each bit.

Bitmasks can be bound to any number of bits up to 64.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_BITMASKS
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_structure:

Structure
---------

Structures are the common complex types, they allow to add any kind of members inside them.
They don't have any value, they are only used to contain other types.

To manage the types inside the structure, users can call the ``get`` and ``set`` functions
according to the kind of the type inside the structure using their ``ids``.
If the structure contains a complex value, it should be used with ``loan_value`` to
access to it and ``return_loaned_value`` to release that pointer.
``DynamicData`` manages the counter of loaned values and users can't loan a value that
has been loaned previously without calling ``return_loaned_value`` before.

The ``Ids`` must be consecutive starting by zero, and the ``DynamicType`` will change that
Id if it doesn't match with the next value.
If two members have the same Id, after adding the second one, the previous
will change its Id to the next value.
To get the Id of a member by name, ``DynamicData`` exposes the function ``get_member_id_by_name``.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRUCTS
   :end-before: //!--
   :dedent: 8

Structures allow inheritance, exactly with the same OOP meaning. To inherit from another structure, we must create the
structure calling the ``create_child_struct_builder`` of the factory. This function is shared with bitsets and will
deduce our type depending on the parent's type.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_STRUCTS-INHERIT
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_bitset:

Bitset
------

Bitset types are similar to `structure` types, but their members are merely `bitfields`, which are stored optimally.
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

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_BITSETS
   :end-before: //!--
   :dedent: 8

Bitsets allows inheritance, exactly with the same OOP meaning. To inherit from another bitset, we must create the
bitset calling the ``create_child_struct_builder`` of the factory. This function is shared with structures and will
deduce our type depending on the parent's type.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_BITSETS-INHERIT
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_union:

Union
-----

Unions are a special kind of structures where only one of the members is active
at the same time.
To control these members, users must set the ``discriminator`` type that is going to be used
to select the current member calling the ``create_union_builder`` function.
The ``discriminator`` itself is a Dynamic Type of any primitive type, string type or union type.

Every member that is going to be added needs at least one ``union_case_index`` to set
how it is going to be selected and, optionally, if it is the default value of the union.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_UNIONS
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_sequence:

Sequence
--------

A complex type that manages its members as a list of items allowing users to
insert, remove or access to a member of the list. To create this type users
need to specify the type that it is going to store and optionally the size
limit of the list.

To ease the memory management of this type, ``DynamicData`` has these functions:

 - ``insert_sequence_data``: Creates a new element at the end of the list and returns
   the ``id`` of the new element.
 - ``remove_sequence_data``: Removes the element of the given index and refreshes the ids
   to keep the consistency of the list.
 - ``clear_data``: Removes all the elements of the list.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_SEQUENCES
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_array:

Array
-----

Arrays are pretty similar to sequences with two main differences:
they can have multiple dimensions and they don't need that the elements
are stored consecutively.

An array needs to know the number of dimensions it is managing.
For that, users must provide a vector with as many elements as dimensions in the array.
Each element in the vector represents the size of the given dimension.
If the value of an element is set to zero, the default value applies ( :class:`100` ).

Id values on the ``set`` and ``get`` functions of ``DynamicData`` correspond to the array index.
To ease the management of array elements, every ``set`` function in ``DynamicData`` class creates
the item if the given index is empty.

To ease the memory management of this type, ``DynamicData`` has these functions:
 - ``insert_array_data``: Creates a new element at the end of the array and returns
   the ``id`` of the new element.
 - ``remove_array_data``: Clears the element of the given index.
 - ``clear_data``: Removes all the elements of the array.
 - ``get_array_index``: Returns the position id giving a vector of indexes on every dimension
   that the arrays support, which is useful in multidimensional arrays.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_ARRAYS
   :end-before: //!--
   :dedent: 8

.. _dynamictypes_supportedtypes_map:

Map
---

Maps contain a list of pairs 'key-value' types, allowing users to insert, remove or
modify the element types of the map.
The main difference with sequences is that the map works with pairs of elements and
creates copies of the key element to block the access to these elements.

To create a map, users must set the types of the key and the value elements, and,
optionally, the size limit of the map.

To ease the memory management of this type, ``DynamicData`` has these functions:
 - ``insert_map_data``: Inserts a new key value pair and returns the ids of the newly
   created key and value elements.
 - ``remove_map_data``: Uses the given id to find the key element and removes the key
   and the value elements from the map.
 - ``clear_data``: Removes all the elements from the map.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_CREATE_MAPS
   :end-before: //!--
   :dedent: 8

