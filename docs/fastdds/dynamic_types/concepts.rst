.. _dynamictypes_concepts:

Concepts
========

This section defines basic concepts related to dynamic types, and that are used through the rest of
the documentation.

Type Descriptor
---------------

Stores the information about one type with its relationships and restrictions.
It's the minimum class needed to generate a Dynamic type and in case of the
complex ones, it stores information about its children or its parent types.

Member Descriptor
-----------------

Several complex types need member descriptors to declare the relationship between types.
This class stores information about that members like their name, their unique ID,
the type that is going to be created, and the default value after the creation.
Union types have special fields to identify each member by labels.

Dynamic Type Builder Factory
----------------------------

*Singleton* class that is in charge of the creation and the management of every
``DynamicTypes`` and ``DynamicTypeBuilders``.
It declares functions to create each kind of supported types, making easier the
management of the descriptors.
Every object created by the factory must be deleted calling the ``delete_type`` function.

Dynamic Type Builder
--------------------

Intermediate class used to configure and create ``DynamicTypes``.
By design Dynamic types can't be modified, so the previous step to create a new one is to create a builder and apply
the settings that the user needs.
Users can create several types using the same builder, but the changes applied
to the builder don't affect to the types created previously.
Every object created by a builder must be deleted calling the ``delete_type`` function
of the Dynamic Type builder Factory.

Dynamic Type
------------

Base class in the declaration of Dynamic types, it stores the information about
its type and every Member that is related to it.
It creates a copy of the descriptor on its creation and cannot be changed to keep the consistency.

Dynamic Type Member
-------------------

A class that creates the relationship between a member descriptor with its parent type.
Dynamic Types have a one Dynamic type member for every child member added to it.

Dynamic Data Factory
--------------------

*Singleton* class that is in charge of the creation and the management of every
``DynamicData``.
It creates them using the given ``DynamicType`` with its settings.
Every data object created by the factory must be deleted calling the ``delete_type`` function.
Allows creating a ``TypeIdentifier`` and a (Minimal and Complete) ``TypeObject`` from a ``TypeDescriptor``.

Dynamic Data
------------

A class that manages the data of the Dynamic Types. It stores the information that is
sent and received.
There are two ways to work with DynamicDatas, the first one is the
most secured, activating the macro ``DYNAMIC_TYPES_CHECKING``, it creates a variable for
each primitive kind to help the debug process.
The second one reduces the size of the ``DynamicData`` class using only the minimum
values and making the code harder to debug.

Dynamic PubSubType
------------------

A class that inherits from ``TopicDataType`` and works as an intermediary between RTPS Domain and the Dynamic Types.
It implements the functions needed to create, serialize, deserialize and delete ``DynamicData`` instances when the
participants need to convert the received information from any transport to the registered dynamic type.


