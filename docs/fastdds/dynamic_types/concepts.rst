.. include:: ../../03-exports/aliases.include

.. _dynamictypes_overview:

Overview of Dynamic Types
=========================

This section describes the classes related to dynamic types that are used through the rest of the documentation.
At the bottom of the section you can also find a short example using the functionality.

Involved classes
----------------

The following class diagram describes the relationship among the classes related to dynamic types.
Please, refer to the description of each class to find its purpose and the nature of the relationship
with the rest of the classes.

.. figure:: /01-figures/dynamic_types_class_diagram.svg
    :align: center

    Dynamic types class diagram

.. contents::
    :local:
    :backlinks: none
    :depth: 2

.. _dynamictypes_overview_dynamictype:

DynamicType
^^^^^^^^^^^^

Base class of all types declared dynamically.
It represents a dynamic data type that can be used to create
|DynamicData| values.
By design, the structure of a dynamic type (its member fields) cannot
be modified once the type is created.


.. _dynamictypes_overview_dynamictypebuilderfactory:

DynamicTypeBuilderFactory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Singleton* class that is in charge of the creation and the management of every
DynamicType and |DynamicTypeBuilder|.
It declares functions to create builders for each kind of supported types.
Given a builder for a specific type, it can also create the corresponding
DynamicType.
Some simpler types can be created directly, avoiding the step of creating a
DynamicTypeBuilder.
Please, refer to the |Supported Types| documentation for details about
which ones support this option.

Every object created by the factory must be deleted to avoid memory leaking.
Refer to the |Memory management| section for details.


.. _dynamictypes_overview_dynamictypebuilder:

DynamicTypeBuilder
^^^^^^^^^^^^^^^^^^^^

Intermediate class used to configure a DynamicType
before it is created.
By design, the structure of a DynamicType (its member fields) cannot
be modified once the object is created.
Therefore, all its structure must be defined prior to its creation.
The builder is the object used to set up this structure.

Once defined, the DynamicTypeBuilderFactory is used to create
the DynamicType from the information contained in the builder.
As a shortcut, the builder exposes a function :func:`build` that internally uses the
DynamicTypeBuilderFactory to return a fully constructed
DynamicType.
The types created with :func:`build` are still subject to the |Memory management|
restrictions, and must be deleted by the DynamicTypeBuilderFactory.

Builders can be reused after the creation of a DynamicType, as
the changes applied to the builder do not affect to types created previously.


.. _dynamictypes_overview_typedescriptor:

TypeDescriptor
^^^^^^^^^^^^^^^

Stores the information about one type with its relationships and restrictions.
This is the class that describes the inner structure of a
DynamicType.
The DynamicTypeBuilder has an internal instance of
TypeDescriptor that modifies during the type building process.
When the DynamicType is created, the
DynamicTypeBuilderFactory uses the information
of the TypeDescriptor in the builder to create the DynamicType.
During the creation, the TypeDescriptor is copied to the DynamicType,
so that it becomes independent from the DynamicTypeBuilder,
and the builder can be reused for another type.


.. _dynamictypes_overview_dynamictypemember:

DynamicTypeMember
^^^^^^^^^^^^^^^^^^^

Represents a data member of a DynamicType that is also a
DynamicType.
Compound types (dynamic types that are composed of other dynamic types) have a
DynamicTypeMember for every child DynamicType added to it.


.. _dynamictypes_overview_memberdescriptor:

MemberDescriptor
^^^^^^^^^^^^^^^^^

Just as a TypeDescriptor describes the inner structure of a
DynamicType,
a MemberDescriptor stores all the information needed to manage a
DynamicTypeMember, like their name, their unique ID, or
the default value after the creation.
This information is copied to the |DynamicData| on its creation.


.. _dynamictypes_overview_dynamicdata:

DynamicData
^^^^^^^^^^^^

While a DynamicType *describes* a type,
DynamicData represents a data instance of a DynamicType.
It provides functions to access and modify the data values in the instance.

There are two ways to work with DynamicData:

* Activating the macro ``DYNAMIC_TYPES_CHECKING``, which creates a variable for
  each primitive kind to help the debug process.
* Without this macro, the size of the DynamicData is reduced, using only the minimum needed
  internal values, but it makes the code harder to debug.


.. _dynamictypes_overview_dynamicdatafactory:

DynamicDataFactory
^^^^^^^^^^^^^^^^^^^^

*Singleton* class that is in charge of the creation and the management of every
DynamicData.
It can take a DynamicType and create an instance of a
corresponding DynamicData.
Every data object created by the factory must be deleted to avoid memory leaking.
Refer to the |Memory management| section for details.

It also allows to create a :class:`TypeIdentifier` and a (Minimal and Complete) :class:`TypeObject` from a
:class:`TypeDescriptor`.


.. _dynamictypes_overview_dynamicpubsubtype:

DynamicPubSubType
^^^^^^^^^^^^^^^^^^

This class is an adapter that allows using DynamicData on Fast DDS.
It inherits from ``TopicDataType`` and implements the functions needed to communicate the
DynamicData between Publishers and Subscribers.


.. _dynamictypes_overview_example:

Minimum example
---------------

This is a short example to illustrate the use of the dynamic types and how the classes describe above interact
with each other.
While the code snippet can be used as a quick reference for code building, the sequence diagram below provides a
visual interpretation of the actions.

.. literalinclude:: /../code/CodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_TYPES_QUICK_EXAMPLE
   :end-before: //!
   :dedent: 8

.. figure:: /01-figures/dynamic_types_sequence_diagram.svg
    :align: center

    Sequence diagram of the code above

