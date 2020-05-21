.. _dynamictypes_overview:

Overview of Dynamic Types
=========================

This section describes the classes related to dynamic types, and that are used through the rest of the documentation.
At the bottom of the section you can also find a short example using the functionality.

Involved classes
----------------

The following class diagram describes the relationship among the classes related to dynamic types.
Please, refer to the description of each class to find its purpose and the nature of the relationship
with the rest of the classes.

.. figure:: /01-figures/dynamic_types_class_diagram.svg
    :align: center

    Dynamic types class diagram

* :ref:`dynamictypes_overview_dynamictype`
* :ref:`dynamictypes_overview_dynamictypebuilderfactory`
* :ref:`dynamictypes_overview_dynamictypebuilder`
* :ref:`dynamictypes_overview_typedescriptor`
* :ref:`dynamictypes_overview_dynamictypemember`
* :ref:`dynamictypes_overview_memberdescriptor`
* :ref:`dynamictypes_overview_dynamicdata`
* :ref:`dynamictypes_overview_dynamicdatafactory`
* :ref:`dynamictypes_overview_dynamicpubsubtype`

.. _dynamictypes_overview_dynamictype:

Dynamic Type
^^^^^^^^^^^^

Base class of all types declared dynamically.
It represents a dynamic data type that can be used to create
:ref:`dynamictypes_overview_dynamicdata` values.
By design, the structure of a dynamic type (its member fields) cannot
be modified once the type is created.


.. _dynamictypes_overview_dynamictypebuilderfactory:

Dynamic Type Builder Factory
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Singleton* class that is in charge of the creation and the management of every
:ref:`dynamictypes_overview_dynamictype` and :ref:`dynamictypes_overview_dynamictypebuilder`.
It declares functions to create builders for each kind of supported types.
Given a builder for a specific type, it can also create the corresponding
:ref:`dynamictypes_overview_dynamictype`.
Some simpler types can be created directly, avoiding the step of creating a
:ref:`dynamictypes_overview_dynamictypebuilder`.
Please, refer to the :ref:`dynamictypes_supportedtypes` documentation for details about
which ones support this option.

Every object created by the factory must be deleted to avoid memory leaking.
Refer to the :ref:`dynamictypes_memorymanagement` section for details.


.. _dynamictypes_overview_dynamictypebuilder:

Dynamic Type Builder
^^^^^^^^^^^^^^^^^^^^

Intermediate class used to configure a :ref:`dynamictypes_overview_dynamictype`
before it is created.
By design, the structure of a :ref:`dynamictypes_overview_dynamictype` (its member fields) cannot
be modified once the object is created.
Therefore, all its structure must be defined prior to its creation.
The builder is the object used to set up this structure.

Once defined, the :ref:`dynamictypes_overview_dynamictypebuilderfactory` is used to create
the :ref:`dynamictypes_overview_dynamictype` from the information contained in the builder.
As a shortcut, the builder exposes a function ``build`` that internally uses the
:ref:`dynamictypes_overview_dynamictypebuilderfactory` to return a fully constructed
:ref:`dynamictypes_overview_dynamictype`.
The types created with ``build`` are still subject to the :ref:`dynamictypes_memorymanagement`
restrictions, and must be deleted by the :ref:`dynamictypes_overview_dynamictypebuilderfactory`.

Builders can be reused after the creation of a :ref:`dynamictypes_overview_dynamictype`, as
the changes applied to the builder do not affect to types created previously.


.. _dynamictypes_overview_typedescriptor:

Type Descriptor
^^^^^^^^^^^^^^^

Stores the information about one type with its relationships and restrictions.
This is the class that describes the inner structure of a
:ref:`dynamictypes_overview_dynamictype`.
The :ref:`dynamictypes_overview_dynamictypebuilder` has an internal instance of
:class:`TypeDescriptor` that modifies during the type building process.
When the :ref:`dynamictypes_overview_dynamictype` is created, the
:ref:`dynamictypes_overview_dynamictypebuilderfactory` uses the information
of the :class:`TypeDescriptor` in the builder to create the :ref:`dynamictypes_overview_dynamictype`.
During the creation, the :class:`TypeDescriptor` is copied to the :ref:`dynamictypes_overview_dynamictype`,
so that it becomes independent from the :ref:`dynamictypes_overview_dynamictypebuilder`,
and the builder can be reused for another type.


.. _dynamictypes_overview_dynamictypemember:

Dynamic Type Member
^^^^^^^^^^^^^^^^^^^

Represents a data member of a :ref:`dynamictypes_overview_dynamictype` that is also a
:ref:`dynamictypes_overview_dynamictype`.
Compound types (dynamic types that are composed of other dynamic types) have a
:class:`DynamicTypeMember` for every child :ref:`dynamictypes_overview_dynamictype` added to it.


.. _dynamictypes_overview_memberdescriptor:

Member Descriptor
^^^^^^^^^^^^^^^^^

Just as a :ref:`dynamictypes_overview_typedescriptor` describes the inner structure of a
:ref:`dynamictypes_overview_dynamictype`,
a :class:`MemberDescriptor` stores all the information needed to manage a
:ref:`dynamictypes_overview_dynamictypemember`, like their name, their unique ID, or
the default value after the creation.
This information is copied to the :ref:`dynamictypes_overview_dynamicdata` on its creation.


.. _dynamictypes_overview_dynamicdata:

Dynamic Data
^^^^^^^^^^^^

While a :ref:`dynamictypes_overview_dynamictype` *describes* a type,
:class:`DynamicData` represents a data instance of a :ref:`dynamictypes_overview_dynamictype`.
It provides functions to access and modify the data values in the instance.

There are two ways to work with :class:`DynamicData`:

* Activating the macro ``DYNAMIC_TYPES_CHECKING``, which creates a variable for
  each primitive kind to help the debug process.
* Without this macro, the size of the :class:`DynamicData` is reduced, using only the minimum needed
  internal values, but it makes the code harder to debug.


.. _dynamictypes_overview_dynamicdatafactory:

Dynamic Data Factory
^^^^^^^^^^^^^^^^^^^^

*Singleton* class that is in charge of the creation and the management of every
:ref:`dynamictypes_overview_dynamicdata`.
It can take a :ref:`dynamictypes_overview_dynamictype` and create an instance of a
corresponding :ref:`dynamictypes_overview_dynamicdata`.
Every data object created by the factory must be deleted to avoid memory leaking.
Refer to the :ref:`dynamictypes_memorymanagement` section for details.

It also allows to create a ``TypeIdentifier`` and a (Minimal and Complete) ``TypeObject`` from a ``TypeDescriptor``.


.. _dynamictypes_overview_dynamicpubsubtype:

Dynamic PubSubType
^^^^^^^^^^^^^^^^^^

This class is an adapter that allows using :ref:`dynamictypes_overview_dynamicdata` on Fast DDS.
It inherits from ``TopicDataType`` and implements the functions needed to communicate the
:ref:`dynamictypes_overview_dynamicdata` between Publishers and Subscribers.


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


