.. include:: ../../03-exports/aliases-api.include

.. _xtypes_type_serializing:

Dynamic Type Serializing
========================

Fast-DDS provides methods to serialize `DynamicType` objects.

Dynamic Type to IDL
-------------------

The method `idl_serialize` converts a `DynamicType` object into an IDL string.
The following `DynamicType` object would be serialized as follows:

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_HELLO_WORLD
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_HELLO_WORLD<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_HELLO_WORLD
            :end-before: //!--

.. note::

    The conversion to IDL only supports the annotations: `@bit_bound`, `@extensibility`, `@key`, and `@position`.

.. note::

    The conversion to IDL supports inheritance of :ref:`xtypes_supportedtypes_struct` fully, and of :ref:`xtypes_supportedtypes_bitset` at the cost of collapsing the base and derived bitsets into a single bitset.

Default values
++++++++++++++

In general, if a user explicitly sets a value to its default value, the serialization to IDL will not set the value explicitly.

Example: annotations
^^^^^^^^^^^^^^^^^^^^

In the following example, `MyBitMask` has two explicit annotations (`@bit_bound(32)` and `@position(0)`) that do not appear in the IDL, since they both are the default values.
The annotation `@position(2)` does appear, since it is not the default.
Furthermore, the annotation `@position(2)` increases the position of `flag2` to `3`, but the `@position(3)` annotation does not appear since by default the position is one more than the previous position.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_BITMASK_DEFAULT_ANNOTATIONS
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITMASK_DEFAULT_ANNOTATIONS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITMASK_DEFAULT_ANNOTATIONS
            :end-before: //!--

Example: bitsets
^^^^^^^^^^^^^^^^

In the following example, the three bitfields of `MyBitSet` explicitly specify their type.
Two of the types (`unsigned char` and `boolean`) are the default and, therefore, do not appear in the IDL.
The type `short` does appear since it is not the default type for a 12-bit long variable.

.. tabs::

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_BITSET_DEFAULT_TYPES
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITSET_DEFAULT_TYPES<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITSET_DEFAULT_TYPES
            :end-before: //!--
