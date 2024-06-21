.. include:: ../../03-exports/aliases-api.include

.. _xtypes_type_serializing:

Dynamic Type Serializing
========================

Fast-DDS provides methods to serialize DynamicType objects.

Dynamic Type to IDL
-------------------

The method `idl_serialize` converts a DynamicType object into an IDL string.
The following `DynamicType` object would be serialized as follows:

.. tabs::

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_HELLO_WORLD
            :end-before: //!--

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_HELLO_WORLD<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_HELLO_WORLD
            :end-before: //!--

.. note::

    The conversion to IDL does not support :ref:`xtypes_supportedtypes_alias`, :ref:`xtypes_supportedtypes_bitset`, :ref:`xtypes_supportedtypes_bitmask`, or :ref:`xtypes_annotations`.

.. note::

    The conversion to IDL does not support inheritance of :ref:`xtypes_supportedtypes_structure`.
