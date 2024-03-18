.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _xmldynamictypes:

Dynamic Types profiles
----------------------

*Fast DDS* supports the implementation of |DynamicTypes| by defining them through XML files.
Thus the :ref:`Dynamic Types <dynamic-types>` can be modified without the need to modify the source code of the DDS
application.

XML Structure
^^^^^^^^^^^^^

The definition of type profiles in the XML file is done with the ``<types>`` tag.
Each ``<types>`` element can contain one or more :ref:`Type definitions <Type definition>`.
Defining several types within a ``<types>`` element or a single type for each ``<types>`` element has the same
result.
Below, an example of a stand-alone types definition via XML is shown.

.. literalinclude:: /../code/XMLTesterSkipValidation.xml
    :language: xml
    :start-after: <!--STAND_ALONE_TYPES_START-->
    :end-before: <!--STAND_ALONE_TYPES_END-->

.. note::

    For more information on the difference between stand-alone and rooted definitions please refer to section
    :ref:`rootedvsstandalone`.

.. _Type definition:

Type definition
^^^^^^^^^^^^^^^
Below, the types supported by *eProsima Fast DDS* are presented.
For further information about the supported |DynamicTypes|, please, refer to :ref:`xtypes_supportedtypes`.
For each of the types detailed below, an example of how to build the type's XML profile is provided.

*   `Member types`_

    -  `Primitive types`_
    -  `Bounded strings`_
    -  `Sequences`_
    -  `Arrays`_
    -  `Maps`_

*   `Enumerations`_
*   `Typedef`_
*   `Struct`_
*   `Union`_
*   `Bitset`_
*   `Bitmask`_
*   `Complex types`_

Member types
""""""""""""

Member types are defined as any type that can belong to a `Struct`_ or a `Union`_, or be aliased by a
`Typedef`_.
These can be defined by the ``<member>`` XML tag.

.. _xmldynamictypes_primivites:

Primitive types
***************

The identifiers of the available basic types are listed in the table below.
Please, refer to :ref:`xtypes_supportedtypes_primitive` for more information on the primitive types.

.. list-table::

  * - ``boolean``
    - ``char8``
    - ``char16``
  * - ``byte``
    - ``octet``
    - ``uint8``
  * - ``int8``
    - ``int16``
    - ``int32``
  * - ``uint16``
    - ``uint32``
    - ``int64``
  * - ``uint64``
    - ``float32``
    - ``float64``
  * - ``float128``
    - ``string``
    - ``wstring``

All of them are defined as follows:

.. tabs::

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_PRIMITIVES<-->
        :end-before: <!--><-->

   .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_PRIMITIVES
        :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Primitives <xtypes_supportedtypes_primitive>`.

.. _xmldynamictypes_strings:

Bounded strings
***************

Bounded strings are defined as any other ``string`` or ``wstring`` but adding the attribute ``stringMaxLength`` with the
maximum length available for that specific string.
Please, refer to :ref:`xtypes_supportedtypes_string` for more information on string type.

.. tabs::

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_STRINGS<-->
        :end-before: <!--><-->

   .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_STRINGS
        :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Strings <xtypes_supportedtypes_string>`.

.. _xmldynamictypes_sequence:

Sequences
*********

The sequence type is implemented by setting three attributes: ``name``, ``type``, and the
``sequenceMaxLength``.
The type of its content should be defined by the ``type`` attribute.
Please, refer to :ref:`xtypes_supportedtypes_sequence` section for more information on sequence type.

.. tabs::

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_SEQUENCES<-->
        :end-before: <!--><-->

   .. tab:: C++

      .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //!--CPP_SEQUENCES
        :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Sequences <xtypes_supportedtypes_sequence>`.

.. _xmldynamictypes_array:

Arrays
******

Arrays are defined in the same way as any other member type but they add the attribute ``arrayDimensions``.
The format of the ``arrayDimensions`` attribute value is the size of each dimension separated by commas.
Please, refer to :ref:`xtypes_supportedtypes_array` explanation for more information on array type.

.. tabs::

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->XML_ARRAYS<-->
      :end-before: <!--><-->

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--CPP_ARRAYS
      :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Arrays <xtypes_supportedtypes_array>`.

.. _xmldynamictypes_map:

Maps
****

Maps are similar to sequences, but they need to define two content types.
The ``key_type`` defines the type of the map key, while the ``type`` defines the map value type.
See section :ref:`xtypes_supportedtypes_map` for more information on map type.

.. tabs::

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->XML_MAPS<-->
      :end-before: <!--><-->

  .. tab:: C++

    .. literalinclude:: /../code/DDSCodeTester.cpp
      :language: c++
      :start-after: //!--CPP_MAPS
      :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Maps <xtypes_supportedtypes_map>`.

.. _xmlxtypes_complextypes:

Complex types
"""""""""""""

The complex types are a combination of the aforementioned types.
Complex types can be defined using the ``<member>`` element in the same way a basic or an array type would be.
The ``type`` in this case is ``nonBasic`` (not a `Primitive types`_) and the name of the previously defined type is
given in the ``nonBasicTypeName`` attribute.
Please, refer to :ref:`xtypes_complextypes` section for more information on complex types.

The following example shows structure having another structure as a member.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_COMPLEX_STRUCTS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_COMPLEX_STRUCTS
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Nested Types <xtypes_nested_structures>`.

This example shows union having another union as a member.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_COMPLEX_UNIONS<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_COMPLEX_UNIONS
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Nested Types <xtypes_nested_unions>`.

Complex types attributes
************************

The attributes of a complex type element can be highly varied depending on the type being defined.
Since the attributes that can be defined for each of the types have already been listed,
these attributes are then defined in the following table.

.. list-table::
    :header-rows: 1

    *   - Name
        - Description
    *   - ``type``
        - Data type.
          This can be a `Primitive types`_ or a ``nonBasic`` type. |br|
          The latter is used to denote that a complex type is defined.
    *   - ``nonBasicTypeName``
        - Name of the complex type. Only applies if the ``type`` attribute is set to ``nonBasic``.
    *   - ``stringMaxLength``
        - Maximum length of a string.
    *   - ``sequenceMaxLength``
        - Maximum length of a `Sequences`_.
    *   - ``arrayDimensions``
        - Dimensions of an array.
    *   - ``key_type``
        - Data type of a map key.
    *   - ``mapMaxLength``
        - Maximum length of a `Maps`_.

.. _xmldynamictypes_typedef:

Typedef
"""""""

The ``<typedef>`` XML element is defined by a ``name`` and a ``type`` mandatory attributes, and any of the optional
attributes presented in `Complex types attributes`_ section.
This element allows for defining complex types without the need to define them previously as members.
Maps, arrays and sequences can be elements within another container using ``<typedef>``.
The ``<typedef>`` element corresponds to :ref:`xtypes_supportedtypes_alias` in :ref:`xtypes_supportedtypes`
section.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_TYPEDEF<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_TYPEDEF
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Alias <xtypes_supportedtypes_alias>`.

.. _xmldynamictypes_enums:

Enumerations
""""""""""""

The ``<enum>`` type is defined by its attribute ``name`` and a set of ``<enumerator>`` child elements.
Each ``<enumerator>`` is defined by two attributes: a mandatory ``name`` and an optional unsigned integer ``value``.
Please, refer to :ref:`xtypes_supportedtypes_enumeration` for more information on the ``<enum>`` type.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_ENUM<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_ENUM
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Enumerations <xtypes_supportedtypes_enumeration>`.

.. _xmldynamictypes_struct:

Struct
""""""

The ``<struct>`` element is defined by its ``name`` attribute and its ``<member>`` child elements.
Please, refer to :ref:`xtypes_supportedtypes_structure` for more information on the ``<struct>`` type.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_STRUCT<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_STRUCT
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Structures <xtypes_supportedtypes_structure>`.

.. _xmldynamictypes_struct_inheritance:

Structs can inherit from another struct.
This is implemented by defining the value of the ``baseType`` attribute, on the child ``<struct>`` element to be the
value of the ``name`` attribute of the parent ``<struct>`` element.
This is exemplified by the code snippet below.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_STRUCT_INHERITANCE<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_STRUCT_INHERITANCE
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Structures Inheritance <xtypes_structure_inheritance>`.

.. _xmldynamictypes_union:

Union
"""""

The ``<union>`` type is defined by a ``name`` attribute, a ``<discriminator>`` child element and a set of ``<case>``
child elements.
The ``<discriminator>`` must define its ``type``
Each ``<case>`` element has one or more ``<caseDiscriminator>`` elements, which type must be consistent with the
``<discriminator>`` type, and a unique ``<member>`` element.
Please, refer to :ref:`xtypes_supportedtypes_union` for more information on the ``<union>`` type.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_UNION<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_UNION
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Unions <xtypes_supportedtypes_union>`.

.. _xmldynamictypes_bitset:

Bitset
""""""

The ``<bitset>`` element defines the :ref:`xtypes_supportedtypes_bitset` type.
It is comprised by a ``name`` attribute and a set of ``<bitfield>`` child elements.
In turn, the ``<bitfield>`` element has the mandatory ``bit_bound`` attribute, which cannot be higher than 64, and
two optional attributes: ``name`` and ``type``.
A ``<bitfield>`` without ``name`` attribute is an inaccessible set of bits.
Its management ``type`` can ease the ``<bitfield>`` modification and access.
Please, refer to :ref:`xtypes_supportedtypes_bitset` for more information about the ``<bitset>`` type.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITSET<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITSET
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Bitset <xtypes_supportedtypes_bitset>`.

.. _xmlxtypes_bitset_inheritance:

Moreover, bitsets can inherit from another bitsets.
This is implemented by defining the value of the ``baseType`` attribute, on the child ``<bitset>`` element to be the
value of the ``name`` attribute of the parent ``<bitset>`` element.
This is exemplified by the code snippet below.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITSET_INHERITANCE<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITSET_INHERITANCE
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Bitset Inheritance <xtypes_bitset_inheritance>`.

.. _xmldynamictypes_bitmask:

Bitmask
"""""""

The ``<bitmask>`` element, which corresponds to the :ref:`xtypes_supportedtypes_bitmask` type, is defined by
a mandatory ``name`` attribute, an optional ``bit_bound`` attribute, and several ``<bit_value>`` child elements.
The ``bit_bound`` attribute specifies the number of bits that the bitmask type will manage.
The maximum value allowed for the ``bit_bound`` is 64.
The ``<bit_value>`` element must define the ``name`` attribute and it might define its position in the bitmask setting
the ``positition`` attribute.
Please, refer to :ref:`xtypes_supportedtypes_bitmask` for more information on the ``<bitmask>`` type.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITMASK<-->
            :end-before: <!--><-->

    .. tab:: C++

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //!--CPP_BITMASK
            :end-before: //!--

For a full example of how to define this type, please refer to :ref:`Bitmasks <xtypes_supportedtypes_bitmask>`.

.. _Usage:

Loading dynamic types in a *Fast DDS* application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the *Fast DDS* application that will make use of the *XML Types*, the XML files that
define the types must be loaded before trying to instantiate |DynamicPubSubType-api| objects of these types.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //XML-USAGE
    :end-before: //!--
    :dedent: 8
