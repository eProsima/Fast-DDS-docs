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
Below, the types supported by *Fast DDS* are presented .
For further information about the supported |DynamicTypes|, please, refer to :ref:`dynamictypes_supportedtypes`.
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
A member can be annotated as ``key`` (equivalent of the IDL's ``@key``) by setting the ``key`` attribute to ``"true"``.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-MEMBER_WITH_KEY<-->
  :end-before: <!--><-->

Primitive types
***************

The identifiers of the available basic types are listed in the table below.
Please, refer to :ref:`dynamictypes_supportedtypes_primitive` for more information on the primitive types.

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

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-GENERIC<-->
  :end-before: <!--><-->

Bounded strings
***************

Bounded strings are defined as any other ``string`` or ``wstring`` but adding the attribute ``stringMaxLength`` with the
maximum length available for that specific string.
Please, refer to :ref:`dynamictypes_supportedtypes_string` for more information on string type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-BOUNDEDSTRINGS<-->
  :end-before: <!--><-->

Sequences
*********

The sequence type is implemented by setting three attributes: ``name``, ``type``, and the
``sequenceMaxLength``.
The type of its content should be defined by the ``type`` attribute.
Please, refer to :ref:`dynamictypes_supportedtypes_sequence` section for more information on sequence type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-SEQUENCES<-->
  :end-before: <!--><-->

Arrays
******

Arrays are defined in the same way as any other member type but they add the attribute ``arrayDimensions``.
The format of the ``arrayDimensions`` attribute value is the size of each dimension separated by commas.
Please, refer to :ref:`dynamictypes_supportedtypes_array` explanation for more information on array type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-ARRAYS<-->
  :end-before: <!--><-->

Maps
****

Maps are similar to sequences, but they need to define two content types.
The ``key_type`` defines the type of the map key, while the ``type`` defines the map value type.
See section :ref:`dynamictypes_supportedtypes_map` for more information on map type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-MAPS<-->
  :end-before: <!--><-->

Complex types
"""""""""""""

The complex types are a combination of the aforementioned types.
Complex types can be defined using the ``<member>`` element in the same way a basic or an array type would be.
The ``type`` in this case is ``nonBasic`` (not a `Primitive types`_) and the name of the previously defined type is
given in the ``nonBasicTypeName`` attribute.
Please, refer to :ref:`dynamictypes_complextypes` section for more information on complex types.

The following example shows a new structure with the ``primitive_types_example`` struct defined in `Primitive types`_
example.

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-COMPLEX<-->
    :end-before: <!--><-->

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

Typedef
"""""""

The ``<typedef>`` XML element is defined by a ``name`` and a ``type`` mandatory attributes, and any of the optional
attributes presented in `Complex types attributes`_ section.
This element allows for defining complex types without the need to define them previously as members.
Maps, arrays and sequences can be elements within another container using ``<typedef>``.
The ``<typedef>`` element corresponds to :ref:`dynamictypes_supportedtypes_alias` in :ref:`dynamictypes_supportedtypes`
section.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-TYPEDEF<-->
  :end-before: <!--><-->

Enumerations
""""""""""""

The ``<enum>`` type is defined by its attribute ``name`` and a set of ``<enumerator>`` child elements.
Each ``<enumerator>`` is defined by two attributes: a mandatory ``name`` and an optional unsigned integer ``value``.
Please, refer to :ref:`dynamictypes_supportedtypes_enumeration` for more information on the ``<enum>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-DYN-ENUM<-->
  :end-before: <!--><-->

Struct
""""""

The ``<struct>`` element is defined by its ``name`` attribute and its ``<member>`` child elements.
Please, refer to :ref:`dynamictypes_supportedtypes_structure` for more information on the ``<struct>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-STRUCT<-->
  :end-before: <!--><-->

Structs can inherit from another struct.
This is implemented by defining the value of the ``baseType`` attribute, on the child ``<struct>`` element to be the
value of the ``name`` attribute of the parent ``<struct>`` element.
This is exemplified by the code snippet below.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-STRUCT-INHERIT<-->
  :end-before: <!--><-->

Union
"""""

The ``<union>`` type is defined by a ``name`` attribute, a ``<discriminator>`` child element and a set of ``<case>``
child elements.
The ``<discriminator>`` must define its ``type``
Each ``<case>`` element has one or more ``<caseDiscriminator>`` elements, which type must be consistent with the
``<discriminator>`` type, and a unique ``<member>`` element.
Please, refer to :ref:`dynamictypes_supportedtypes_union` for more information on the ``<union>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-UNION<-->
  :end-before: <!--><-->

Bitset
""""""

The ``<bitset>`` element defines the :ref:`dynamictypes_supportedtypes_bitset` type.
It is comprised by a ``name`` attribute and a set of ``<bitfield>`` child elements.
In turn, the ``<bitfield>`` element has the mandatory ``bit_bound`` attribute, which cannot be higher than 64, and
two optional attributes: ``name`` and ``type``.
A ``<bitfield>`` without ``name`` attribute is an inaccessible set of bits.
Its management ``type`` can ease the ``<bitfield>`` modification and access.
Please, refer to :ref:`dynamictypes_supportedtypes_bitset` for more information about the ``<bitset>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-BITSET<-->
  :end-before: <!--><-->

Moreover, bitsets can inherit from another bitsets:

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-BITSET-INHERIT<-->
  :end-before: <!--><-->

Bitmask
"""""""

The ``<bitmask>`` element, which corresponds to the :ref:`dynamictypes_supportedtypes_bitmask` type, is defined by
a mandatory ``name`` attribute, an optional ``bit_bound`` attribute, and several ``<bit_value>`` child elements.
The ``bit_bound`` attribute specifies the number of bits that the bitmask type will manage.
The maximum value allowed for the ``bit_bound`` is 64.
The ``<bit_value>`` element must define the ``name`` attribute and it might define its position in the bitmask setting
the ``positition`` attribute.
Please, refer to :ref:`dynamictypes_supportedtypes_bitmask` for more information on the ``<bitmask>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-BITMASK<-->
  :end-before: <!--><-->

.. _Usage:

Loading dynamic types in a *Fast DDS* application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the *Fast DDS* application that will make use of the *XML Types*, the XML files that
define the types must be loaded before trying to instantiate |DynamicPubSubType| objects of these types.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //XML-USAGE
    :end-before: //!--
    :dedent: 8

