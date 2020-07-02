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

.. literalinclude:: /../code/XMLTester.xml
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

*   `Enum`_
*   `Typedef`_
*   `Struct`_
*   `Union`_
*   `Bitset`_
*   `Bitmask`_
*   `Member types`_

    -  `Primitive types`_
    -  `Arrays`_
    -  `Sequences`_
    -  `Maps`_

*   `Complex types`_


Enum
""""

The ``<enum>`` type is defined by its attribute ``name`` and a set of ``<enumerator>`` child elements.
Each ``<enumerator>`` is defined by two attributes: a ``name`` and an optional ``value``.
Please, refer to :ref:`dynamictypes_supportedtypes_enumeration` for more information on the ``<enum>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-DYN-ENUM<-->
  :end-before: <!--><-->

Typedef
"""""""

The ``<typedef>`` XML element is defined by a ``name`` and a ``type`` mandatory attributes, and various optional
attributes for complex types definition.
These optional attributes are: ``key_type``, ``arrayDimensions``, ``nonBasicTypeName``, ``sequenceMaxLength``, and
``mapMaxLength``.
See `Complex types attributes`_ for more information on these attributes.
The ``<typedef>`` element corresponds to :ref:`dynamictypes_supportedtypes_alias` in :ref:`dynamictypes_supportedtypes`
section.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-TYPEDEF<-->
  :end-before: <!--><-->

Struct
""""""

The ``<struct>`` element is defined by its ``name`` attribute and its ``<member>`` child elements.
Please, refer to :ref:`dynamictypes_supportedtypes_structure` for more information on the ``<struct>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-STRUCT<-->
  :end-before: <!--><-->

Structs can inherit from another structs.
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
Each ``<case>`` element has one or more ``<caseDiscriminator>`` and a ``<member>`` child elements.
Please, refer to :ref:`dynamictypes_supportedtypes_union` for more information on the ``<union>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-UNION<-->
  :end-before: <!--><-->

Bitset
""""""

The ``<bitset>`` element defines the :ref:`dynamictypes_supportedtypes_bitset` type.
It is comprised by a ``name`` attribute and a set of ``<bitfield>`` child elements.
In turn, the ``<bitfield>`` element has the mandatory ``bit_bound`` attribute, which can not be higher than 64, and
two optional attributes:
``name`` and ``type``.
A ``<bitfield>`` with a blank ``name`` attribute is an inaccessible set of bits.
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
The ``bit_bound`` attribute specifies the number of bits that the type will manage.
The maximum value allowed for the ``bit_bound`` is 64.
The ``<bit_value>`` element can define its position in the bitmask setting the ``positition`` attribute.
Please, refer to :ref:`dynamictypes_supportedtypes_bitmask` for more information on the ``<bitmask>`` type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-BITMASK<-->
  :end-before: <!--><-->

Member types
""""""""""""

Member types are defined as any type that can belong to a ``<struct>`` or a ``<union>``, or be aliased by a
``<typedef>``.
These can be defined by the ``<member>`` XML tag.

Primitive types
***************

The identifiers of the available basic types are listed in the table below.
Please, refer to :ref:`dynamictypes_supportedtypes_primitive` for more information on the primitive types.

+--------------------------------------+---------------------------------------+---------------------------------------+
| ``bool``                             | ``int32_t``                           | ``float32``                           |
+--------------------------------------+---------------------------------------+---------------------------------------+
| ``byte``                             | ``int64_t``                           | ``float64``                           |
+--------------------------------------+---------------------------------------+---------------------------------------+
| ``char``                             | ``uint16_t``                          | ``float128``                          |
+--------------------------------------+---------------------------------------+---------------------------------------+
| ``wchar``                            | ``uint32_t``                          | ``string``                            |
+--------------------------------------+---------------------------------------+---------------------------------------+
| ``int16_t``                          | ``uint64_t``                          | ``wstring``                           |
+--------------------------------------+---------------------------------------+---------------------------------------+

All of them are defined as follows:

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-GENERIC<-->
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

Sequences
*********

The sequence type is implemented by setting three attributes: ``name``, the ``type``, and the
``sequenceMaxLength``.
The type of its content should be defined by the ``type`` attribute.
The following example shows the implementation of a sequence of maximum length equal to 3.
In turn, this is a sequence of sequences of maximum length of 2 and contents of type ``int32``.
Please, refer to :ref:`dynamictypes_supportedtypes_sequence` section for more information on sequence type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-SEQUENCES<-->
  :end-before: <!--><-->

Maps
****

Maps are similar to sequences, but they need to define two content types.
The ``key_type`` defines the type of the map key, while the ``type`` defines the map value type.
Again, both types can be defined as attributes of a ``<typedef>`` element, or as a ``<member>`` child element of a
``<struct>`` or ``<union>`` elements.
See section :ref:`dynamictypes_supportedtypes_map` for more information on map type.

.. literalinclude:: /../code/XMLTester.xml
  :language: xml
  :start-after: <!-->XML-MAPS<-->
  :end-before: <!--><-->

Complex types
"""""""""""""

The complex types are a combination of the aforementioned types.
Complex types can be defined using the ``<member>`` element in the same way a basic or an array type would be.
Please, refer to :ref:`dynamictypes_complextypes` section for more information on complex types.

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
    *   - ``arrayDimensions``
        - Dimensions of an array.
    *   - ``sequenceMaxLength``
        - Maximum length of a `Sequences`_.
    *   - ``mapMaxLength``
        - Maximum length of a `Maps`_.
    *   - ``key_type``
        - Data type of a map key.


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

