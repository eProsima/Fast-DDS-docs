.. _xmldynamictypes:

XML Dynamic Types
-----------------

XML Dynamic Types allows creating *eProsima Fast RTPS Dynamic Types* directly defining them through XML.
It allows any application to change TopicDataTypes without modifying its source code.

XML Structure
^^^^^^^^^^^^^

The XML Types definition (``<types>`` tag) can be placed similarly to the profiles tag inside the XML file.
It can be a stand-alone XML Types file or be a child of the Fast-RTPS XML root tag (``<dds>``).
Inside the types tag, there must be one or more type tags (``<type>``).

Stand-Alone:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-- STAND ALONE TYPES START -->
    :end-before: <!-- STAND ALONE TYPES END -->

Rooted:

.. literalinclude:: /../code/XMLTesterAux.xml
    :language: xml
    :start-after: <!-- ROOTED TYPES START -->
    :end-before: <!-- ROOTED TYPES END -->

Finally, each ``<type>`` tag can contain one or more :ref:`Type definitions <Type definition>`.
Defining several types inside a ``<type>`` tag or defining each type in its ``<type>`` tag has the same result.

.. _Type definition:

Type definition
^^^^^^^^^^^^^^^

**Enum**

The ``<enum>`` type is defined by its ``name`` and a set of ``enumerators``,
each of them with its ``name`` and its (optional) ``value``.

Example:

+-----------------------------------------------+-------------------------------------------------------+
| XML                                           | C++                                                   |
+===============================================+=======================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp           |
|   :language: xml                              |     :language: cpp                                    |
|   :start-after: <!-->XML-DYN-ENUM<-->         |     :start-after: //XML-DYN-ENUM                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                                |
|                                               |                                                       |
+-----------------------------------------------+-------------------------------------------------------+

**Typedef**

The ``<typedef>`` type is defined by its ``name`` and its ``value`` or an inner element for complex types.
``<typedef>`` corresponds to :class:`Alias` in Dynamic Types glossary.

Example:

+-----------------------------------------------+------------------------------------------------------+
| XML                                           | C++                                                  |
+===============================================+======================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp          |
|   :language: xml                              |     :language: cpp                                   |
|   :start-after: <!-->XML-TYPEDEF<-->          |     :start-after: //XML-TYPEDEF                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                               |
|                                               |                                                      |
+-----------------------------------------------+------------------------------------------------------+

**Struct**

The ``<struct>`` type is defined by its ``name`` and inner *members*.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-STRUCT<-->           |     :start-after: //XML-STRUCT                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

Structs can inherit from another structs:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-STRUCT-INHERIT<-->   |     :start-after: //XML-STRUCT-INHERIT              |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+



**Union**

The ``<union>`` type is defined by its ``name``, a ``discriminator`` and a set of ``cases``.
Each ``case`` has one or more ``caseDiscriminator`` and a ``member``.


Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-UNION<-->            |     :start-after: //XML-UNION                       |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

**Bitset**

The ``<bitset>`` type is defined by its ``name`` and inner *bitfields*.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BITSET<-->           |     :start-after: //XML-BITSET                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

A bitfield without name is an inaccessible set of bits. Bitfields can specify their management type to ease their
modification and access. The bitfield's bit_bound is mandatory and cannot be bigger than 64.

Bitsets can inherit from another bitsets:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BITSET-INHERIT<-->   |     :start-after: //XML-BITSET-INHERIT              |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

**Bitmask**

The ``<bitmask>`` type is defined by its ``name`` and inner *bit_values*.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-BITMASK<-->          |     :start-after: //XML-BITMASK                     |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

The bitmask can specify its bit_bound, this is, the number of bits that the type will manage. Internally will be
converted to the minimum type that allows to store them. The maximum allowed bit_bound is 64.
Bit_values can define their position inside the bitmask.


Member types
^^^^^^^^^^^^

Member types are any type that can belong to a ``<struct>`` or a ``<union>``, or be aliased by a ``<typedef>``.

**Basic types**

The identifiers of the available basic types are:

+------------------------+------------------------+------------------------+
| ``boolean``            | ``int64``              | ``float128``           |
+------------------------+------------------------+------------------------+
| ``byte``               | ``uint16``             | ``string``             |
+------------------------+------------------------+------------------------+
| ``char``               | ``uint32``             | ``wstring``            |
+------------------------+------------------------+------------------------+
| ``wchar``              | ``uint64``             |                        |
+------------------------+------------------------+------------------------+
| ``int16``              | ``float32``            |                        |
+------------------------+------------------------+------------------------+
| ``int32``              | ``float64``            |                        |
+------------------------+------------------------+------------------------+


All of them are defined as follows:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-GENERIC<-->          |     :start-after: //XML-GENERIC                     |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

**Arrays**

Arrays are defined in the same way as any other member type but add the attribute ``arrayDimensions``.
The format of this dimensions attribute is the size of each dimension separated by commas.

Example:


+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-ARRAYS<-->           |     :start-after: //XML-ARRAYS                      |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+


It's IDL analog would be:

.. code-block:: c

    long long_array[2][3][4];

**Sequences**

Sequences are defined by its ``name``, its content ``type``, and its ``sequenceMaxLength``.
The type of its content should be defined by its ``type`` attribute.

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-SEQUENCES<-->        |     :start-after: //XML-SEQUENCES                   |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

The example shows a sequence with ``sequenceMaxLength`` ``3`` of sequences with ``sequenceMaxLength`` ``2``
with ``<int32>`` contents.
As IDL would be:

.. code-block:: c

    sequence<sequence<long,2>,3> my_sequence_sequence;

Note that the inner sequence has been defined before.

**Maps**

Maps are similar to sequences, but they need to define two types instead of one.
One type defines its ``key_type``, and the other type defines its elements types.
Again, both types can be defined as attributes or as members, but when defined
as members, they should be contained in another XML element (``<key_type>`` and ``<type>`` respectively).

Example:

+-----------------------------------------------+-----------------------------------------------------+
| XML                                           | C++                                                 |
+===============================================+=====================================================+
| .. literalinclude:: /../code/XMLTester.xml    | .. literalinclude:: /../code/CodeTester.cpp         |
|   :language: xml                              |     :language: cpp                                  |
|   :start-after: <!-->XML-MAPS<-->             |     :start-after: //XML-MAPS                        |
|   :end-before: <!--><-->                      |     :end-before: //!--                              |
|                                               |                                                     |
+-----------------------------------------------+-----------------------------------------------------+

Is equivalent to the IDL:

.. code-block:: c

    map<long,map<long,long,2>,2> my_map_map;

**Complex types**

Once defined, complex types can be used as members in the same way a basic or array type would be.

Example:

.. literalinclude:: /../code/XMLTester.xml
    :language: xml
    :start-after: <!-->XML-COMPLEX<-->
    :end-before: <!--><-->

.. _Usage:

Usage
^^^^^

In the application that will make use of *XML Types*, it's mandatory to load the XML file that defines
the types before trying to instantiate *DynamicPubSubTypes* of these types.
It's important to remark that only ``<struct>`` types generate usable *DynamicPubSubType* instances.

.. literalinclude:: /../code/CodeTester.cpp
    :language: cpp
    :start-after: //XML-USAGE
    :end-before: //!--
