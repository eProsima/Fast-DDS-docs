.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _xmldynamictypes:

Dynamic Types profiles
----------------------

*Fast DDS* supports the implementation of |DynamicTypes| by defining them through XML files.
Thus the topic data types can be modified without the need to modify the source code of the DDS application.

XML Structure
^^^^^^^^^^^^^

The definition of data type profiles in the XML file is done with the :code:`types` tag.
Each :code:`types` element can contain one or more :ref:`Type definitions <Type definition>`.
Defining several types within a :code:`types` element or a single type for each :code:`types` element has the same
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
For each type listed below, an example of how to build the type's XML profile is provided.

* `Primitive types`_
* `String types`_
* `Enumeration types`_
* `Bitmask types`_
* `Alias types`_
* `Sequence types`_
* `Array types`_
* `Map types`_
* `Structure types`_
* `Union types`_
* `Bitset types`_

.. _xmldynamictypes_primivites:

Primitive types
"""""""""""""""

Primitive types are built-in types and they should be declared as members of an aggregated type (`Structure types`_ or
`Union types`_).
Primitive types are declared by attribute :code:`type` and the possible values are listed in the table below.
Please, refer to :ref:`xtypes_supportedtypes_primitive` for more information on primitive types.

.. list-table::

  * - ``boolean``
    - ``byte``
    - ``char8``
    - ``char16``
    - ``int32``
  * - ``uint32``
    - ``int8``
    - ``uint8``
    - ``int16``
    - ``uint16``
  * - ``int64``
    - ``uint64``
    - ``float32``
    - ``float64``
    - ``float128``

All of them are declared as follows:

.. tabs::

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_PRIMITIVES<-->
        :end-before: <!--><-->

   .. tab:: IDL

      .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_PRIMITIVES
        :end-before: //!--


.. _xmldynamictypes_strings:

String Types
""""""""""""

String types should be defined as members of an aggregated type (`Structure types`_ or `Union types`_).
String types are defined with attribute :code:`type` set to :code:`string` or :code:`wstring`.
An optional attribute :code:`stringMaxLength` might used to set a maximum length for the string collection.
Please, refer to :ref:`xtypes_supportedtypes_string` for more information on string types.

.. tabs::

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_STRINGS<-->
        :end-before: <!--><-->

   .. tab:: IDL

      .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_STRINGS
        :end-before: //!--

.. _xmldynamictypes_enums:

Enumeration Types
"""""""""""""""""

Enumeration types are defined using the :code:`<enum>` tag.
Attribute :code:`name` and at least one :code:`<enumerator>` child element are mandatory.
Enumeration literals are defined using the :code:`<enumerator>` tag with mandatory attribute :code:`name`.
Optionally, unsigned integer attribute :code:`value` might be added to set a specific value for the enumeration literal.

.. note::

    :code:`value` attribute is equivalent to :code:`@value` builtin annotation which is not still supported in neither
    the plain (IDL) nor |DynamicTypes|.

Please, refer to :ref:`xtypes_supportedtypes_enumeration` for more information on enumeration types.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_ENUM<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_ENUM
            :end-before: //!--

.. _xmldynamictypes_bitmask:

Bitmask Types
"""""""""""""

Bitmask types are defined using the :code:`<bitmask>` tag.
Attribute :code:`name` and at least on :code:`<bit_value>` child element are mandatory.
Optionally, :code:`bit_bound` attribute might be set to specify the bitmask bound (by default 32 bits).
Bitflag elements are defined using the :code:`<bit_value>` tag with mandatory attribute :code:`name`.
Optionally, :code:`position` attribute might be defined to set the bitflag position within the bitmask.
Please, refer to :ref:`xtypes_supportedtypes_bitmask` for more information on bitmask types.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITMASK<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_BITMASK
            :end-before: //!--

.. _xmldynamictypes_typedef:

Alias Types
"""""""""""

Alias types are defined using the :code:`<typedef>` tag.
Attributes :code:`name` and :code:`type` are mandatory.
Depending on the aliased type, some other mandatory and/or optional attributes might be necessary or available.
Non-primitive types must define the :code:`type` attribute as :code:`nonBasic` and include the :code:`nonBasicTypeName`
attribute with the name of the aliased type.
Please, refer to :ref:`xtypes_supportedtypes_alias` for more information on alias types.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_TYPEDEF<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_TYPEDEF
            :end-before: //!--

.. _xmldynamictypes_sequence:

Sequence Types
""""""""""""""

Sequence types should be defined as members of an aggregated type (`Structure types`_ or `Union types`_).
Sequence types are defined with mandatory attributes :code:`type` set to the collection's element type, and
:code:`sequenceMaxLength` used to set the maximum collection's length.
Unbounded sequences should set :code:`sequenceMaxLength` attribute to :code:`-1`.
Please, refer to :ref:`xtypes_supportedtypes_sequence` for more information on sequence types.

.. tabs::

   .. tab:: XML

      .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->XML_SEQUENCES<-->
        :end-before: <!--><-->

   .. tab:: IDL

      .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
        :language: omg-idl
        :start-after: //!--IDL_SEQUENCES
        :end-before: //!--

.. _xmldynamictypes_array:

Array Types
"""""""""""

Array types should be defined as members of an aggregated type (`Structure types`_ or `Union types`_).
Array types are defined with mandatory attributes :code:`type` set to the collection's element type, and
:code:`arrayDimensions` used to set the collection's dimensions.
The format of :code:`arrayDimensions` attribute value is the size of each dimension separated by commas.
Please, refer to :ref:`xtypes_supportedtypes_array` for more information on array types.

.. tabs::

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->XML_ARRAYS<-->
      :end-before: <!--><-->

  .. tab:: IDL

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
      :language: omg-idl
      :start-after: //!--IDL_ARRAYS
      :end-before: //!--

.. _xmldynamictypes_map:

Map Types
"""""""""

Map types should be defined as members of an aggregated type (`Structure types`_ or `Union types`_).
Map types are defined with mandatory attributes :code:`type` set to the map's value type, :code:`key_type` set to the
map's key type, and :code:`mapMaxLength` used to set the maximum map's number of key-value pairs.
Unbounded maps should set :code:`mapMaxLength` attribute to :code:`-1`.
Please, refer to :ref:`xtypes_supportedtypes_map` for more information on map types.

.. tabs::

  .. tab:: XML

    .. literalinclude:: /../code/XMLTester.xml
      :language: xml
      :start-after: <!-->XML_MAPS<-->
      :end-before: <!--><-->

  .. tab:: IDL

    .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
      :language: omg-idl
      :start-after: //!--IDL_MAPS
      :end-before: //!--

.. _xmldynamictypes_struct:

Structure Types
"""""""""""""""

Structure types are defined using the :code:`<struct>` tag with mandatory attribute :code:`name`.
Structure inheritance may be configured setting optional attribute :code:`baseType`.
XML Structure Types require at least one member defined.

.. note::

    `IDL specification <https://www.omg.org/spec/IDL/4.2/About-IDL>`__ introduced in version 4.1 the possibility of void
    content structures.
    Empty structures are not supported in XML Types profiles yet.

Structure members are defined using the :code:`<member>` tag with mandatory attributes :code:`name` and :code:`type`.
Depending on the member type, some other mandatory and/or optional attributes might be necessary or available.
Non-primitive types must define the :code:`type` attribute as :code:`nonBasic` and include the :code:`nonBasicTypeName`
attribute with the name of the member type.

.. note::

    Currently, XML Types profiles does not support setting the member ID or marking a member as key.

Please, refer to :ref:`xtypes_supportedtypes_structure` for more information on structure types.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_STRUCT<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_STRUCT
            :end-before: //!--

.. _xmldynamictypes_union:

Union Types
"""""""""""

Union types are defined using the :code:`<union>` tag with mandatory attribute :code:`name`.
A mandatory discriminator child must be defined using :code:`<discriminator>` tag.
Discriminator element requires :code:`<type>` as mandatory attribute.

Union types also require at least one case child defined using the :code:`<case>` tag.
Each case child requires at least one label child using the :code:`<caseDiscriminator>` tag.
:code:`value` attribute is mandatory and defines the label value.
Several labels might be defined using several :code:`<caseDiscriminator>` elements.
Each case child must have exclusively one union member defined.

Union members are defined using the :code:`<member>` tag with mandatory attributes :code:`name` and :code:`type`.
Depending on the member type, some other mandatory and/or optional attributes might be necessary or available.
Non-primitive types must define the :code:`type` attribute as :code:`nonBasic` and include the :code:`nonBasicTypeName`
attribute with the name of the member type.
At least one union member must be defined for the union type to be consistent.

.. note::

    Currently, XML Types profiles does not support setting the member ID or marking a member as key.

Please, refer to :ref:`xtypes_supportedtypes_union` for more information on the union types.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_UNION<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_UNION
            :end-before: //!--

.. _xmldynamictypes_bitset:

Bitset Types
""""""""""""

Bitset types are defined using the :code:`<bitset>` tag with mandatory attribute :code:`name`.
Bitset inheritance may be configured setting optional attribute :code:`baseType`.
At least one bitfield child must be defined using :code:`bitfield` tag.

Bitfield elements require mandatory attribute :code:`bit_bound` with the number of bits managed by the bitfield (maximum
64 bits).
Optionally, attributes :code:`name` and :code:`type` might be defined.
An anonymous bitfield (attribute :code:`name` not set) is not accessible and serves as padding between named bitfields.
The :code:`type` attribute can ease bitfield management explicitly setting an integer type that handles the bitfield.

Please, refer to :ref:`xtypes_supportedtypes_bitset` for more information about the bitset types.

.. tabs::

    .. tab:: XML

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->XML_BITSET<-->
            :end-before: <!--><-->

    .. tab:: IDL

        .. literalinclude:: /../code/DynamicTypesIDLExamples.idl
            :language: omg-idl
            :start-after: //!--IDL_BITSET
            :end-before: //!--

.. _Usage:

Loading XML Types profile in *Fast DDS* application
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Fast DDS* application can use types defined in XML configuration files once those files have been loaded into the
|DomainParticipantFactory-api| using |DomainParticipantFactory::load_XML_profiles_file-api|.
Types might be retrieved using |DomainParticipantFactory::get_dynamic_type_builder_from_xml_by_name-api|.
After getting the DynamicType, objects of |DynamicPubSubType-api| class might be instantiated and used to write/read
data.

.. literalinclude:: /../code/DDSCodeTester.cpp
    :language: c++
    :start-after: //XML-USAGE
    :end-before: //!--
    :dedent: 8
