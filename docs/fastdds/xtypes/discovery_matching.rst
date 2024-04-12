.. include:: ../../03-exports/aliases-api.include

.. _xtypes_discovery_matching:

Remote Data Types Discovery
===========================

.. _DDS-XTypes specification: https://www.omg.org/spec/DDS-XTypes/1.3

`DDS-XTypes specification`_ defines an internal mechanism to discover the remote data types at runtime and match
depending on the extensible types compatibility rules configured using the :ref:`typeconsistencyenforcementqospolicy`.

.. note::

     *eProsima Fast DDS* does not support XTypes compatibility check yet.

The remote data type discovery mechanism is based on the exchange of the data type information optimized in order to
reduce the required bandwidth.
On the one hand, :code:`TypeInformation` structure defined in the IDL below (extracted from Annex B of the
`DDS-XTypes specification`_), is used to communicate the Topic Data Type and its dependencies.

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
     :language: idl
     :start-after: //!--TYPEINFORMATION
     :end-before: //!--

:code:`TypeInformation` includes the :code:`TypeIdentifier`, the data type information hashed which identifies almost
univocally the data type.
The data type information is contained in the :code:`TypeObject` union:

.. literalinclude:: /../code/DynamicTypesIDLExamples.idl
     :language: idl
     :start-after: //!--TYPEOBJECT
     :end-before: //!--

The :code:`CompleteTypeObject` includes the data type full description.
On the other hand, :code:`MinimalTypeObject` only includes the minimum required information in order to check type
compatibility.

.. important::

     Current TypeObject representation implementation does not support forward declarations or recursive data types
     defined using the :code:`@external` annotation.
     Please, remember to disable TypeObject generation code using :code:`-no-typeobjectsupport` option when generating
     the code using Fast DDS-Gen.

Prerequisites
-------------

The remote data type discovery feature only works if some requisites are met:

1. The local data types must be registered into the |ITypeObjectRegistry-api|.
   The types are automatically registered when calling |DomainParticipant::register_type-api| /
   |TypeSupport::register_type-api| if the code required for registration has been generated using
   :ref:`eProsima Fast DDS-Gen<fastddsgen_intro>`.
   Fast DDS-Gen generates the required files (*<IDLFileName>TypeObjectSupport.cxx/.hpp*) by default.

   .. note::

     :code:`-no-typeobjectsupport` option disables the generation of these files and effectively disables the
     discovery of remote types.

   .. note::

     Currently there is no support to register the TypeObject when the data type is defined using the
     :ref:`xtypes_language_binding`.
     Consequently, local types defined using the dynamic language binding API, are not going to be discovered by remote
     DomainParticipants.

2. :code:`TypeInformation` should be received with the DomainParticipant's endpoint discovery information.
   A DomainParticipant that does not inform about its :code:`TypeInformation` would not trigger the remote data type
   discovery mechanism.

If the prerequisites are not met, endpoint matching relies on type name and topic name in order to match the discovered
endpoints.

Remote types discovery example
------------------------------

Please, refer to :ref:`use-case-remote-type-discovery-and-matching` for more information about how to leverage this
feature.
