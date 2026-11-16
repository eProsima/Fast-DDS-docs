.. _dynamic-types:

XTypes
======

.. _Extensible and Dynamic Topic Types for DDS: https://www.omg.org/spec/DDS-XTypes/1.3

*eProsima Fast DDS* supports the OMG `Extensible and Dynamic Topic Types for DDS`_ specification (also known as XTypes).
This specification defines the following concepts:

* DDS supported type system, including the concept of extensible types that might evolve in time.
* Type representation, including :ref:`IDL<fastddsgen_idl_datatypes>` and :ref:`TypeObject<xtypes_discovery_matching>`
  representations.
* Data representation over the wire.
* Language binding, defining both a plain and a dynamic language binding.
  :ref:`eProsima Fast DDS-Gen<fastddsgen_intro>` generates the plain language binding given an IDL type representation.
  :ref:`xtypes_language_binding` section explains the required API to define and set/read the data types.
* DDS builtin mechanism to automatically discover remote data types.
  More information in :ref:`xtypes_discovery_matching` section.

.. toctree::
   :maxdepth: 3

   /fastdds/xtypes/discovery_matching.rst
   /fastdds/xtypes/language_binding.rst
   /fastdds/xtypes/type_serializing.rst
   /fastdds/xtypes/idl_parsing.rst

.. note::

  Type compatibility rules among evolved types are still unsupported in Fast DDS.
