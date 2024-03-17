.. _dynamic-types:

XTypes 1.3
==========

.. _DDS-XTypes V1.3: http://www.omg.org/spec/DDS-XTypes/1.3

eProsima Fast DDS provides a dynamic way to define and use topic types and topic data.
Our implementation follows the *OMG Extensible and Dynamic Topic Types for DDS interface*.
For more information, For more information, refer to the specification for `DDS-XTypes V1.3`_.

The dynamic topic types offer the possibility to work over RTPS without the restrictions related to the IDLs.
Using them, the users can declare the different types that they need and manage the information directly,
avoiding the additional step of updating the IDL file and the generation of *C++* classes.

.. toctree::
   :maxdepth: 3

   /fastdds/dynamic_types/discovery_matching.rst
   /fastdds/dynamic_types/language_binding.rst
