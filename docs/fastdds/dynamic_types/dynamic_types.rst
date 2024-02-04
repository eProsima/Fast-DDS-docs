.. _dynamic-types:

Dynamic Topic Types
===================

.. _DDS-XTypes V1.3: http://www.omg.org/spec/DDS-XTypes/1.3

eProsima Fast DDS provides a dynamic way to define and use topic types and topic data.
Our implementation follows the *OMG Extensible and Dynamic Topic Types for DDS interface*.
For more information, you can read the specification for `DDS-XTypes V1.3`_.

The dynamic topic types offer the possibility to work over RTPS without the restrictions related to the IDLs.
Using them, the users can declare the different types that they need and manage the information directly,
avoiding the additional step of updating the IDL file and the generation of *C++* classes.

.. toctree::
   :maxdepth: 3

   /fastdds/dynamic_types/concepts.rst
   /fastdds/dynamic_types/supported_types.rst
   /fastdds/dynamic_types/complex_types.rst
   /fastdds/dynamic_types/annotations.rst
   /fastdds/dynamic_types/discovery.rst
   /fastdds/dynamic_types/serialization.rst
   /fastdds/dynamic_types/xml_profiles.rst
   /fastdds/dynamic_types/memory_management.rst
   /fastdds/dynamic_types/examples.rst
