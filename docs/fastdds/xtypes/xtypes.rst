.. include:: ../../03-exports/aliases-api.include

.. _dynamic-types:

XTypes
======

.. _DDS-XTypes V1.3: https://www.omg.org/spec/DDS-XTypes/1.3

*eProsima Fast DDS* manages :ref:`Discovery and Endpoint Matching <xtypes_discovery_matching>`
using |TypeInformation-api| and |TypeObject-api| when posible.
When remote endpoints are discovered, it checks that the types are known, initiating a type discovery process if
necessary before the matching between the endpoints takes place.
Alternatively, it uses a fallback mechanism based on type names when |TypeInformation-api| is unavailable.

*eProsima Fast DDS* provides a dynamic way to define and use topic types and topic data.
The :ref:`Dynamic Language Binding <xtypes_language_binding>` offers the possibility to work over DDS/RTPS
without the restrictions related to the IDLs.
The users can declare the different types that they need and manage the information on runtime,
avoiding the additional step of updating the IDL file and the generation of *C++* classes.

This implementation follows the *OMG Extensible and Dynamic Topic Types for DDS interface*.
For more information, refer to the specification for `DDS-XTypes V1.3`_.

.. toctree::
   :maxdepth: 3

   /fastdds/xtypes/discovery_matching.rst
   /fastdds/xtypes/language_binding.rst
