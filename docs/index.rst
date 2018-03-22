.. eProsima Fast RTPS documentation master file.

eProsima Fast RTPS Documentation
=======================================

.. image:: logo.png
   :height: 100px
   :width: 100px
   :align: left
   :alt: eProsima
   :target: http://www.eprosima.com/

*eprosima Fast RTPS* is a C++ implementation of the RTPS (Real Time Publish Subscribe) protocol, which provides publisher-subscriber communications over unreliable transports such as UDP, 
as defined and maintained by the Object Management Group (OMG) consortium. RTPS is also the wire interoperability protocol defined for the Data Distribution
Service (DDS) standard, again by the OMG. *eProsima Fast RTPS* holds the benefit of being standalone and up-to-date, as most vendor solutions either implement RTPS as a tool to implement 
DDS or use past versions of the specification.

Some of the main features of this library are:

* Configurable best-effort and reliable publish-subscribe communication policies for real-time applications.
* Plug and play connectivity so that any new applications are automatically discovered by any other members of the network.
* Modularity and scalability to allow continuous growth with complex and simple devices in the network.
* Configurable network behavior and interchangeable transport layer: Choose the best protocol and system input/output channel combination for each deployment.
* Two API Layers: a high-level Publisher-Subscriber one focused on usability and a lower-level Writer-Reader one that provides finer access to the inner workings of the RTPS protocol.

eProsima Fast RTPS has been adopted by multiple organizations in many sectors including these important cases:

* Robotics: ROS (Robotic Operating System) as their default middleware for ROS2.
* EU R&D: FIWARE Incubated GE.

This documentation is organized into the following sections:

* :ref:`installation`
* :ref:`user`
* :ref:`gen`
* :ref:`notes`

.. _installation:

.. toctree::
   :caption: Installation manual

   requirements
   binaries
   sources


.. _user:

.. toctree::
   :caption: User Manual

   introduction
   overview
   list
   pubsub
   rtps
   advanced
   security
   persistence
   fastrtpsgen


.. _gen:

.. toctree::
   :caption: FastRTPSGen Manual

   geninfo
   genuse


.. _notes:

.. toctree::
   :caption: Release Notes

   notes

