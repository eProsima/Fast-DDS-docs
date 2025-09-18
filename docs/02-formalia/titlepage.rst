
What is *Fast DDS*?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. image:: /_static/eprosima-logo.svg
  :height: 100px
  :width: 100px
  :align: left
  :alt: eProsima
  :target: http://www.eprosima.com/

*eProsima Fast DDS* is a C++ implementation of the
`DDS (Data Distribution Service) Specification <https://www.omg.org/spec/DDS/About-DDS/>`__, a protocol
defined by the `Object Management Group (OMG) <https://www.omg.org/>`__.
The *eProsima Fast DDS* library provides both an Application Programming Interface (API) and a communication protocol
that deploy
a Data-Centric Publisher-Subscriber (DCPS) model, with the purpose of establishing efficient and reliable
information distribution among Real-Time Systems.

.. raw:: html

    <br/>

Commercial support
^^^^^^^^^^^^^^^^^^

.. |commercial_support_link| raw:: html

   <a href="https://forms.eprosima.com/reach/form/CommercialSupportRequest/formperma/Ac8GwewD7PTDadQZIV92qDEzNFfMlJnYmA029mSJtJ8" target="_blank" rel="noopener noreferrer">share your details</a>

Looking for commercial support? Please |commercial_support_link| so we can assist you.

.. |eprosima_webpage_link| raw:: html

   <a href="https://www.eprosima.com" target="_blank" rel="noopener noreferrer">eProsima’s webpage</a>

Find more about us at |eprosima_webpage_link|.

Key features
^^^^^^^^^^^^

*eProsima Fast DDS* is predictable, scalable, flexible, and efficient in resource handling.
For meeting these requirements, it makes use of typed interfaces and hinges on a many-to-many
distributed network paradigm that neatly allows separation of the publisher and subscriber sides of the communication.
*eProsima Fast DDS* comprises:

* The :ref:`DDS API <dds_layer>` implementation.
* :ref:`Fast DDS-Gen <fastddsgen_intro>`, a generation tool for bridging typed interfaces with the middleware
  implementation.
* The underlying :ref:`RTPS <rtps_layer>` wire protocol implementation.

For all the above, *eProsima Fast DDS* has been chosen as the default middleware supported by the
`Robot Operating System 2 (ROS 2) <https://index.ros.org/doc/ros2/>`__ in every long term (LTS) releases and most of the
non-LTS releases.

DDS API
^^^^^^^

The communication model adopted by DDS is a many-to-many unidirectional data exchange where the applications that
produce the data publish it to the local caches of subscribers belonging to applications that consume the data.
The information flow is regulated by Quality of Service (QoS) policies established between the entities in
charge of the data exchange.

As a data-centric model, DDS builds on the concept of a "global data space" accessible to all interested applications.
Applications that want to contribute information declare their intent to become publishers, whereas applications that
want to access portions of the data space declare their intent to become subscribers.
Each time a publisher posts new data into this space, the middleware propagates the information to all
interested subscribers.

The communication happens across domains, i. e. isolated abstract planes that link all the distributed applications
able to communicate with each other.
Only entities belonging to a same domain can interact, and the matching between entities subscribing to data and
entities publishing them is mediated by topics. Topics are unambiguous identifiers that associate a
name, which is unique in the domain, to a data type and a set of attached data-specific QoS.

DDS entities are modeled either as classes or typed interfaces.
The latter imply a more efficient resource handling as knowledge of the data
type prior to the execution allows allocating memory in advance rather than dynamically.


.. figure:: /01-figures/DDS_concept.svg
    :align: center

    Conceptual diagram of how information flows within DDS domains.
    Only entities belonging to the same domain can discover each
    other through matching topics, and consequently exchange data between publishers and subscribers.

Fast DDS-Gen
^^^^^^^^^^^^

Relying on interfaces implies the need for a generation tool that translates type descriptions into appropriate
implementations that fill the gap between the interfaces and the middleware.
This task is carried out by a dedicated generation tool, :ref:`Fast DDS-Gen <fastddsgen_intro>`, a Java application
that generates source code using the data types defined in an
`Interface Definition Language (IDL) <https://www.omg.org/spec/IDL/About-IDL/>`__ file.

RTPS Wire Protocol
^^^^^^^^^^^^^^^^^^

The protocol used by *eProsima Fast DDS* to exchange messages over standard networks is the `Real-Time
Publish-Subscribe protocol (RTPS) <https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/>`__, an interoperability wire
protocol for DDS defined and maintained by the OMG
consortium.
This protocol provides publisher-subscriber communications over transports such as TCP/UDP/IP, and guarantees
compatibility among different DDS implementations.

Given its publish-subscribe roots and its specification designed for meeting the same requirements addressed by the DDS
application domain, the RTPS protocol maps to many DDS concepts and is therefore a natural choice for DDS
implementations.
All the RTPS core entities are associated with an RTPS domain, which represents an isolated communication plane where
endpoints match.
The entities specified in the RTPS protocol are in one-to-one correspondence with the DDS entities, thus allowing
the communication to occur.

Main Features
^^^^^^^^^^^^^

* **Two API Layers.** *eProsima Fast DDS* comprises a high-level DDS compliant layer focused on usability and a
  lower-level RTPS compliant layer that provides finer access to the RTPS protocol.

* **Real-Time behaviour.** *eProsima Fast DDS* can be configured to offer real-time features, guaranteeing responses
  within specified time constrains.

* **Built-in Discovery Server.** *eProsima Fast DDS* is based on the dynamical discovery of existing publishers and
  subscribers, and performs this task continuously without the need to contacting or setting any servers.
  However, a Client-Server discovery as well as other discovery paradigms can also be configured.

* **Sync and Async publication modes.** *eProsima Fast DDS* supports both synchronous and asynchronous data publication.

* **Best effort and reliable communication.** *eProsima Fast DDS* supports an optional reliable communication paradigm
  over *Best Effort* communications protocols
  such as UDP. Furthermore, another way of setting a reliable communication is to use our TCP transport.

* **Transport layers.** *eProsima Fast DDS* implements an architecture of pluggable transports. The current version
  implements five transports: UDPv4, UDPv6, TCPv4, TCPv6 and SHM (shared memory).

* **Security.** *eProsima Fast DDS* can be configured to provide secure communications. For this purpose, it implements
  pluggable security at three levels: authentication of remote participants, access control of entities and encryption
  of data.

* :ref:`Statistics Module. <statistics>` *eProsima Fast DDS* can be configured to gather and provide information
  about the data being exchanged by the user application.

* **Flow controllers.** We support user-configurable flow controllers, that can be used to limit the amount
  of data to be sent under certain conditions.

* **Plug-and-play Connectivity.** New applications and services are automatically discovered, and can join and leave
  the network at any time without the
  need for reconfiguration.

* **Scalability and Flexibility.** DDS builds on the concept of a global data space. The middleware is in charge of
  propagating the information between publishers and subscribers. This guarantees that the distributed network is
  adaptable to reconfigurations and scalable to a large number of entities.

* **Application Portability.** The DDS specification includes a platform specific mapping to IDL, allowing an
  application using DDS to switch among DDS implementations with only a re-compile.

* **Extensibility.** *eProsima Fast DDS* allows the protocol to be extended and enhanced with new services without
  breaking backwards compatibility and interoperability.

* **Configurability and Modularity.** *eProsima Fast DDS* provides an intuitive way to be configured, either through
  code or XML profiles. Modularity allows simple devices to implement a subset of the protocol and still participate in
  the network.

* **High performance.** *eProsima Fast DDS* uses a static low-level serialization library,
  `Fast CDR <https://github.com/eProsima/Fast-CDR>`__,
  a C++ library that serializes according to the standard CDR serialization mechanism defined in the `RTPS
  Specification <https://www.omg.org/spec/DDSI-RTPS/>`__ (see the Data Encapsulation chapter as a reference).

* **Easy to use.** The project comes with an out-of-the-box example, the *DDSHelloWorld*
  (see :ref:`getting_started`) that puts into communication a
  publisher and a subscriber, showcasing how *eProsima Fast DDS* is deployed.
  Additionally, the interactive demo *ShapesDemo* is available for the user to dive into the DDS world.
  The DDS and the RTPS layers are thoroughly explained in the :ref:`DDS Layer <dds_layer>` and
  :ref:`RTPS Layer <rtps_layer>` sections.

* **Low resources consumption.** *eProsima Fast DDS*:

  * Allows to preallocate resources, to minimize dynamic resource allocation.
  * Avoids the use of unbounded resources.
  * Minimizes the need to copy data.

* **Multi-platform.** The OS dependencies are treated as pluggable modules.
  Users may easily implement platform modules using the *eProsima Fast DDS* library on their target platforms.
  By default, the project can run over Linux, Windows and MacOS.

* **Free and Open Source.** The Fast DDS library, the underneath RTPS library, the generator tool, the internal
  dependencies (such as *eProsima Fast CDR*) and the external ones (such as the *foonathan* library) are free and
  open source.

Dependencies and compatibilities
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*eProsima Fast DDS* is continuously evolving and improving.
This means that the different software products that are part of the Fast DDS ecosystem are evolving and improving
together with Fast DDS.

Fast DDS has some :ref:`library dependencies <dependencies_compatibilities_library_dependencies>`, e.g. the previously
mentioned Fast CDR for data serialization, or OpenSSL for secure communications.
Depending on different :ref:`platform support levels <dependencies_compatibilities_platform_support>`, it has also
different :ref:`build dependencies <dependencies_compatibilities_build_system_dependencies>`.

Finally, there are some other *eProsima* products that use Fast DDS as a middleware, such as *Micro XRCE-DDS*,
*DDS Router* and *Fast DDS python* wrapper.
Those that are strongly attached to each Fast DDS supported version are described in :ref:`this product compatibility
table<dependencies_compatibilities_product_compatibility>`.

Contributing to the documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Fast DDS-Docs* is an open source project, and as such all contributions, both in the form of feedback and content
generation, are most welcomed.
To make such contributions, please refer to the
`Contribution Guidelines <https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md>`_ hosted in our GitHub repository.

Structure of the documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This documentation is organized into the sections below.

* :ref:`Installation Manual <linux_binaries>`
* :ref:`Fast DDS <getting_started>`
* :ref:`Fast DDS-Gen <fastddsgen_intro>`
* :ref:`Release Notes <release_notes>`

The documentation includes a :ref:`Frequently Asked Questions (FAQ) <frequently_asked_questions>` section that can be
consulted for a quick overview.
