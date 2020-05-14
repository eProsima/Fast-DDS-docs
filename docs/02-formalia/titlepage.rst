.. image:: /01-figures/logo.png
   :height: 100px
   :width: 100px
   :align: left
   :alt: eProsima
   :target: http://www.eprosima.com/

*eprosima Fast DDS* is a C++ implementation of the DDS (Data Distribution Service) Specification, a protocol
defined by the Object Management Group (OMG).
The *eprosima Fast DDS* library provides both an Application Interface (API) and a communication protocol that deploy
a Data-Centric Publisher-Subscriber (DCPS) model, with the purpose of establishing efficient and reliable
information distribution among Real-Time Systems.
*eprosima Fast DDS* is predictable, scalable, flexible, and efficient in resource handling.
For meeting these requirements, it makes use of typed interfaces and hinges on a many-to-many
distributed network paradigm that neatly allows separation of the publisher and subscriber sides of the communication.

*eprosima Fast DDS* comprises:

* The DDS API implementation.
* A generation tool for bridging typed interfaces with the middleware implementation.
* The underlying wire protocol.

DDS API
^^^^^^^

The communication model adopted by DDS is one of a many-to-many unidirectional data exchange where the applications that
produce the data publish them to the local caches of subscribers belonging to applications that consume the data.
The information flow is regulated by Quality of Service (QoS) policies established between the entities in
charge of the data exchange.

As a data-centric model, DDS builds on the concept of a "global data space" accessible to all interested applications.
Applications that want to contribute information declare their intent to become publishers whereas applications that
want to access portions of the data space declare their intent to become subscribers.
Each time a publisher posts new data into this space, the middleware propagates the information to all
interested subscribers.

The communication happens across domains, isolated abstract planes that link all the distributed applications able to
communicate with each other.
Only entities belonging to a same domain can interact, and the matching between entities subscribing to data and
entities publishing them is mediated by topics. Topics are unambiguous identifiers that associate a
name, that is unique in the domain, to a data type and a set of attached data-specific QoS.

DDS entities are modeled either as classes or typed interfaces.
The latter imply a more efficient resource handling as knowledge of the data
type prior to the execution allows allocating memory in advance rather than dynamically.

FastRTPSGen
^^^^^^^^^^^^

Relying on interfaces implies the need for a generation tool that translates type descriptions into appropriate
implementations that fill the gap between the interfaces and the middleware.
This task is carried out by a dedicated generation tool, *FastRTPSGen*, a Java application that generates source code
using the data types defined in an IDL file.

RTPS Wire Protocol
^^^^^^^^^^^^^^^^^^

The protocol used by *eProsima Fast DDS* to exchange messages over standard networks is the Real-Time
Publish-Subscribe protocol (RTPS), an interoperability wire protocol for DDS defined and maintained by the OMG
consortium.
This protocol provides publisher-subscriber communications over transports such as TCP/UDP/IP, and guarantees
compatibility among different DDS implementations.

Given its publish-subscribe roots and its specification designed for meeting the same requirements addressed by the DDS
application domain, the RTPS protocol is a natural choice for DDS implementations and maps naturally to many DDS
concepts.
All the RTPS core entities are associated with an RTPS domain, which represents an isolated communication plane where
endpoints match.
The entities specified in the RTPS protocol are in one-to-one correspondence with the DDS entities, thus allowing
the communication to occur.

Main Features
^^^^^^^^^^^^^

**High performance.**

*eProsima Fast DDS* uses a static low-level serialization library (*eProsima Fast CDR*) that serializes in *CDR*.

**Low resources consumption.**

*eProsima Fast DDS*:

* Allows to pre-allocate resources, to minimize dynamic resource allocation.
* Avoids the use of unbounded resources.
* Minimizes the need to copy data.

**Multi-platform.**

The OS dependencies are treated as pluggable modules.
The user can easily implement his platform modules to *eProsima Fast DDS* library in his specific platform.
By default, the project can run over Linux and Windows.

**Free and Open Source.**

The Fast DDS library, the underneath RTPS library, the generator tool, the internal dependencies (such as *eProsima
Fast CDR*) and the external ones (such as the *foonathan* library) are free and open source.

**Easy to use.**

The project comes with an out-of-the-box example, the *DDSHelloWorld* (TODO: add ref) that puts into communication a
publisher and a subscriber, showcasing how *eProsima Fast DDS* is deployed.
Additionally, the interactive demo *ShapesDemo* is available for the user to dive into the DDS world.
Both the DDS and the RTPS layers are thoroughly explained in the Fast DDS section (TODO: add ref).

**Best effort and reliable communication.**

*eProsima Fast-DDS* supports both *best effort*, for fast and light communication, and *reliable*, when reliability
is needed.

**Built-in Discovery Service.**

*eProsima Fast DDS* is based on the dynamical discover of the existence of publishers and subscribers
and performs this task continuously without
the need to contact any servers.

**Plug-and-play Connectivity**

New applications and services are automatically discovered, and can join and leave the network at any time without the
need for reconfiguration.

**Scalability and Flexibility**

DDS builds on the concept of a global data space. The middleware is in charge of propagating the information between
publishers and subscribers. This guarantees that the distributed network is adaptable to reconfigurations
and scalable to a large number of entities.

**Application Portability**

The DDS specification includes a platform specific mapping to IDL, allowing an application using
DDS to switch among DDS implementations with only a re-compile.

**Extensibility**

*eProsima Fast DDS* allows the protocol to be extended and enhanced with new services without
breaking backwards compatibility and interoperability.

**Configurability and Modularity**

*eProsima Fast DDS* provides an intuitive way to be configured, either through code or XML profiles.
Modularity allows simple devices to implement a subset of the protocol and still participate in
the network.

**Two API Layers**

*eProsima Fast DDS* comprises a high-level publisher-subscriber layer focused on usability and a lower-level 
writer-reader layer that provides finer access to the RTPS protocol.

**Commercial support**

Available at support@eprosima.com


Structure of the Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This documentation is organized into the following sections:

Installation Manual
-------------------

This section is meant to provide the user with an easy-to-use installation guide.


Fast DDS
--------

This section provides the rationale and conceptual outline of the DDS API, and an in-depth explanation of the
*eProsima Fast DDS* library and modules breakdown.


FastRTPSGen Manual
------------------

This section addresses the use of the serialization/deserialization tool *FastRTPSGen*, whose aim is to generate source
code that can be used by applications to publish and subscribe to custom topics of arbitrary data types.


Release Notes
^^^^^^^^^^^^^

Notes on the present and previous release versions can be found in this section.

* :ref:`index_installation`
* :ref:`index_fast_dds`
* :ref:`index_gen`
* :ref:`index_notes`
