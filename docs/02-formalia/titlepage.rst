.. image:: /01-figures/logo.png
   :height: 100px
   :width: 100px
   :align: left
   :alt: eProsima
   :target: http://www.eprosima.com/

*eprosima Fast DDS* is a C++ implementation of the
`DDS (Data Distribution Service) Specification <https://www.omg.org/spec/DDS/About-DDS/>`__, a protocol
defined by the Object Management Group (OMG).
The *eprosima Fast DDS* library provides both an Application Programming Interface (API) and a communication protocol
that deploy
a Data-Centric Publisher-Subscriber (DCPS) model, with the purpose of establishing efficient and reliable
information distribution among Real-Time Systems.
*eprosima Fast DDS* is predictable, scalable, flexible, and efficient in resource handling.
For meeting these requirements, it makes use of typed interfaces and hinges on a many-to-many
distributed network paradigm that neatly allows separation of the publisher and subscriber sides of the communication.

*eprosima Fast DDS* comprises:

* The DDS API implementation.
* A generation tool for bridging typed interfaces with the middleware implementation.
* The underlying wire protocol implementation.

.. figure:: /01-figures/DDS_concept_vertical.svg

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

FastRTPSGen
^^^^^^^^^^^^

Relying on interfaces implies the need for a generation tool that translates type descriptions into appropriate
implementations that fill the gap between the interfaces and the middleware.
This task is carried out by a dedicated generation tool, *FastRTPSGen*, a Java application that generates source code
using the data types defined in an `Interactive Data Language (IDL) <https://www.omg.org/spec/IDL/About-IDL/>`__ file.

RTPS Wire Protocol
^^^^^^^^^^^^^^^^^^

The protocol used by *eProsima Fast DDS* to exchange messages over standard networks is the `Real-Time
Publish-Subscribe protocol (RTPS) <https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/>`__, an interoperability wire
protocol for DDS defined and maintained by the OMG
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

* **High performance.** *eProsima Fast DDS* uses a static low-level serialization library,
  `Fast CDR <https://github.com/eProsima/Fast-CDR>`,
  a C++ library that serializes either according to the standard CDR serialization mechanism (defined in the `RTPS
  Specification <https://www.omg.org/spec/DDSI-RTPS/>`__, see Data Encapsulation for cfr).

* **Low resources consumption.** *eProsima Fast DDS*:

  * Allows to pre-allocate resources, to minimize dynamic resource allocation.
  * Avoids the use of unbounded resources.
  * Minimizes the need to copy data.

* **Multi-platform.** The OS dependencies are treated as pluggable modules.
  The user can easily implement his platform modules to *eProsima Fast DDS* library in his specific platform.
  By default, the project can run over Linux, Windows and MacOS.

* **Free and Open Source.** The Fast DDS library, the underneath RTPS library, the generator tool, the internal
  dependencies (such as *eProsima
  Fast CDR*) and the external ones (such as the *foonathan* library) are free and open source.

* **Easy to use.** The project comes with an out-of-the-box example, the *DDSHelloWorld*
  (see :ref:`Getting Started <getting started>`) that puts into communication a
  publisher and a subscriber, showcasing how *eProsima Fast DDS* is deployed.
  Additionally, the interactive demo *ShapesDemo* is available for the user to dive into the DDS world.
  Both the DDS and the RTPS layers are thoroughly explained in the :ref:`Fast DDS <Fast DDS>` section.

* **Best effort and reliable communication.** *eProsima Fast DDS* supports an optional reliable communication paradigm
  over *Best Effort* communications protocols
  such as UDP. Furthermore, another way of setting a reliable communication is to use our TCP transport.

* **Built-in Discovery Service.** *eProsima Fast DDS* is based on the dynamical discovery of existing publishers and
  subscribers, and performs this task continuously without the need to contacting or setting any servers.
  However, a Client-Server discovery as well as other discovery paradigms can also be configured. (link)

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

* **Two API Layers.** *eProsima Fast DDS* comprises a high-level publisher-subscriber layer focused on usability and a
  lower-level RTPS layer that provides finer access to the RTPS protocol.

* **Security.** *eProsima Fast DDS* can be configured to provide secure communications. For this purpose, it implements
  pluggable security at three levels: authentication of remote participants, access control of entities and encryption
  of data.

* **Transport layers.** *eProsima Fast DDS* implements an architecture of pluggable transports. The current version
  implements five transports: UDPv4, UDPv6, TCPv4, TCPv6 and SHM (shared memory).

* **Real-Time behaviour.** *eProsima Fast DDS* can be configured to offer real-time features, guaranteeing responses
  within specified time constrains.

* **Sync and Async publication modes.** *eProsima Fast DDS* supports both synchronous and asynchronous data access.

* **Throughput controllers.** We support user-configurable throughput controllers, that can be used to limit the amount
  of data to be sent under certain conditions.

* **Default ROS 2 Middleware.** *eProsima Fast DDS* is the default middleware supported by the `Robot Operating System 2
  (ROS 2) <https://index.ros.org/doc/ros2/>`__.

Contacts and Commercial support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Find more about us at `*eProsima*'s webpage <https://eprosima.com/>`__.

Support available at:

* Email: support@eprosima.com
* Phone: +34 91 804 34 48 


This documentation is organized into the sections below.

* :ref:`index_installation`
* :ref:`index_fast_dds`
* :ref:`index_gen`
* :ref:`index_notes`
