<a id="what-is-fast-dds"></a>

# What is *Fast DDS*?

[![eProsima](_static/eprosima-logo.svg)](http://www.eprosima.com/)

*eProsima Fast DDS* is a C++ implementation of the
[DDS (Data Distribution Service) Specification](https://www.omg.org/spec/DDS/About-DDS/), a protocol
defined by the [Object Management Group (OMG)](https://www.omg.org/).
The *eProsima Fast DDS* library provides both an Application Programming Interface (API) and a communication protocol
that deploy
a Data-Centric Publisher-Subscriber (DCPS) model, with the purpose of establishing efficient and reliable
information distribution among Real-Time Systems.

<a id="fast-dds-pro"></a>

# Fast DDS **[Pro]**

*Fast DDS Pro* is the commercial edition of *eProsima Fast DDS*, created for industrial and defense applications that
require advanced networking capabilities.

It extends the open-source version with premium features such as:

* [Low Bandwidth Transports](../fastdds/transport/low-bandwidth-transports.md#low-bandwidth-transports) **[Pro]** - optimized DDS communication for constrained or
  unstable links.
* [TSN Support](../fastdds/use_cases/tsn/tsn.md#use-case-dds-tsn) **[Pro]** - deterministic communication with guaranteed latency.
* [IP Mobility](../fastdds/use_cases/ip_mobility/ip_mobility.md#ip-mobility) **[Pro]** - uninterrupted communication even when network conditions change.
* [RPC over DDS](../fastdds/rpc_dds/rpc_dds_intro.md#rpc-dds-intro) **[Pro]** - seamless integration of remote procedure calls with DDS communication.
* [Congestion Control](../fastdds/use_cases/large_data/large_data.md#congestion-control) **[Pro]** - adaptive traffic management in congested environments.
* [Discovery Server topic interest propagation](../fastdds/discovery/discovery_server.md#ds-topic-interest) **[Pro]** - discovery information exchanged
  between *servers* limited to the topics actually in use.

The following table contains a brief comparison between *Fast DDS* and *Fast DDS Pro*:

<style>
  .md-table {
    width: 100%;
    border-collapse: collapse;
    font-family: sans-serif;
    font-size: 0.95em;
  }
  .md-table th, .md-table td {
    border: 1px solid var(--color-background-border, #dfe2e5);
    padding: 10px 16px;
    text-align: left;
    color: var(--color-foreground-primary, inherit);
    background-color: var(--color-background-primary, transparent);
  }
  .md-table thead tr {
    background-color: var(--color-background-secondary, #f6f8fa) !important;
    font-weight: bold;
    text-align: center;
  }
  .md-table tbody tr:nth-child(even) td {
    background-color: var(--color-background-secondary, #f6f8fa);
  }
  .md-table tbody tr:hover td {
    background-color: var(--color-background-hover, #eef2f5);
  }
  .md-table .section-header {
    background-color: var(--color-background-secondary, #e8ecef) !important;
    font-weight: bold;
    text-align: center;
    color: var(--color-foreground-secondary, #444);
    letter-spacing: 0.05em;
    text-transform: uppercase;
    font-size: 0.85em;
  }
</style>

<table class="md-table">
  <thead>
    <tr>
      <th style="width:20%"></th>
      <th style="width:40%; text-align:center; vertical-align:bottom;">
        <img src="pathto('_static/fast-dds-pro-logo-large.png', 1)" alt="Fast DDS Pro" width="300px"><br>
      </th>
      <th style="width:40%; text-align:center; vertical-align:bottom;">
        <img src="pathto('_static/fast-dds-logo-large.png', 1)" alt="Fast DDS (Community)" width="250px"><br>
      </th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <th>Target usage</th>
      <td>Production systems, industrial, robotics, defense</td>
      <td>Evaluation, prototyping, early development, research</td>
    </tr>
    <tr>
      <th>License</th>
      <td>Commercial</td>
      <td>Open Source (Apache 2.0)</td>
    </tr>
    <tr>
      <th>Support</th>
      <td>✅ Direct engineering support (unlimited)</td>
      <td>❌ Community-based</td>
    </tr>
    <tr>
      <th>Maintenance / LTS</th>
      <td>✅ Long-term support (~24 months, backports)</td>
      <td>❌ No guaranteed maintenance</td>
    </tr>
    <tr>
      <th>Predictability</th>
      <td>✅ Controlled releases &amp; fixes</td>
      <td>⚠️ Best-effort evolution</td>
    </tr>
    <tr>
      <th>DDS Pub/Sub &amp; QoS</th>
      <td>✅ Full</td>
      <td>✅ Full</td>
    </tr>
    <tr>
      <th>TSN over DDS</th>
      <td>✅</td>
      <td>❌</td>
    </tr>
    <tr>
      <th>RPC over DDS</th>
      <td>✅ (incl. streaming)</td>
      <td>❌</td>
    </tr>
    <tr>
      <th>IP Mobility</th>
      <td>✅</td>
      <td>❌</td>
    </tr>
    <tr>
      <th>Low Bandwidth Transports</th>
      <td>✅</td>
      <td>❌</td>
    </tr>
    <tr>
      <th>Adaptive Congestion Control</th>
      <td>✅</td>
      <td>❌</td>
    </tr>
    <tr>
      <th>Discovery Server</th>
      <td>✅ Advanced / optimized matching</td>
      <td>⚠️ Standard functionality</td>
    </tr>
    <tr>
      <th>Discovery Server topic interest propagation</th>
      <td>✅ Servers exchange only the discovery data of the topics in use</td>
      <td>❌ Servers exchange their whole discovery database</td>
    </tr>
  </tbody>
</table>

To discover all features and request a commercial license please <a href="https://forms.eprosima.com/reach/form/CommercialSupportRequest/formperma/Ac8GwewD7PTDadQZIV92qDEzNFfMlJnYmA029mSJtJ8" target="_blank" rel="noopener noreferrer">share your contact details</a>.

<a id="commercial-support"></a>

# Commercial support

Looking for commercial support? Please <a href="https://forms.eprosima.com/reach/form/CommercialSupportRequest/formperma/Ac8GwewD7PTDadQZIV92qDEzNFfMlJnYmA029mSJtJ8" target="_blank" rel="noopener noreferrer">share your details</a> so we can assist you.

Find more about us at <a href="https://www.eprosima.com" target="_blank" rel="noopener noreferrer">eProsima’s webpage</a>.

<a id="key-features"></a>

# Key features

*eProsima Fast DDS* is predictable, scalable, flexible, and efficient in resource handling.
For meeting these requirements, it makes use of typed interfaces and hinges on a many-to-many
distributed network paradigm that neatly allows separation of the publisher and subscriber sides of the communication.
*eProsima Fast DDS* comprises:

* The [DDS API](../fastdds/dds_layer/dds_layer.md#dds-layer) implementation.
* [Fast DDS-Gen](../fastddsgen/introduction/introduction.md#fastddsgen-intro), a generation tool for bridging typed interfaces with the middleware
  implementation.
* The underlying [RTPS](../fastdds/rtps_layer/rtps_layer.md#rtps-layer) wire protocol implementation.

For all the above, *eProsima Fast DDS* has been chosen as the default middleware supported by the
[Robot Operating System 2 (ROS 2)](https://index.ros.org/doc/ros2/) in every long term (LTS) releases and most of the
non-LTS releases.

<a id="dds-api"></a>

# DDS API

The communication model adopted by DDS is a many-to-many unidirectional data exchange where the applications that
produce the data publish it to the local caches of subscribers belonging to applications that consume the data.
The information flow is regulated by Quality of Service (QoS) policies established between the entities in
charge of the data exchange.

As a data-centric model, DDS builds on the concept of a “global data space” accessible to all interested applications.
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

![image](01-figures/DDS_concept.svg)

*Conceptual diagram of how information flows within DDS domains.
Only entities belonging to the same domain can discover each
other through matching topics, and consequently exchange data between publishers and subscribers.*

<a id="fast-dds-gen"></a>

# Fast DDS-Gen

Relying on interfaces implies the need for a generation tool that translates type descriptions into appropriate
implementations that fill the gap between the interfaces and the middleware.
This task is carried out by a dedicated generation tool, [Fast DDS-Gen](../fastddsgen/introduction/introduction.md#fastddsgen-intro), a Java application
that generates source code using the data types defined in an
[Interface Definition Language (IDL)](https://www.omg.org/spec/IDL/About-IDL/) file.

<a id="rtps-wire-protocol"></a>

# RTPS Wire Protocol

The protocol used by *eProsima Fast DDS* to exchange messages over standard networks is the [Real-Time
Publish-Subscribe protocol (RTPS)](https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/), an interoperability wire
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

<a id="main-features"></a>

# Main Features

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
* [Statistics Module.](../fastdds/statistics/statistics.md#statistics) *eProsima Fast DDS* can be configured to gather and provide information
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
  [Fast CDR](https://github.com/eProsima/Fast-CDR),
  a C++ library that serializes according to the standard CDR serialization mechanism defined in the [RTPS
  Specification](https://www.omg.org/spec/DDSI-RTPS/) (see the Data Encapsulation chapter as a reference).
* **Easy to use.** The project comes with an out-of-the-box example, the *DDSHelloWorld*
  (see [Getting Started](../fastdds/getting_started/getting_started.md#getting-started)) that puts into communication a
  publisher and a subscriber, showcasing how *eProsima Fast DDS* is deployed.
  Additionally, the interactive demo *ShapesDemo* is available for the user to dive into the DDS world.
  The DDS and the RTPS layers are thoroughly explained in the [DDS Layer](../fastdds/dds_layer/dds_layer.md#dds-layer) and
  [RTPS Layer](../fastdds/rtps_layer/rtps_layer.md#rtps-layer) sections.
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

<a id="dependencies-and-compatibilities"></a>

# Dependencies and compatibilities

*eProsima Fast DDS* is continuously evolving and improving.
This means that the different software products that are part of the Fast DDS ecosystem are evolving and improving
together with Fast DDS.

Fast DDS has some [library dependencies](../notes/versions.md#dependencies-compatibilities-library-dependencies), e.g. the previously
mentioned Fast CDR for data serialization, or OpenSSL for secure communications.
Depending on different [platform support levels](../notes/versions.md#dependencies-compatibilities-platform-support), it has also
different [build dependencies](../notes/versions.md#dependencies-compatibilities-build-system-dependencies).

Finally, there are some other *eProsima* products that use Fast DDS as a middleware, such as *Micro XRCE-DDS*,
*DDS Router* and *Fast DDS python* wrapper.
Those that are strongly attached to each Fast DDS supported version are described in [this product compatibility
table](../notes/versions.md#dependencies-compatibilities-product-compatibility).

<a id="contributing-to-the-documentation"></a>

# Contributing to the documentation

*Fast DDS-Docs* is an open source project, and as such all contributions, both in the form of feedback and content
generation, are most welcomed.
To make such contributions, please refer to the
[Contribution Guidelines](https://github.com/eProsima/all-docs/blob/master/CONTRIBUTING.md) hosted in our GitHub repository.

<a id="structure-of-the-documentation"></a>

# Structure of the documentation

This documentation is organized into the sections below.

* [Installation Manual](../installation/binaries/binaries_linux.md#linux-binaries)
* [Fast DDS](../fastdds/getting_started/getting_started.md#getting-started)
* [Fast DDS-Gen](../fastddsgen/introduction/introduction.md#fastddsgen-intro)
* [Release Notes](../notes/notes.md#release-notes)

The documentation includes a [Frequently Asked Questions (FAQ)](../fastdds/faq/faq.md#frequently-asked-questions) section that can be
consulted for a quick overview.
