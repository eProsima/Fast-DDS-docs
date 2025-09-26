.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _use-case-dds-tsn:

DDS-TSN |Pro|
=============

.. _tsn_intro:

Time-Sensitive Networking (TSN)
-------------------------------

Time-Sensitive Networking (TSN) is a set of IEEE 802.1 standards that provide deterministic data delivery,
low latency, and high reliability over Ethernet networks.
TSN enables real-time communication for critical applications by ensuring that time-critical data is delivered
within strict timing constraints, even in the presence of non-critical traffic on the same network.

.. _tsn_standards:

OMG DDS-TSN Specification
-------------------------

The Object Management Group (OMG) created the
`DDS-TSN specification <https://www.omg.org/spec/DDS-TSN/1.0/Beta1/PDF/>`_ to define standard mappings between
the Data Distribution Service (DDS) and Time-Sensitive Networking (TSN).
This integration allows DDS applications to leverage TSN capabilities to achieve deterministic, real-time
communication while maintaining the data-centric approach of DDS.

The specification defines:

* Mapping of DDS QoS policies to TSN capabilities
* Support for DDSI-RTPS over TSN, both for `UDP/IP` and raw `Ethernet` transports
* Flow configuration based on transport identifiers:
    * `UDPv4` / `UDPv6`: `IPv4Tuple` / `IPv6Tuple`
    * `Ethernet`: `IEEE802MacAddresses` & `IEEE802VlanTag`

.. _tsn_fastdds:

Fast DDS TSN Implementation
---------------------------

*Fast DDS Pro* provides the required scaffolding for using DDS over TSN compliant with the OMG specification.

TSN flow configuration can be set via a mapping between a defined
:ref:`TransportPriorityQosPolicy <transportpriorityqospolicy>` value and a set of tuple parameters.
Then, any :ref:`DataWriter<dds_layer_publisher_dataWriter>` which has been assigned such mapped
:ref:`TransportPriorityQosPolicy <transportpriorityqospolicy>` will use the corresponding tuple parameters for
its communication.

Note that a :ref:`DataWriter<dds_layer_publisher_dataWriter>` can have its :ref:`TransportPriorityQosPolicy
<transportpriorityqospolicy>` updated at any time, and the new tuple parameters will be applied to subsequent
sends.

Supported transports
^^^^^^^^^^^^^^^^^^^^

*Fast DDS Pro* currently supports two transports that can be used over TSN-capable networks:

- **UDP Transport Support (UDPv4 / UDPv6)**

  *Fast DDS* supports DDSI-RTPS communication over standard POSIX `UDPv4` and `UDPv6` sockets.
  *Fast DDS Pro* extends these transports to enable seamless operation on TSN-capable networks
  while preserving compatibility with existing DDS deployments.

  In this transport, the tuple parameters (`IPv4Tuple` / `IPv6Tuple`) are:

  * **Source IP address / port**
  * **Destination IP address / port**
  * **DSCP**
  * **Protocol**

  The `protocol` field is fixed to `UDPv4` / `UDPv6` by the selected
  :ref:`UDP Transport <transport_udp_transportDescriptor>`.
  The rest of the fields can be set like in the following example:

  .. tab-set-code::

    .. literalinclude:: /../code/ProDDSCodeTester.cpp
       :language: c++
       :start-after: //TSN_SET_UDP_TUPLE
       :end-before: //!--
       :dedent: 8

    .. literalinclude:: /../code/ProXMLTester.xml
       :language: xml
       :start-after: <!-->TSN_SET_UDP_TUPLE<-->
       :end-before: <!--><-->

  .. note::

    On Windows systems, modifying the `DSCP` value requires additional operating system changes.
    It is not sufficient to configure the QoS profile alone.
    You must also update the Windows registry and group policies to allow applications to set `DSCP` values:

    - In the `regedit` application, under ``HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\Tcpip\Parameters``,
      create a new `DWORD` value named `DisableUserTOSSetting`, and set it to `0`.
    - Under ``HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Services\Tcpip\QoS`` (create the ``QoS`` key if it does
      not exist), create a new string value named  ``Do not use NLA`` and set it to ``1``.
    - Reboot the system and open ``gpedit.msc``.
    - Create a new policy under *Computer Configuration → Windows Settings → Policy-based QoS*.
      Select *Specify DSCP Value* and enter the desired value.
      On the next page, select `Only applications with this executable name`, and specify the full path to your
      application’s executable file.
      Then click "Next" until the configuration is complete.

    After completing these steps, your application will be able to use the configured `DSCP` value.

- **Ethernet Transport Support**

  *Fast DDS Pro* includes a custom `Ethernet` transport that operates directly at Layer 2 (data link layer),
  bypassing the TCP/IP stack for reduced latency and direct control over ethernet frames.

  *Fast DDS* extends the standard `Locator` class with a new kind for ethernet communication,
  ``LOCATOR_KIND_ETHERNET: 0x02000000``.

  In this transport, the tuple parameters (`IEEE802MacAddresses` & `IEEE802VlanTag`) are:

  * **Source MAC address**

    Optionally, a virtual sending logical port may be defined to allow multiple DataWriters to share a MAC
    address while still offering a useful way to identify the source of the traffic.
  * **Destination MAC address**
  * **PCP**
  * **VLAN ID**

  Additionally, the Ethernet transport requires setting a default source port for non-prioritized traffic
  (i.e. traffic from DataWriters with :ref:`TransportPriorityQosPolicy <transportpriorityqospolicy>` set to 0).
  This traffic will have ``PCP=0`` and ``VLAN ID=0`` by default.

  All fields can be set like in the following example:

  .. tab-set-code::

    .. literalinclude:: /../code/ProDDSCodeTester.cpp
       :language: c++
       :start-after: //TSN_SET_ETHERNET_TUPLE
       :end-before: //!--
       :dedent: 8

    .. literalinclude:: /../code/ProXMLTester.xml
       :language: xml
       :start-after: <!-->TSN_SET_ETHERNET_TUPLE<-->
       :end-before: <!--><-->

  .. note::

    This transport is only supported on Linux systems.
    It requires either root privileges or the ``CAP_NET_RAW`` capability.

  .. note::

    The *Fast DDS Pro* installation provides a helper file ``rtps_eth.lua`` under the ``share/fastdds``
    subfolder.
    This script allows to easily dissect packets from this transport in Wireshark.
    For details on installing custom plugins, see the
    `Wireshark documentation <https://www.wireshark.org/docs/wsug_html_chunked/ChPluginFolders.html>`_.

TSN Compatible QoS Policies
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The DDS-TSN specification recommends using the following QoS configuration in the topics that need to leverage
TSN capabilities:

* **Reliability**

  Must use `BEST_EFFORT_RELIABILITY_QOS`
* **Durability**

  Must use `VOLATILE_DURABILITY_QOS`
* **History**

  Must use `KEEP_LAST_HISTORY_QOS with depth=1`

These restrictions are in place to align with the deterministic nature of TSN networks, where re-transmissions
and history caching would potentially disrupt the time-critical delivery of messages.
