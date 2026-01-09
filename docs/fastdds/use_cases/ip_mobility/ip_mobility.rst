.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _ip-mobility:

IP mobility |Pro|
=================

DDS :ref:`Simple Discovery <simple_disc_settings>` relies on well-known multicast addresses and ports to relay
the Participant announcement messages (see :ref:`disc_phases`).
Such Participant announcement includes information about the unicast address-port pairs (a.k.a. locators) where
the Participant is expecting to receive incoming metatraffic data.
The list with these unicast locators is automatically initialized taking into account the network interfaces that
are available when the *Fast DDS* DomainParticipant is enabled.

Fast DDS Pro detects changes in the system's IP addresses, and automatically updates them through the discovery
protocol, so other participants start sending data to the new addresses.

.. important::

   This feature is still under development and is only officially supported for UDPv4 Transport without whitelisting.

   This feature is only available in Windows and Linux.
   MacOS will be supported in future releases.

Prerequisites
-------------

This feature is intended to be used when *Fast DDS* automatically sets the listening unicast locators.
Consequently, all the locator lists in |DomainParticipantQos::wire_protocol-api| must be empty when the participant
is created.
If any of them is not empty, no updates will be transmitted to other participants, and communication with them will
stop.

These locator lists are:

* |BuiltinAttributes::metatrafficUnicastLocatorList-api|
* |BuiltinAttributes::metatrafficMulticastLocatorList-api|
* |WireProtocolConfigQos::default_unicast_locator_list-api|
* |WireProtocolConfigQos::default_multicast_locator_list-api|

Please refer to :ref:`dds_layer_domainParticipantQos` for more information about these attributes.

.. note::

   Be aware of the remote locators' collections limits set within the |DomainParticipantQoS|
   (please refer to :ref:`remotelocatorsallocationattributes`).
   It is recommended to use the highest number of local addresses found on all the systems belonging to the same
   domain.

.. _tcp-keep-alive:

TCP Keep Alive |Pro|
====================

Fast DDS Pro additionally provides a TCP keep-alive mechanism to detect stalled TCP connections and recover from link failures
more reliably.
When enabled, the TCP transport periodically sends keep-alive requests on established TCP channels, and monitors
activity to determine whether a connection is still responsive.
Keep alive requests are sent only when no incoming data has been received for a specified interval.

If the configured timeout is reached without observing activity after a keep-alive request, the channel is
considered broken and is closed, allowing Fast DDS to reconnect and restore communication.

Note that this feature operates at the application level, independently of any TCP keep-alive settings at the OS level.
This allows for more fine-grained control over connection monitoring and recovery within Fast DDS.
It also ensures consistent behavior across different operating systems and increases portability.

Keep alive is disabled by default and can be configured through the following parameters in
|TCPTransportDescriptor-api|:

.. tab-set::

    .. tab-item:: XML
        :sync: xml

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->CONF-TCP-KEEP-ALIVE<-->
            :end-before: <!--><-->
            :lines: 2-3, 6-
            :append: </profiles>

    .. tab-item:: C++
        :sync: cpp

        .. literalinclude:: ../../../../code/DDSCodeTester.cpp
            :language: c++
            :dedent: 8
            :start-after: //CONF-TCP-KEEP-ALIVE
            :end-before: //!

.. note::

   This feature is intended for TCP transports and only applies to connected TCP channels.
