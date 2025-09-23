.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _ip-mobility:

IP mobility |Pro|
=================

DDS :ref:`Simple Discovery <simple_disc_settings>` relies on well-known multicast addresses and ports to relay the
Participant announcement messages (see :ref:`disc_phases`).
Such Participant announcement includes information about the unicast address-port pairs (a.k.a. locators) where the
Participant is expecting to receive incoming metatraffic data.
The list with these unicast locators is automatically initialized taking into account the network interfaces that are
available when the *Fast DDS* DomainParticipant is enabled.

Fast DDS Pro detects changes in the system's IP addresses, and automatically updates them through the discovery
protocol, so other participants start sending data to the new addresses.

.. important::

   This feature is still under development and only officially supported for UDPv4 Transport without whitelisting.

   This feature is only available in Windows and Linux.
   MacOS will be supported in future releases.

Prerequisites
-------------

This feature is intended to be used when *Fast DDS* automatically sets the listening unicast locators.
Consequently, all the locator lists in |DomainParticipantQos::wire_protocol-api| must be empty when the
participant is created.
If any of them is not empty, no updates will be transmitted to other participants, and communication with
them will stop.

These locator lists are:

* |BuiltinAttributes::metatrafficUnicastLocatorList-api|
* |BuiltinAttributes::metatrafficMulticastLocatorList-api|
* |WireProtocolConfigQos::default_unicast_locator_list-api|
* |WireProtocolConfigQos::default_multicast_locator_list-api|

Please refer to :ref:`dds_layer_domainParticipantQos` for more information about these attributes.

.. note::

   Be aware of the remote locators' collections limits set within the |DomainParticipantQoS| (please refer to
   :ref:`remotelocatorsallocationattributes`).
   It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.
