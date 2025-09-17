.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _ip-mobility:

IP mobility |Pro|
=================

DDS :ref:`Simple Discovery <simple_disc_settings>` relies on well-known multicast addresses and ports to relay the
Participant announcement messages (see :ref:`disc_phases`).
Such Participant announcement includes information about the unicast address-port pairs (a.k.a locators) where the
Participant is expecting to receive incoming metatraffic data.
The list with these unicast locators is automatically initialized taking into account the network interfaces that are
available when the *Fast DDS* DomainParticipant is enabled.

Fast DDS Pro detects changes in the system's IP addresses, and automatically updates them through the discovery
protocol, so other participants start sending data to the new addresses.

.. important::

   This feature is only available in Windows and Linux.
   MacOS will be supported in future releases.
