.. include:: ../../03-exports/aliases-api.include

.. _announced_locators:

Announced Locators
==================

In order for communication to take place, DDS entities need to exchange the list of addresses and ports where
they can be reached.
Apart from the :ref:`default announced locators<default_announced_locators>`, which correspond to addresses
of the interfaces in the host where the application is running, the user can configure
:ref:`additional locators<external_locators>` with addresses and ports on other networks, when routing rules
have been correspondingly set up.

.. _default_announced_locators:

Default Announced Locators
--------------------------

The default list of announced locators will be constructed from the :ref:`listening locators<listening_locators>`,
as follows:

* If the address field of the locator is a null address (i.e. 0.0.0.0 for UDPv4), a locator of the same kind and port
  will be announced for each of the addresses of the network interfaces of the host.
* If the address field of the locator is not a null address, a single locator with that address will be announced.

.. _external_locators:

External Locators
-----------------

The user can configure a set of external locators for each of the lists of unicast locators.
An external locator is made up of the standard locator fields (kind, address, and port), plus the following attributes:

* An *externality* that indicates the number of hops from the host where the application is running to the LAN
  represented by the external locator.
* A *cost* indicating the communication cost relative to other locators on the same externality level.
* A *mask* with the number of significant bits on the LAN represented by the external locator.

Externality levels
^^^^^^^^^^^^^^^^^^

The main purpose of the external locators is to enable communication accross different levels of interconnected LANs.
Communication will be performed using the locators of the innermost LAN available.

As an example, consider a network topology where the application is running on a host connected to a LAN of an office,
which in turn connects to a LAN for all the offices in the same floor, which in turn connects to a LAN for the
building.

With the default configuration, communication will only occur between hosts on the LAN for the office.
This is considered the externality level 0, which is reserved for the LANs directly connected to the network interfaces
of the host where the application is running.
This is the externality level that will be used on the matching algorithm for the
:ref:`default announced locators<default_announced_locators>`.

The floor LAN will be configured as externality level 1.
The building LAN will be configured as externality level 2.

Note that in order for the communication to be successful, routing rules should most probably need to be added to the
different network routers.

Additional considerations
^^^^^^^^^^^^^^^^^^^^^^^^^

Since using external locators increases the number of locators announced, the
:ref:`allocation limits for locators discovery<remotelocatorsallocationattributes>` would need to be adjusted for
your application.

Participants running on the same host, but using different addresses on their
|BuiltinAttributes::metatrafficMulticastLocatorList-qos-api| will discard shared memory transport locators.
Data sharing communication is not affected by this limitation.
