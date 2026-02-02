.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. warning::

  The configuration options described in this section apply to network interfaces.
  Therefore, it is only available on :ref:`transport_tcp_tcp` and :ref:`transport_udp_udp`.

.. _ifaces_config:

Interfaces configuration
========================

By default, *Fast DDS* makes use of all active network interfaces found in the system to communicate (note: applies to
:ref:`transport_udp_udp` and :ref:`transport_tcp_tcp`).
However, it is possible for a user to indicate a specific set of network interfaces to be used by the library,
and/or configure some of them in a specific manner.

.. _netmask_filtering:

Netmask filtering
-----------------

The standard behaviour in *Fast-DDS* is to attempt data transmission to any remote locator for which a compatible
transport (based on :ref:`kind <transport_transportApi_kind>`) is registered.
This may result in a non-optimum resource utilization, as messages could be sent from an interface to an unreachable
destination given a particular network architecture.
In this situation, a user may decide to enable the netmask filtering feature, which would prevent this behavior by only
sending data from a network interface to remote locators within the same subnetwork.

This configuration option can be set at participant, transport and interface levels, and its possible values are:

.. _netmask_filter_values:

.. list-table::
   :header-rows: 1
   :align: left

   * - Value
     - Description
   * - |NetmaskFilterKind::ON-api|
     - Enable netmask filtering.
   * - |NetmaskFilterKind::OFF-api|
     - Disable netmask filtering.
   * - |NetmaskFilterKind::AUTO-api|
     - Use container's netmask filter configuration.

An |NetmaskFilterKind::AUTO-api| netmask filter configuration means that its effective value will be given by that of
its "container", which in the case of an :ref:`allowlist <interfaces_allowlist>` entry would be the
:ref:`transport descriptor <transportdescriptors>` where it is included, and in the case of a transport descriptor
would be the participant where it is registered.

However not all configurations are valid; this is, for example, a transport's netmask filter configuration cannot
be |NetmaskFilterKind::OFF-api| if it is |NetmaskFilterKind::ON-api| for the participant where this transport
is registered.
Likewise, the netmask filter configuration for an allowlist entry cannot be |NetmaskFilterKind::ON-api| if it is
|NetmaskFilterKind::OFF-api| for the transport descriptor where this allowlist is defined.

.. note::

    Due to implementation details, it is required to set `ignore_non_matching_locators` to `true`
    (see :ref:`Matching algorithm <external_locators_algorithm>`) both in :ref:`participants <wireprotocolconfigqos>`
    and :ref:`endpoints <rtpsendpointqos>` when enabling the netmask filtering feature at participant or transport level
    without defining an :ref:`allowlist <interfaces_allowlist>` or :ref:`blocklist <interfaces_blocklist>`.

Additional considerations need to be taken into account when using netmask filtering in combination with
:ref:`external locators <external_locators>`.
In particular, it is not possible to enable netmask filtering in all entries of an allowlist when a set of local
external locators (with an *externality* greater than `0`) is defined for a :ref:`participant <wireprotocolconfigqos>`
or :ref:`endpoint <rtpsendpointqos>`.
The reason for this is that a matching remote external locator would then (most likely) be effectively ignored, as no
network interface would be able to reach it according to its network mask.

Netmask filtering can be enabled at participant level both via C++ API or XML configuration:

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-NETMASK-FILTER
        :end-before: //!--
        :lines: 3-5
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->PARTICIPANT-NETMASK-FILTER
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

For socket (UDP/TCP) transport descriptors:

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-NETMASK-FILTER
        :end-before: //!--
        :lines: 7-12
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->TRANSPORT-NETMASK-FILTER
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

See :ref:`Allowlist <interfaces_allowlist>` to learn how to configure netmask filtering for specific network devices.


.. _interfaces_allowlist:

Allowlist
---------

Using *Fast DDS*, it is possible to limit the network interfaces used by :ref:`transport_tcp_tcp`
and :ref:`transport_udp_udp`.
This is achieved by adding the interfaces to the |SocketTransportDescriptor::interface_allowlist-api| field in the
:ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.
Thus, the communication interfaces used by the |DomainParticipants| whose |TransportDescriptorInterface-api| defines an
|SocketTransportDescriptor::interface_allowlist-api| is limited to the interfaces defined in that list, therefore
avoiding the use of the rest of the network interfaces available in the system.
The interfaces in |SocketTransportDescriptor::interface_allowlist-api| can be specified both by IP address
or interface name.
Additionally, each entry added to the allowlist may specify a :ref:`netmask filter <netmask_filtering>`
configuration value (|NetmaskFilterKind::AUTO-api| by default).

For example:

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-INTERFACES-ALLOWLIST
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->INTERFACES-ALLOWLIST
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

.. important::

  If none of the values in the transport descriptor's allowlist match the interfaces on the host, then all the
  interfaces in the allowlist are filtered out and therefore no communication will be established through that
  transport.


.. _interfaces_blocklist:

Blocklist
---------

Apart from defining a list of :ref:`allowed interfaces <interfaces_allowlist>`, it is also possible to define a list of
interfaces that should be blocked.
This is accomplished through the |SocketTransportDescriptor::interface_blocklist-api| field present in the
:ref:`transport_tcp_transportDescriptor` or :ref:`transport_udp_transportDescriptor`.

.. note::
  This list takes precedence over the allowlist, so if a network interface is in both lists, it will be blocked.

The interfaces in |SocketTransportDescriptor::interface_blocklist-api| can be specified both by IP address
or interface name.

For example:

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-INTERFACES-BLOCKLIST
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->INTERFACES-BLOCKLIST
        :end-before: <!--><-->
        :lines: 2-3,5-
        :append: </profiles>

