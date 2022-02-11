.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _dynamic-network-interfaces:

Dynamic network interfaces
==========================

DDS :ref:`Simple Discovery <simple_disc_settings>` relies on well-known multicast addresses and ports to relay the
Participant announcement messages (see :ref:`disc_phases`).
Such Participant announcement includes information about the unicast address-port pairs (a.k.a locators) where the
Participant is expecting to receive incoming metatraffic data.
The list with these unicast locators is automatically initialized taking into account the network interfaces that are
available when the *Fast DDS* DomainParticipant is enabled.
Consequently, any network interface that is added after enabling the DomainParticipant should be notified to *Fast DDS*
in order to initialize an unicast locator in said network, so communication can be established over that new interface.

Dynamic network interface addition at run-time
----------------------------------------------

In case that the user wants to include new network interfaces at run-time, some prerequisites have to be fulfilled.
Then, once the interfaces are available, the user may notify Fast DDS so these interfaces are also used in the
communication.

Prerequisites
^^^^^^^^^^^^^

This feature is intended to be used when *Fast DDS* automatically sets the listening unicast locators.
Consequently, both |BuiltinAttributes::metatrafficUnicastLocatorList-api| and
|BuiltinAttributes::metatrafficMulticastLocatorList-api| lists must be empty.
These attributes are set within the `builtin` member of |DomainParticipantQos::wire_protocol-api| contained in the
|DomainParticipantQos-api| (please refer to :ref:`dds_layer_domainParticipantQos`).

.. note::

   Be aware of the remote locators' collections limits set within the |DomainParticipantQoS| (please refer to
   :ref:`remotelocatorsallocationattributes`).
   It is recommended to use the highest number of local addresses found on all the systems belonging to the same domain.

Notify *Fast DDS*
^^^^^^^^^^^^^^^^^

Once a new network interface has been enabled, *Fast DDS* has to be manually notified.
This is done calling |DomainParticipant::set_qos-api|.
The DomainParticipantQoS that is passed to the method can either change one of the mutable DomainParticipant QoS or
it can simply be the current DomainParticipant QoS (obtained with |DomainParticipant::get_qos-api|).

Using |DomainParticipant::set_qos-api| is the reason for the previous prerequisites: once the DomainParticipant is
enabled, there are several QoS policies that are immutable and cannot be changed at run-time.
:ref:`wireprotocolconfigqos` where the aforementioned lists are defined is among these immutable policies.

Find below a brief snippet of how to use this feature:

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: //DYNAMIC_NETWORK_INTERFACES_USE_CASE
   :end-before: //!--
   :dedent: 8

.. important::

   This feature is still under development and only officially supported for UDPv4 Transport without whitelisting.
