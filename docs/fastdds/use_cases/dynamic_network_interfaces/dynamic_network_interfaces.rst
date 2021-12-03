.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _dynamic-network-interfaces:

Dynamic network interfaces
==========================

Background
----------

DDS :ref:`simple_disc_settings` relies on well-known multicast addresses and ports to relay the Participant announcement
messages (:ref:`disc_phases`).
Such Participant announcement includes information about the unicast address and port where the Participant is expecting
to receive incoming meta data traffic.
The unicast metatraffic list is automatically initialized taking into account the network interfaces that are available
when the *Fast DDS* DomainParticipant is enabled.
Consequently, any network interface that is added after enabling the DomainParticipant should be notified to *Fast DDS*
in order to initialize a unicast locator in said network, so communication can be established over that new network
interface.

Dynamic network interface addition at run-time
----------------------------------------------

Prerequisites
^^^^^^^^^^^^^

This feature is intended to be used when *Fast DDS* automatically sets the listening unicast locators.
Consequently, both |BuiltinAttributes::metatrafficUnicastLocatorList-api| and
|BuiltinAttributes::metatrafficMulticastLocatorList-api| lists must be empty.
This attribute is set within the `builtin` member of |DomainParticipantQos::wire_protocol-api| contained in the
|DomainParticipantQos-api| (please refer to :ref:`dds_layer_domainParticipantQos`).
Otherwise a warning is going to be issued because these attributes are immutable once the DomainParticipant has been
enabled.

Notify *Fast DDS*
^^^^^^^^^^^^^^^^^

Once a new network interface has been enabled, *Fast DDS* has to be manually notified.
This is done calling |DomainParticipant::set_qos-api|.
The DomainParticipantQoS that is passed to the method can either change one of the mutable DomainParticipant QoS or
it can simply be the current DomainParticipant QoS (obtained with |DomainParticipant::get_qos-api|).
