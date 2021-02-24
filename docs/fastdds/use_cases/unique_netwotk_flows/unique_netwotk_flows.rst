.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _use-case-unique-flows:

Unique network flows
====================

This section explains which APIs should be used on Fast DDS in order to have unique network flows on specific topics.

.. contents::
    :local:
    :backlinks: none
    :depth: 1

Background
----------

IP networking is the pre-dominant inter-networking technology used nowadays.
Ethernet, WiFi, 4G/5G telecommunication all rely on IP networking.

Streams of IP packets from a given source to destination are called *packet flows* or simply *flows*.
The network QoS of a flow can be configured when using certain networking equipment (routers, switches).
Such equipments are typically supporting 3GPP/5QI protocols to assign certain Network QoS parameters to
specific flows.

Requesting a specific Network QoS is usually done on the endpoint sending the data, as it is the one
that usually haves complete information about the network flow.

Applications may need to use specific Network QoS parameters on different topics.

This means an application should be able to

a) Identify the flows being used in the communications, so they can correctly configure the networking
   equipment.
b) Use specific flows on selected topics.

Identifying a flow
------------------

The *5-tuple* is a traditional unique identifier for flows on 3GPP enabled equipment.
The 5-tuple consists of five parameters: source IP address, source port, destination IP address, destination port,
and the transport protocol (example, TCP/UDP).

Fast DDS provides the APIs needed for the identification of such flows.

Requesting unique flows
-----------------------

:ref:`rtpsendpointqos`
