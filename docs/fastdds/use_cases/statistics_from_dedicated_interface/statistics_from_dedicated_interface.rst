.. include:: ../../../03-exports/aliases-api.include

.. _use-case-statistics-from-dedicated-interface:

Receive Statistics data from a dedicated interface
==================================================

*eProsima Fast DDS* is designed to flexibly manage data transmission across different network configurations.
The default behavior is attempting to transmit data to any remote locator for which a compatible transport
is registered. This behavior facilitates robust connectivity across diverse network environments, but may
not always align with specific user requirements. In some scenarios, the user may need to send specific data
through different interfaces. For example, the user may want to receive :ref:`statistics information
<statistics_module>` only over a dedicated interface in the same subnetwork.

Overview
--------

In this use case the requirement is sending statistics information only through a specific network interface
dedicated to monitor, while all other user data are transmitted through a different network interface.
Consider a scenario involving three participants:

*  Participant 1 and Participant 2 are engaged in regular data exchange.
*  Monitor Participant is tasked with monitoring the statistics information of Participant 2.

This configuration ensures that monitoring traffic does not interfere with or congest the primary data exchange channels.

Implementing data transmission over dedicated interfaces
--------------------------------------------------------

To achieve this level of control, users can leverage two features: :ref:`netmask filtering <netmask_filtering>` and
:ref:`unicast locators <listening_locators_userUnicast>`. By enabling the netmask filtering feature, Fast-DDS restricts
data transmission to only those remote locators that reside within the same subnetwork as the sending interface. This
is particularly useful in preventing the transmission of data to unintended or non-optimal network segments. While
setting a unicast locator with a specific interface address ensures that the Monitor Participant only communicates over
the network interface specifically intended for monitoring purposes.

Configuration
-------------

Participant 2 is configured with two unicast locators: One for interfacing with Participant 1, restricted to the subnetwork
that includes Participant 1. Another for sending statistics information to the Monitor Participant, restricted to the
monitoring subnetwork.
Monitor Participant is configured to listen on the unicast locator that corresponds to its dedicated monitoring
interface, ensuring it only receives statistics information and not regular user data.

.. image:: /01-figures/NETMASK_FILTER_ON.svg
    :align: center

Example
-------

Using netmask filtering in conjunction with specific unicast locators allows for management of data paths
in Fast-DDS, aligning network traffic with organizational needs and network architecture.

.. literalinclude:: /../code/DDSCodeTester.cpp
   :language: c++
   :start-after: // STATISTICS_DEDICATED_INTERFACE
   :end-before: //!--
