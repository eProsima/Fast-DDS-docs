.. _congestion-control:

Congestion Control |Pro|
------------------------

When transmitting large amounts of data over a network, it is crucial to manage congestion to ensure optimal
performance and prevent packet loss.
Fast DDS |Pro| allows enabling a congestion control mechanism on a participant, which makes every DataWriter use
a different bandwidth limitation on each DataReader according to the network conditions.

This feature can be customized through :ref:`congestion control plugins <congestion-control-plugins>`, which define
the algorithm used to adjust the bandwidth based on the detected congestion.

To enable congestion control, the DomainParticipant must be configured with the appropriate properties.

.. _congestion-control-plugins:

Congestion Control Plugins |Pro|
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
