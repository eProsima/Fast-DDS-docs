.. _disc_mechanisms:

Discovery mechanisms
--------------------

Fast-RTPS provides the following discovery mechanisms:

- :ref:`Simple Discovery <simple_disc_settings>`: This is the default mechanism.
  It upholds the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_
  for both PDP and EDP phases, and therefore provides compatibility with any
  other DDS and RTPS implementations.

- :ref:`Static Discovery <static_edp>`: This mechanisms uses the Simple Participant Discovery Protocol (SPDP) for the
  PDP phase (as specified by the RTPS standard), but allows for skipping the Simple Participant Discovery Protocol
  (SEDP) phase when all the publishers' and subscribers' addresses and ports, data types, and topics are known
  beforehand.

- :ref:`Server-Client Discovery <discovery_server>`: This discovery mechanism uses a centralized discovery architecture,
  where servers act as a hubs for discovery meta traffic.

- **Manual Discovery**: This mechanism is only compatible with the ``RTPSDomain`` layer.
  It disables the PDP discovery phase, letting the user to manually match and unmatch RTPS participants, readers, and
  writers using whatever, external meta-information channel of its choice.
