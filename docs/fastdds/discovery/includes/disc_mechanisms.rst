.. _disc_mechanisms:

Discovery mechanisms
--------------------

Fast DDS provides the following discovery mechanisms:

- :ref:`Simple Discovery <simple_disc_settings>`: This is the default mechanism.
  It upholds the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_ for both PDP and EDP, and therefore
  provides compatibility with any other DDS and RTPS implementations.

- :ref:`Static Discovery <static_edp>`: This mechanisms uses the Simple Participant Discovery Protocol (SPDP) for the
  PDP phase (as specified by the `RTPS standard <https://www.omg.org/spec/DDSI-RTPS/2.2/PDF>`_), but allows for skipping
  the Simple Endpoint Discovery Protocol (SEDP) phase when all the DataWriters' and DataReaders' IPs and ports,
  data types, and Topics are known beforehand.

- :ref:`Discovery Server <discovery_server>`: This discovery mechanism uses a centralized discovery architecture,
  where a DomainParticipant, referred as Server, act as a hub for discovery meta traffic.

- **Manual Discovery**: This mechanism is only compatible with the RTPS layer.
  It disables the PDP, letting the user to manually match and unmatch |RTPSParticipants-api|, |RTPSReaders-api|, and
  |RTPSWriters-api| using whatever external meta-information channel of its choice.
  Therefore, the user must access the RTPSParticipant implemented by the DomainParticipant and directly match the
  RTPS Entities.

