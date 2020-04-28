Discovery phases
----------------

#. **Participant Discovery Phase (PDP)**: During this phase the participants acknowledge each other's existence.
   To do that, each participant sends periodic announcement messages, which specify, among other things, unicast
   addresses (IP and port) where the participant is listening for incoming meta and user data traffic.
   Two given participants will match when they exist in the same domain.
   By default, the announcement messages are sent using well-known multicast addresses and ports (calculated using the
   domain).
   Furthermore, it is possible to specify a list of addresses to send
   announcements using unicast (see in :ref:`initial-peers`).
   Moreover, is is also possible to configure the periodicity of such announcements (see
   :ref:`Discovery Configuration <dconf>`).

#. **Endpoint Discovery Phase (EDP)**: During this phase, the publishers and subscribers acknowledge each other.
   To do that, the participants share information about their publishers and subscribers with each other, using the
   communication channels established during the PDP.
   This information contains, among other things, the topic and data type.
   For two endpoints to match, their topic and data type must coincide.
   Once publisher and subscriber have matched, they are ready for sending/receiving user data traffic.
