Discovery phases
----------------

#. **Participant Discovery Phase (PDP)**: During this phase the |DomainParticipants| acknowledge each other's existence.
   To do that, each |DomainParticipant| sends periodic announcement messages, which specify, among other things, unicast
   addresses (IP and port) where the |DomainParticipant| is listening for incoming meta and user data traffic.
   Two given |DomainParticipants| will match when they exist in the same DDS Domain.
   By default, the announcement messages are sent using well-known multicast addresses and ports (calculated using the
   |DomainId|).
   Furthermore, it is possible to specify a list of addresses to send
   announcements using unicast (see in :ref:`Simple Initial Peers`).
   Moreover, is is also possible to configure the periodicity of such announcements (see
   :ref:`Discovery Configuration <dconf>`).

#. **Endpoint Discovery Phase (EDP)**: During this phase, the |DataWriters| and |DataReaders| acknowledge each other.
   To do that, the |DomainParticipants| share information about their |DataWriters| and |DataReaders| with each other,
   using the communication channels established during the PDP.
   This information contains, among other things, the |Topic| and data type (see :ref:`dds_layer_topic`).
   For two endpoints to match, their topic and data type must coincide.
   Once |DataWriter| and |DataReader| have matched, they are ready for sending/receiving user data traffic.
