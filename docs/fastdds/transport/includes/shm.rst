.. _comm-transports-shm:

Shared memory Transport (SHM)
-----------------------------

The shared memory transport enables fast communications between entities running in the same processing unit/machine,
relying on the shared memory mechanisms provided by the host operating system.

SHM transport provides better performance than other transports like UDP / TCP, even when these transports use loopback
interface.
This is mainly due to the following reasons:

 * Large message support: Network protocols need to fragment data in order to comply with the specific protocol and
   network stacks requirements.
   SHM transport allows the copy of full messages where the only size limit is the machine's memory capacity.

 * Reduce the number of memory copies: When sending the same message to different endpoints, SHM transport can
   directly share the same memory buffer with all the destination endpoints.
   Other protocols require to perform one copy of the message per endpoint.

 * Less operating system overhead: Once initial setup is completed, shared memory transfers require much less system
   calls than the other protocols.
   Therefore there is a performance/time consume gain by using SHM.

When two participants on the same machine have SHM transport enabled, all communications between them are automatically
performed by SHM transport only.
The rest of the enabled transports are not used between those two participants.

In order to change the default parameters of SHM transport, you need to add the SharedMemTransportDescriptor to the
``rtps.userTransports`` attribute (C++ code) or define a transport_descriptor of type SHM in the
XML file. In both cases ``rtps.useBuiltinTransports`` must be disabled (see below examples).

+--------------------------------------------------+
| **C++**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/CodeTester.cpp      |
|    :language: c++                                |
|    :start-after: //CONF-SHM-TRANSPORT-SETTING    |
|    :end-before: //!--                            |
+--------------------------------------------------+
| **XML**                                          |
+--------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml       |
|    :language: xml                                |
|    :start-after: <!-->CONF-SHM-TRANSPORT-SETTING |
|    :end-before: <!--><-->                        |
+--------------------------------------------------+

SHM configuration parameters:

 * ``segment_size``: The size of the shared memory segment in bytes.
   A shared memory segment is created by each participant.
   Participant's writers copy their messages into the segment and send a message reference to the destination readers.

 * ``port_queue_capacity``: Each participant with SHM transport enabled listens on a queue (port) for incoming SHM
   message references.
   This parameter specifies the queue size (in messages).

 * ``healthy_check_timeout_ms``: With SHM, Readers and writers use a queue to exchange messages (called Port).
   If one of the processes involved crashes while using the port, the structure can be left inoperative.
   For this reason, every time a port is opened, a healthy check is performed.
   If the attached listeners don't respond in ``healthy_check_timeout_ms`` milliseconds, the port is destroyed and
   created again.

 * ``rtps_dump_file``: Full path, including the file name, of the protocol dump_file.
   When this string parameter is not empty, all the participant's SHM traffic (sent and received) is traced to a file.
   The output file format is *tcpdump* text hex, and can be read with protocol analyzer applications such as Wireshark.
