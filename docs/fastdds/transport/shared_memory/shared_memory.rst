.. _transport_sharedMemory_sharedMemory:

Shared Memory Transport
=======================

The shared memory (SHM) transport enables fast communications between entities running in the same
processing unit/machine, relying on the shared memory mechanisms provided by the host operating system.

SHM transport provides better performance than other network transports like UDP / TCP,
even when these transports use loopback interface.
This is mainly due to the following reasons:

 * Large message support: Network protocols need to fragment data in order to comply with the specific protocol and
   network stacks requirements, increasing communication overhead
   SHM transport allows the copy of full messages where the only size limit is the machine's memory capacity.

 * Reduce the number of memory copies: When sending the same message to different endpoints, SHM transport can
   directly share the same memory buffer with all the destination endpoints.
   Other protocols require to perform one copy of the message per endpoint.

 * Less operating system overhead: Once initial setup is completed, shared memory transfers require much less system
   calls than the other protocols.
   Therefore there is a performance/time consume gain by using SHM.


.. _transport_sharedMemory_concepts:

Definition of Concepts
----------------------

This section describes basic concepts that will help understanding how the Shared Memory Transport works in order
to deliver the data messages to the appropriate :ref:`dds_layer_domainParticipant`.
The purpose is not to be a exhaustive reference of the implementation, but to be a comprehensive explanation
of each concept, so that users can configure the transport to their needs.

Many of the descriptions in this section will be made following the example use case depicted in the following figure,
where *Participant 1* sends a data message to *Participant 2*.
Please, refer to the figure when following the definitions.

.. figure:: /01-figures/shm_comm_sequence_diagram.svg
    :align: center

    Sequence diagram for Shared Memory Transport

* **Segment**: Is a block of shared memory that can be accessed from different processes.
  Every :ref:`dds_layer_domainParticipant` that has been configured with :ref:`transport_sharedMemory_sharedMemory`
  creates a segment of shared memory.
  The :ref:`dds_layer_domainParticipant` writes to this segment any data it needs to deliver to other
  :ref:`DomainParticipants<dds_layer_domainParticipant>`, and the remote
  :ref:`DomainParticipants<dds_layer_domainParticipant>` are able to read it directly using the
  shared memory mechanisms.

* **SegmentId**: Is a 16 character UUID that uniquely identifies shared memory segments.
  They are used to identify and access the segment of each :ref:`dds_layer_domainParticipant`.

* **Shared memory buffer**: Is a buffer allocated in the shared memory segment.
  It works as a container for a DDS message that is places in the segment.
  In other words, each message that the :ref:`dds_layer_domainParticipant` writes on the segment will
  be places in a different buffer.

* **Buffer descriptor**: It acts as a pointer to a specific memory buffer in a specific segment.
  It contains the segmentId and the offset of the buffer from the base of the segment.
  When communicating a message to other :ref:`DomainParticipants<dds_layer_domainParticipant>`,
  :ref:`transport_sharedMemory_sharedMemory` only distributes the Buffer Descriptor, avoiding the copy of
  the message from a :ref:`dds_layer_domainParticipant` to another.
  With this descriptor the receiving :ref:`dds_layer_domainParticipant` can access the message written on the buffer,
  as is uniquely identifies the segment (through the segmentId) and the buffer (through its offset).

* **Shared memory port**: Represents a channel to communicate Buffer Descriptors.
  It is implemented as a ring-buffer in shared memory, so that any :ref:`dds_layer_domainParticipant`
  can potentially read or write information on it.
  Each port has a unique identifier, a 32 bit number that can be used to refer to the port.
  Every :ref:`dds_layer_domainParticipant` that has been configured with :ref:`transport_sharedMemory_sharedMemory`
  creates a port to receive Buffer Descriptors.
  The identifier of this port is shared during the :ref:`discovery`, so that remote peers know which port to use
  when they want to communicate with each :ref:`DomainParticipants <dds_layer_domainParticipant>`.

* **Listener**: :ref:`DomainParticipants <dds_layer_domainParticipant>` create a listener to their receiving port,
  so that they can be notified when a new descriptor is pushed to the port.

* **Port healthcheck**: Every time a :ref:`dds_layer_domainParticipant` opens a port (for reading or writing),
  a health check is performed to assess it is correct.
  The reason is that if one of the processes involved crashes while using the port, the port can be left inoperative.
  If the attached listeners do not respond in a given timeout, the port is considered damaged, and it is destroyed and
  created again.


.. _transport_sharedMemory_transportDescriptor:

SharedMemTransportDescriptor
----------------------------

In addition to the data members defined in the :ref:`transport_transportApi_transportDescriptor`,
the TransportDescriptor for Shared Memory defines the following ones:

+------------------------------+----------------+-----------------------------------------------------------+
| Member                       | Data type      | Description                                               |
+==============================+================+===========================================================+
| ``segment_size_``            | ``uint32_t``   | The size of the shared memory segment, in bytes.          |
+------------------------------+----------------+-----------------------------------------------------------+
| ``port_queue_capacity_``     | ``uint32_t``   | The size of the listening port, in messages.              |
+------------------------------+----------------+-----------------------------------------------------------+
| ``healthy_check_timeout_ms_``| ``uint32_t``   | Timeout for the health check of ports.                    |
+------------------------------+----------------+-----------------------------------------------------------+
| ``rtps_dump_file_``          | ``string``     | Full path of the protocol dump_file.                      |
+------------------------------+----------------+-----------------------------------------------------------+

If ``rtps_dump_file_`` is not empty, all the shared memory traffic on the :ref:`dds_layer_domainParticipant`
(sent and received) is traced to a file.
The output file format is *tcpdump* hexadecimal text, and can be processed with protocol analyzer applications
such as Wireshark.

.. note::

   The *kind* value for a SharedMemTransportDescriptor is given by the value
   ``eprosima::fastrtps::rtps::LOCATOR_KIND_SHM``


.. _transport_sharedMemory_enabling:

Enabling Shared Memory Transport
--------------------------------

SHM transport is not enabled by default.
To enable SHM transport in a :ref:`dds_layer_domainParticipant`, you need to
create an instance of :ref:`transport_sharedMemory_transportDescriptor` and add it to the user transport list of the
:ref:`dds_layer_domainParticipant`.
The examples below show this procedure in both C++ code and XML file.

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

.. note:

  When two participants on the same machine have SHM transport enabled, all communications between them are automatically
  performed by SHM transport only.
  The rest of the enabled transports are not used between those two participants.


