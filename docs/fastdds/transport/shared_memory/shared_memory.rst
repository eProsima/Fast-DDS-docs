.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../includes/aliases.rst

.. _transport_sharedMemory_sharedMemory:

Shared Memory Transport
=======================

The shared memory (SHM) transport enables fast communications between entities running in the same
processing unit/machine, relying on the shared memory mechanisms provided by the host operating system.

.. note::

    Fast DDS utilizes the |DomainParticipant|'s |GuidPrefix_t-api| to identify peers running in the same host.
    Two participants with identical 4 first bytes on the |GuidPrefix_t-api| are considered to be running in the same
    host.
    |is_on_same_host_as-api| API is provided to check this condition.
    Please, take also into account the caveats included in :ref:`intraprocess_delivery_guids`.

SHM transport provides better performance than other network transports like UDP / TCP,
even when these transports use loopback interface.
This is mainly due to the following reasons:

 * Large message support: Network protocols need to fragment data in order to comply with the specific protocol and
   network stacks requirements, increasing communication overhead.
   SHM transport allows the copy of full messages where the only size limit is the machine's memory capacity.

 * Reduce the number of memory copies: When sending the same message to different endpoints, SHM transport can
   directly share the same memory buffer with all the destination endpoints.
   Other protocols require to perform one copy of the message per endpoint.

 * Less operating system overhead: Once initial setup is completed, shared memory transfers require much less system
   calls than the other protocols.
   Therefore, there is a performance/time consume gain by using SHM.


.. _transport_sharedMemory_concepts:

Definition of Concepts
----------------------

This section describes basic concepts to help explain how the Shared Memory Transport works in order
to deliver the data messages to the appropriate :ref:`dds_layer_domainParticipant`.
The purpose is not to be an exhaustive reference of the implementation, but to be a comprehensive explanation
of each concept, so that users can configure the transport to their needs.

Many of the descriptions in this section will be made following the example use case depicted in the following figure,
where *Participant 1* sends a data message to *Participant 2*.
Please, refer to the figure when following the definitions.

.. figure:: /01-figures/shm_comm_sequence_diagram.svg
    :align: center

    Sequence diagram for Shared Memory Transport


.. _transport_sharedMemory_concepts_segment:

Segment
^^^^^^^

A *Segment* is a block of shared memory that can be accessed from different processes.
Every DomainParticipant that has been configured with Shared Memory Transport
creates a segment of shared memory.
The DomainParticipant writes to this segment any data it needs to deliver to other
DomainParticipants, and the remote
DomainParticipants are able to read it directly using the
shared memory mechanisms.

.. note::

    Launching any of the processes with a higher privileged user (for instance, *root*)
    can lead to communication problems, as processes run by non-privileged users may
    not be able to write into the memory segment.

Every segment has a *segmentId*, a 16-character UUID that uniquely identifies each shared memory segment.
These *segmentIds* are used to identify and access the segment of each DomainParticipant.

.. _transport_sharedMemory_concepts_buffer:

Segment Buffer
^^^^^^^^^^^^^^

A buffer allocated in the shared memory Segment.
It works as a container for a DDS message that is placed in the Segment.
In other words, each message that the DomainParticipant writes on the
Segment will be placed in a different buffer.

.. _transport_sharedMemory_concepts_bufferDescriptor:

Buffer Descriptor
^^^^^^^^^^^^^^^^^

It acts as a pointer to a specific Segment Buffer
in a specific Segment.
It contains the *segmentId* and the offset of the Segment Buffer from the base of the
Segment.
When communicating a message to other DomainParticipants,
Shared Memory Transport only distributes the Buffer Descriptor, avoiding the copy of
the message from a DomainParticipant to another.
With this descriptor, the receiving DomainParticipant can access the message written in the buffer,
as is uniquely identifies the Segment (through the *segmentId*)
and the Segment Buffer (through its offset).

.. _transport_sharedMemory_concepts_port:

Port
^^^^
Represents a channel to communicate Buffer Descriptors.
It is implemented as a ring-buffer in shared memory, so that any DomainParticipant
can potentially read or write information on it.
Each port has a unique identifier, a 32 bit number that can be used to refer to the port.
Every DomainParticipant that has been configured with Shared Memory Transport
creates a port to receive Buffer Descriptors.
The identifier of this port is shared during the :ref:`discovery`, so that remote peers know which port to use
when they want to communicate with each DomainParticipant.

DomainParticipants create a listener to their receiving port,
so that they can be notified when a new Buffer Descriptor is pushed to the port.

.. _transport_sharedMemory_concepts_portHealthCheck:

Port Health Check
^^^^^^^^^^^^^^^^^
Every time a DomainParticipant opens a Port
(for reading or writing), a health check is performed to assess its correctness.
The reason is that if one of the processes involved crashes while using a Port,
that port can be left inoperative.
If the attached listeners do not respond in a given timeout, the Port
is considered damaged, and it is destroyed and created again.


.. _transport_sharedMemory_transportDescriptor:

SharedMemTransportDescriptor
----------------------------

In addition to the data members defined in the :ref:`transport_transportApi_transportDescriptor`,
the TransportDescriptor for Shared Memory defines the following ones:


.. list-table::
   :header-rows: 1
   :align: left

   * - Member
     - Data type
     - Default
     - Accessor / Mutator
     - Description
   * - ``segment_size_``
     - ``uint32_t``
     - ``512*1024``
     - |SharedMemTransportDescriptor::segment_size-api|
     - Size of the shared memory segment |br|
       (in octets).
   * - ``port_queue_capacity_``
     - ``uint32_t``
     - ``512``
     - |SharedMemTransportDescriptor::port_queue_capacity-api|
     - The size of the listening port |br|
       (in messages).
   * - ``healthy_check_timeout_ms_``
     - ``uint32_t``
     - ``1000``
     - |SharedMemTransportDescriptor::healthy_check_timeout_ms-api|
     - Timeout for the health check of ports |br|
       (in milliseconds).
   * - ``rtps_dump_file_``
     - ``string``
     - ``""``
     - |SharedMemTransportDescriptor::rtps_dump_file-api|
     - Full path of the protocol dump file.
   * - |PortBasedTransportDescriptor::default_reception_threads-api|
     - |ThreadSettings|
     -
     - |PortBasedTransportDescriptor::default_reception_threads-api|
     - Default |ThreadSettings| for the reception threads.
   * - |PortBasedTransportDescriptor::reception_threads-api|
     - ``std::map<uint32_t, ThreadSettings>``
     -
     - |PortBasedTransportDescriptor::reception_threads-api|
     - |ThreadSettings| for the reception threads on specific ports.
   * - |SharedMemTransportDescriptor::dump_thread-api|
     - |ThreadSettings|
     -
     - |SharedMemTransportDescriptor::dump_thread-api|
     - |ThreadSettings| for the SHM dump thread.

If ``rtps_dump_file_`` is not empty, all the shared memory traffic on the DomainParticipant
(sent and received) is traced to a file.
The output file format is *tcpdump* hexadecimal text, and can be processed with protocol analyzer applications
such as Wireshark.
Specifically, to open the file using Wireshark, use the "Import from Hex Dump" option using the "Raw IPv4" encapsulation
type.

.. note::

   The |TransportInterface::kind-api| value for a |SharedMemTransportDescriptor-api| is given by the value
   |LOCATOR_KIND_SHM-api|.

.. warning::

    Setting a |SharedMemTransportDescriptor::segment_size-api| close to or smaller than the data size poses a high risk
    of data loss, since the write operation will overwrite the buffer during a single send operation.

.. _transport_sharedMemory_enabling:

Enabling Shared Memory Transport
--------------------------------

*Fast DDS* enables a SHM transport by default.
Nevertheless, the application can enable other SHM transports if needed.
To enable a new SHM transport in a :ref:`dds_layer_domainParticipant`, first
create an instance of :ref:`transport_sharedMemory_transportDescriptor`,
and add it to the user transport list of the :ref:`dds_layer_domainParticipant`.

The examples below show this procedure in both C++ code and XML file.

.. tab-set::

    .. tab-item:: C++
        :sync: cpp

        .. literalinclude:: /../code/DDSCodeTester.cpp
            :language: c++
            :start-after: //CONF-SHM-TRANSPORT-SETTING
            :end-before: //!--
            :dedent: 8

    .. tab-item:: XML
        :sync: xml

        .. literalinclude:: /../code/XMLTester.xml
            :language: xml
            :start-after: <!-->CONF-SHM-TRANSPORT-SETTING
            :end-before: <!--><-->
            :lines: 2-4,6-41,43-44

.. note::

  In case that several transports are enabled, the discovery traffic is always performed using the UDP/TCP transport,
  even if the SHM transport is enabled in both participants running in the same machine.
  This may cause discovery issues if one or several of the participants only has SHM enabled and other participants use
  some other transport at the same time.
  Also, when two participants on the same machine have SHM transport enabled, the user data communication between them
  is automatically performed by SHM transport only.
  The rest of the enabled transports are not used between those two participants.

.. hint::
  To configure discovery traffic through Shared Memory, the default builtin transports must be disabled.
  In that way, communication is performed completely using Shared Memory.
  The snippet examples below show this procedure in both C++ code and XML file.
  See :ref:`transport_sharedMemory_example` for a complete example.

  .. tab-set::

      .. tab-item:: C++
          :sync: cpp

          .. literalinclude:: /../code/DDSCodeTester.cpp
              :language: c++
              :start-after: //CONF-SHM-TRANSPORT-DISABLE-BUILTIN-TRANSPORTS
              :end-before: //!--
              :dedent: 8

      .. tab-item:: XML
          :sync: xml

          .. literalinclude:: /../code/XMLTester.xml
              :language: xml
              :start-after: <!-->CONF-SHM-TRANSPORT-DISABLE-BUILTIN-TRANSPORTS
              :end-before: <!--><-->
              :lines: 2-3,5-
              :append: </profiles>

.. _transport_sharedMemory_example:

Delivery Mechanisms example
---------------------------

A hello world example suitable for supported delivery mechanisms can be found in the
`delivery_mechanisms folder <https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/delivery_mechanisms>`_.
It shows a publisher and a subscriber that communicate through the desired delivery mechanism (which can be set to
shared memory only).
