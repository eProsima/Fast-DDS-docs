.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _freq_transport_layer_questions:

TRANSPORT LAYER Frequently Asked Questions
==========================================

Transport API
-------------

.. collapse::  What is the role of the "TransportDescriptorInterface"?




    |br|

    It acts as a builder for a given transport, allowing to configure the transport and building it, using its ``create_transport`` factory member function. For further information, see :ref:`transport_transportApi_transportDescriptor`.



-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is the transport instance created?




    |br|

    Applications do not create the ``Transport`` instance themselves. Instead, applications use a ``TransportDescriptor`` instance to configure the desired transport, and add this configured instance to the list of user-defined transports of the DomainParticipant. The DomainParticipant will use the factory function on the ``TransportDescriptor`` to create the ``Transport`` when required. For further information, see :ref:`transport_transportApi_transport`.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Can I modify the ``transport_kind_`` data member?




    |br|

    No. ``transport_kind_`` is a protected data member for internal use. It cannot be accessed nor modified from the public API. However, users that are implementing a custom Transport need to fill it with a unique constant value in the new implementation. For further information, see :ref:`transport_transportApi_transport`.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


.. collapse::  What is the function of a "Locator_t" in a DDS system?




    |br|

    A ``Locator_t`` uniquely identifies a communication channel with a remote peer for a particular transport. For further information, see :ref:`transport_transportApi_locator`.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


.. collapse::  What is the primary purpose of the "IPLocator" class?




    |br|

    ``IPLocator`` is an auxiliary static class that offers methods to manipulate IP-based locators. It is convenient when setting up a new UDP Transport or TCP Transport, as it simplifies setting IPv4 and IPv6 addresses, or manipulating ports. For further information, see :ref:`transport_transportApi_ipLocator`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


UDP Transport
-------------



.. collapse::  What is the primary characteristic of UDP transport in terms of communication?




    |br|

    UDP is a connectionless transport, where the receiving DomainParticipant must open a UDP port listening for incoming messages, and the sending DomainParticipant sends messages to this port. For further information, see :ref:`transport_udp_udp`.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



.. collapse::  What is the best configuration for high-frequency best-effort writers?




    |br|

    Setting ``non_blocking_send`` to true. It is also applicable for TCP Transport. For further information, see :ref:`transport_udp_transportDescriptor`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is required to enable a new UDP transport in a DomainParticipant?




    |br|

    Fast DDS enables a UDPv4 transport by default. Nevertheless, the application can enable other UDP transports if needed. To enable a new UDP transport in a |DomainParticipant-api|, first create an instance of |UDPv4TransportDescriptor-api| (for UDPv4) or |UDPv6TransportDescriptor-api| (for UDPv6), and add it to the user transport list of the DomainParticipant. For further information, see :ref:`transport_udp_enabling`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

TCP Transport
-------------



.. collapse::  What is the primary requirement for establishing a connection using TCP transport in DDS?




    |br|

    TCP is a connection-oriented transport, so the DomainParticipant must establish a TCP connection to the remote peer before sending data messages. Therefore, one of the communicating DomainParticipants (the one acting as server) must open a TCP port listening for incoming connections, and the other one (the one acting as *client*) must connect to this port. For further information, see :ref:`transport_tcp_tcp`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens if there are multiple listening ports in TCP transport?




    |br|

    Only the first listening port will be effectively used. The rest of the ports will be ignored. For further information, see :ref:`transport_tcp_transportDescriptor`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens if "listening_ports" is left empty?




    |br|

    If ``listening_ports`` is left empty, the participant will not be able to receive incoming connections but will be able to connect to other participants that have configured their listening ports. For further information, see :ref:`transport_tcp_transportDescriptor`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is TCP transport enabled?




    |br|

    There are several ways of enabling TCP transport in Fast-DDS. The first option is to modify the builtin transports that are responsible for the creation of the DomainParticipant transports. The second option is to set up a Server-Client configuration. You need to create an instance of |TCPv4TransportDescriptor-api| (for TCPv4) or |TCPv6TransportDescriptor-api| (for TCPv6), and add it to the user transport list of the DomainParticipant. For further information, see :ref:`transport_tcp_enabling`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What configuration is required on the server side to enable communication with the client over TCP?




    |br|

    The router must be configured to forward traffic incoming to port 5100. For further information, see :ref:`transport_tcp_transportDescriptor`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Which configuration is needed to allow incoming connections through a WAN?




    |br|

    The |TCPv4TransportDescriptor-api| must indicate its public IP address in the "wan_addr" data member. On the client side, the DomainParticipant must be configured with the public IP address and "listening_ports" of the TCP as Initial peers.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Is there any configuration that makes the TCP transport more secure?




    |br|

    Yes, with TLS by including ``apply_security=true`` data member and ``tls_config`` data member on the ``TCPTransportDescriptor``. For further information, see :ref:`transport_tcp_tls`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Shared Memory Transport
-----------------------

.. collapse::  What is the primary advantage of using the Shared Memory Transport (SHM) compared to other network transports like UDP/TCP?




    |br|

    The shared memory (SHM) transport enables fast communications between entities running in the same processing unit/machine, relying on the shared memory mechanisms provided by the host operating system. For further information, see :ref:`transport_sharedMemory_sharedMemory`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How are peers running in the same host identified?




    |br|

    Using the DomainParticipant's ``GuidPrefix_t``. Two participants with identical 4 first bytes on the ``GuidPrefix_t`` are considered to be running in the same host. For further information, see :ref:`transport_sharedMemory_sharedMemory`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is a Segment Buffer in the context of the Shared Memory Transport?




    |br|

    A buffer allocated in the shared memory Segment. It works as a container for a DDS message that is placed in the Segment. In other words, each message that the DomainParticipant writes on the Segment will be placed in a different buffer. For further information, see :ref:`transport_sharedMemory_concepts_buffer`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is a Buffer Descriptor?




    |br|

    It acts as a pointer to a specific Segment Buffer in a specific Segment. It contains the *segmentId* and the offset of the Segment Buffer from the base of the Segment. When communicating a message to other DomainParticipants, Shared Memory Transport only distributes the Buffer Descriptor, avoiding the copy of the message from a DomainParticipant to another. With this descriptor, the receiving DomainParticipant can access the message written in the buffer, as is uniquely identifies the Segment (through the segmentId) and the Segment Buffer (through its offset). For further information, see :ref:`transport_sharedMemory_concepts_bufferDescriptor`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the primary purpose of creating a listener for a DomainParticipant's receiving port in the context of Shared Memory Transport?




    |br|

    DomainParticipants create a listener to their receiving port, so that they can be notified when a new Buffer Descriptor is pushed to the port. For further information, see :ref:`transport_sharedMemory_concepts_port`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the purpose of performing a health check when a DomainParticipant opens a Port?




    |br|

    Every time a DomainParticipant opens a Port (for reading or writing), a health check is performed to assess its correctness. The reason is that if one of the processes involved crashes while using a Port, that port can be left inoperative. If the attached listeners do not respond in a given timeout, the Port is considered damaged, and it is destroyed and created again. For further information, see :ref:`transport_sharedMemory_concepts_portHealthcheck`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How is SHM transport enabled?




    |br|

    Fast DDS enables a SHM transport by default. Nevertheless, the application can enable other SHM transports if needed. To enable a new SHM transport in a DomainParticipant, first create an instance of |SharedMemTransportDescriptor-api|, and add it to the user transport list of the DomainParticipant. For further information, see :ref:`transport_sharedMemory_enabling`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What happens to discovery traffic when multiple transports are enabled in a DomainParticipant?




    |br|

    The discovery traffic is always performed using the UDP/TCP transport, even if the SHM transport is enabled in both participants running in the same machine. For further information, see :ref:`transport_sharedMemory_enabling`.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


Data Sharing Delivery
---------------------


.. collapse::  What is the difference between Data Sharing Delivery and Shared Memory Transport?




    |br|

    Although Data-sharing delivery uses shared memory, it differs from Shared Memory Transport in that Shared Memory is a full-compliant transport. That means that with Shared Memory Transport the data being transmitted must be copied from the DataWriter history to the transport and from the transport to the DataReader. With Data-sharing these copies can be avoided. For further information, see :ref:`datasharing-delivery`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Does the use of Data-sharing avoid data copies between the application and the DataReader and DataWriter?




    |br|

    No, it does not prevent data copies. For further information, see :ref:`datasharing-delivery`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How are data copies prevented?




    |br|

    Data copies can be prevented in some cases using Zero-Copy communication. For further information, see :ref:`datasharing-delivery`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is required for two entities to use data-sharing delivery between them, according to their DataSharingQosPolicy configuration?




    |br|

    Two entities will be able to use data-sharing delivery between them only if both have at least a common domain. For further information, see :ref:`datasharing-delivery`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the default value for the maximum number of Data-sharing domain identifiers, and what does it affect?




    |br|

    By default, there is no limit to the number of identifiers. The default value can be changed using the ``max_domains()`` function. For further information, see :ref:`datasharing-delivery`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  What is the consequence of the DataWriter reusing a sample from the pool to publish new data?




    |br|

    The DataReader loses access to the old data sample. For further information, see :ref:`datareader-datawriter-history-coupling`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  Can the communications between entities be sped up?




    |br|

    Yes, but only within the same process using intra-process delivery. For further information, see :ref:`intraprocess-delivery`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

.. collapse::  How are peers running in the same process identified?




    |br|

    Fast DDS utilizes the DomainParticipant's ``GuidPrefix_t`` to identify peers running in the same process. Two participants with identical 8 first bytes on the "GuidPrefix_t" are considered to be running in the same process. For further information, see :ref:`intraprocess_delivery_guids`.

|
