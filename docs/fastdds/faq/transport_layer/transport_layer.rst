.. _freq_transport_layer_questions:

TRANSPORT LAYER Frequently Asked Questions
==========================================

.. raw:: html

   <style>
       .question-box {
           background-color: #e7f3fe;
           border-left: 6px solid #2196F3;
           padding: 10px;
           margin: 10px 0;
       }
       .answer-box {
           background-color: #f9f9f9;
           border-left: 6px solid #4CAF50;
           padding: 10px;
           margin: 10px 0 20px 0;
       }
   </style>

.. raw:: html

    <h2>Transport API</h2>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the role of the "TransportDescriptorInterface"?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> It acts as a builder for a given transport, allowing to configure the transport and building it, using its "create_transport" factory member function.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: How is the transport instance created?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Applications do not create the "Transport" instance themselves. Instead, applications use a "TransportDescriptor" instance to configure the desired transport, and add this configured instance to the list of user-defined transports of the DomainParticipant. The DomainParticipant will use the factory function on the "TransportDescriptor" to create the "Transport" when required.
    </div>

.. raw:: html
    
    <div class="question-box">
        <b>Q: Can I modify the "transport_kind _ " data member?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> No. "transport_kind _ " is a protected data member for internal use. It cannot be accessed nor modified from the public API. However, users that are implementing a custom Transport need to fill it with a unique constant value in the new implementation.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the function of a "Locator_t" in a DDS system?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> A "Locator_t" uniquely identifies a communication channel with a remote peer for a particular transport.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the primary purpose of the "IPLocator" class?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> "IPLocator" is an auxiliary static class that offers methods to manipulate IP-based locators. It is convenient when setting up a new UDP Transport or TCP Transport, as it simplifies setting IPv4 and IPv6 addresses, or manipulating ports.
    </div>

.. raw:: html

    <h2>UDP Transport</h2>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the primary characteristic of UDP transport in terms of communication?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> UDP is a connectionless transport, where the receiving DomainParticipant must open a UDP port listening for incoming messages, and the sending DomainParticipant sends messages to this port.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the best configuration for high-frequency best-effort writers?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Setting "non_blocking_send" to true. It is also applicable for TCP Transport.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is required to enable a new UDP transport in a DomainParticipant?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Fast DDS enables a UDPv4 transport by default. Nevertheless, the application can enable other UDP transports if needed. To enable a new UDP transport in a DomainParticipant, first create an instance of UDPv4TransportDescriptor (for UDPv4) or UDPv6TransportDescriptor (for UDPv6), and add it to the user transport list of the DomainParticipant.
    </div>

.. raw:: html

    <h2>TCP Transport</h2>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the primary requirement for establishing a connection using TCP transport in DDS?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> TCP is a connection-oriented transport, so the DomainParticipant must establish a TCP connection to the remote peer before sending data messages. Therefore, one of the communicating DomainParticipants (the one acting as *server*) must open a TCP port listening for incoming connections, and the other one (the one acting as *client*) must connect to this port.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What happens if there are multiple listening ports in TCP transport?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Only the first listening port will be effectively used. The rest of the ports will be ignored.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What happens if "listening_ports" is left empty?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> If "listening_ports" is left empty, the participant will not be able to receive incoming connections but will be able to connect to other participants that have configured their listening ports.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: How is TCP transport enabled?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> There are several ways of enabling TCP transport in Fast-DDS. The first option is to modify the builtin transports that are responsible for the creation of the DomainParticipant transports. The second option is to set up a Server-Client configuration. You need to create an instance of TCPv4TransportDescriptor (for TCPv4) or TCPv6TransportDescriptor (for TCPv6), and add it to the user transport list of the DomainParticipant.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What configuration is required on the server side to enable communication with the client over TCP?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> The router must be configured to forward traffic incoming to port 5100.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: Which configuration is needed to allow incoming connections through a WAN?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> The TCPv4TransportDescriptor must indicate its public IP address in the "wan_addr" data member. On the client side, the DomainParticipant must be configured with the public IP address and "listening_ports" of the TCP as Initial peers.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: Is there any configuration that makes the TCP transport more secure?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Yes, with TLS by including apply_security=true data member and tls_config data member on the TCPTransportDescriptor.
    </div>

.. raw:: html

    <h2>Shared Memory Transport</h2>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the primary advantage of using the Shared Memory Transport (SHM) compared to other network transports like UDP/TCP?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> The shared memory (SHM) transport enables fast communications between entities running in the same processing unit/machine, relying on the shared memory mechanisms provided by the host operating system.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: How are peers running in the same host identified?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Using the DomainParticipant's GuidPrefix_t. Two participants with identical 4 first bytes on the GuidPrefix_t are considered to be running in the same host.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is a Segment Buffer in the context of the Shared Memory Transport?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> A buffer allocated in the shared memory Segment. It works as a container for a DDS message that is placed in the Segment. In other words, each message that the DomainParticipant writes on the Segment will be placed in a different buffer.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is a Buffer Descriptor?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> It acts as a pointer to a specific Segment Buffer in a specific Segment. It contains the *segmentId* and the offset of the Segment Buffer from the base of the Segment. When communicating a message to other DomainParticipants, Shared Memory Transport only distributes the Buffer Descriptor, avoiding the copy of the message from a DomainParticipant to another. With this descriptor, the receiving DomainParticipant can access the message written in the buffer, as is uniquely identifies the Segment (through the *segmentId*) and the Segment Buffer (through its offset).
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the primary purpose of creating a listener for a DomainParticipant's receiving port in the context of Shared Memory Transport?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> DomainParticipants create a listener to their receiving port, so that they can be notified when a new Buffer Descriptor is pushed to the port.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the purpose of performing a health check when a DomainParticipant opens a Port?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Every time a DomainParticipant opens a Port (for reading or writing), a health check is performed to assess its correctness. The reason is that if one of the processes involved crashes while using a Port, that port can be left inoperative. If the attached listeners do not respond in a given timeout, the Port is considered damaged, and it is destroyed and created again.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: How is SHM transport enabled?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Fast DDS enables a SHM transport by default. Nevertheless, the application can enable other SHM transports if needed. To enable a new SHM transport in a DomainParticipant, first create an instance of SharedMemTransportDescriptor, and add it to the user transport list of the DomainParticipant.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What happens to discovery traffic when multiple transports are enabled in a DomainParticipant?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> The discovery traffic is always performed using the UDP/TCP transport, even if the SHM transport is enabled in both participants running in the same machine.
    </div>

.. raw:: html

    <h2>Data Sharing Delivery</h2>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the difference between Data Sharing Delivery and Shared Memory Transport?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Although Data-sharing delivery uses shared memory, it differs from Shared Memory Transport in that Shared Memory is a full-compliant transport. That means that with Shared Memory Transport the data being transmitted must be copied from the DataWriter history to the transport and from the transport to the DataReader. With Data-sharing these copies can be avoided.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: Does the use of Data-sharing avoid data copies between the application and the DataReader and DataWriter?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> No, it does not prevent data copies.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: How are data copies prevented?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Data copies can be prevented in some cases using Zero-Copy communication.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is required for two entities to use data-sharing delivery between them, according to their DataSharingQosPolicy configuration?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Two entities will be able to use data-sharing delivery between them only if both have at least a common domain.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the default value for the maximum number of Data-sharing domain identifiers, and what does it affect?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> By default, there is no limit to the number of identifiers. The default value can be changed using the max_domains() function.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: What is the consequence of the DataWriter reusing a sample from the pool to publish new data?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> The DataReader loses access to the old data sample.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: Can the communications between entities be sped up?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Yes, but only within the same process using intra-process delivery.
    </div>

.. raw:: html

    <div class="question-box">
        <b>Q: How are peers running in the same process identified?</b>
    </div>
    <div class="answer-box">
        <b>A:</b> Fast DDS utilizes the DomainParticipant's "GuidPrefix_t" to identify peers running in the same process. Two participants with identical 8 first bytes on the "GuidPrefix_t" are considered to be running in the same process.
    </div>


