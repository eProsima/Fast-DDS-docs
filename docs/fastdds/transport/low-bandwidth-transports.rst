.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

Low Bandwidth Transports |Pro|
==============================

The Low Bandwidth Transports feature in Fast DDS Pro enables efficient communication over constrained networks by
reducing the size of network packets.
This optimization is particularly beneficial for environments with limited bandwidth, such as satellite links or
tactical radios, allowing Fast DDS applications to operate reliably and efficiently in such scenarios.

This feature provides two specialized transports:

- **PayloadCompressionTransport**: Compresses the message payload before transmitting it over the network.
  By reducing the amount of data sent, it minimizes bandwidth usage and can improve throughput and reduce latency on
  bandwidth-constrained links.
- **HeaderReductionTransport**: Minimizes the size of the RTPS protocol headers in each packet.
  By reducing header overhead, it further optimizes total packet size and ensures efficient communication,
  especially for small and frequent messages.

Both transports are implemented using the feature :ref:`transport_transportApi_chaining`, therefore both can be
configured independently or together, providing flexibility to adapt Fast DDS to the requirements of low bandwidth
environments.


PayloadCompressionTransport
---------------------------

This low bandwidth transport performs a standard compression of data before sending and the corresponding decompression
after receiving.
The compression algorithm may be selected between Zlib and Bzip2, or even let the transport perform both and use the one
ahich produces the shortest output.

This transport can be configured using the |DomainParticipantQos| |DomainParticipantQos::properties-api|
``rtps.payload_compression``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Property name
     - Property value
   * - ``compression_library``
     - Default compression library for all packets. |br|
       **Values**: ZLIB, BZIP2 or AUTOMATIC
   * - ``compression_level``
     - Default compression level for all packets. |br|
       **Values**: 1 to 9
   * - ``compression_library.small_packets``
     - Compression library for small packets. |br|
       **Values**: ZLIB, BZIP2 or AUTOMATIC
   * - ``compression_level.small_packets``
     - Compression level for small packets. |br|
       **Values**: 1 to 9
   * - ``compression_library.medium_packets``
     - Compression library for medium packets. |br|
       **Values**: ZLIB, BZIP2 or AUTOMATIC
   * - ``compression_level.medium_packets``
     - Compression level for medium packets. |br|
       **Values**: 1 to 9
   * - ``compression_library.large_packets``
     - Compression library for large packets. |br|
       **Values**: ZLIB, BZIP2 or AUTOMATIC
   * - ``compression_level.large_packets``
     - Compression level for large packets. |br|
       **Values**: 1 to 9
   * - ``low_mark``
     - Maximum size for a packet to be considered small.
   * - ``high_mark``
     - Minimum size for a packet to be considered large.

.. _transport_transportApi_payloadcompression_descriptor:

PayloadCompressionTransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

|PayloadCompressionTransportDescriptor-api| has no additional data members from the common ones described in
:ref:`transport_transportApi_chaining_descriptor`.


Enabling PayloadCompressionTransport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To enable a new :class:`PayloadCompressionTransport` in a :ref:`dds_layer_domainParticipant`, first
create an instance of :ref:`transport_transportApi_payloadcompression_descriptor`,
and add it to the user transport list of the :ref:`dds_layer_domainParticipant`.

The examples below show this procedure in both C++ code and XML file.

.. tab-set-code::

    .. literalinclude:: /../code/ProDDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-PAYLOAD-COMPRESSION-TRANSPORT
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/ProXMLTester.xml
        :language: xml
        :start-after: <!-->CONF-PAYLOAD-COMPRESSION-TRANSPORT
        :end-before: <!--><-->
        :lines: 2-4,6-33,35-36

HeaderReductionTransport
------------------------

This transport performs a specific compression of data before sending and the corresponding decompression after
receiving.
The compression algorithm is specific for the RTPS protocol.
It will remove certain headers while compressing others.

This transport can be configured using the |DomainParticipantQos| |DomainParticipantQos::properties-api|
``rtps.header_reduction``.

.. list-table::
   :header-rows: 1
   :align: left

   * - Property name
     - Property value
   * - ``remove_protocol``
     - Removes the ProtocolId, which identifies the message as an RTPS message, from the RTPS header. |br|
       **Values**: true or false
   * - ``remove_version``
     - Removes the ProtocolVersion, which identifies the version of the RTPS protocol, from the RTPS header. |br|
       **Values**: true or false
   * - ``remove_vendor_id``
     - Removes the VendorId, which indicates the vendor that provides the implementation of the RTPS
       protocol, from the RTPS header. |br|
       **Values**: true or false
   * - ``compress_guid_prefix``
     - Compresses the GuidPrefix, which is a unique identifier for each participant in the RTPS
       protocol, from three 32-bit words to a smaller size. |br|
       **Values**: Three numbers between 0 and 32, which tells the number of bits each GuidPrefix world must be
       reduced. |br|
       **Example**: ``8,8,16``
   * - ``submessage.combine_id_and_flags``
     - Combines the SubmessageId and the SubmessageFlags into a single byte in the Submessage header. |br|
       **Values**: true or false
   * - ``submessage.remove_extra_flags``
     - Removes the ExtraFlags field from the Submessage header. |br|
       **Values**: true or false
   * - ``submessage.compress_entitiy_ids``
     - Compresses the EntityId fields in the Submessage header from two 32-bit words to a smaller size. |br|
       **Values**: Two numbers between 0 and 32, which tells the number of bits each EntityId must be reduced. |br|
       **Example**: ``16,16``
   * - ``submessage.compress_sequence_number``
     - Compresses the SequenceNumber field in the Submessage header from a 64-bit word to a smaller size. |br|
       **Values**: A number between 0 and 64, which tells the number of bits the SequenceNumber must be reduced. |br|
       **Example**: ``32``

.. _transport_transportApi_headerreduction_descriptor:

HeaderReductionTransportDescriptor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

|HeaderReductionTransportDescriptor-api| has no additional data members from the common ones described in
:ref:`transport_transportApi_chaining_descriptor`.


Enabling HeaderReductionTransport
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To enable a new :class:`HeaderReductionTransport` in a :ref:`dds_layer_domainParticipant`, first
create an instance of :ref:`transport_transportApi_headerreduction_descriptor`,
and add it to the user transport list of the :ref:`dds_layer_domainParticipant`.

The examples below show this procedure in both C++ code and XML file.

.. tab-set-code::

    .. literalinclude:: /../code/ProDDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-HEADER-REDUCTION-TRANSPORT
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/ProXMLTester.xml
        :language: xml
        :start-after: <!-->CONF-HEADER-REDUCTION-TRANSPORT
        :end-before: <!--><-->
        :lines: 2-4,6-45,47-48

Full example
------------

This example shows how to combine all low bandwidth transports to ensure optimal performance in constrained networks.
This procedure can be configured in C++ code or XML file.

.. tab-set-code::

    .. literalinclude:: /../code/ProDDSCodeTester.cpp
        :language: c++
        :start-after: //CONF-LOW-BANDWIDTH-TRANSPORT
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/ProXMLTester.xml
        :language: xml
        :start-after: <!-->CONF-LOW-BANDWIDTH-TRANSPORT
        :end-before: <!--><-->
        :lines: 2-4,6-54,56-57
