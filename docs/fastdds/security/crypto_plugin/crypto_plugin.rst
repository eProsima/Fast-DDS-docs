.. include:: ../includes/aliases.rst

.. _dds_layer_security_crypto_plugin:

Cryptographic plugin
---------------------

The cryptographic plugin provides the tools and operations required to support encryption and decryption,
digests computation, message authentication codes computation and verification, key generation, and key exchange for
DomainParticipants, DataWriters and DataReaders.
Encryption can be applied over three different levels of DDS protocol:

* The whole RTPS messages.
* The RTPS submessages of a specific DDS Entity (DataWriter or DataReader).
* The payload (user data) of a particular DataWriter.

The authentication plugin implemented in Fast DDS is referred to as "DDS\:Crypto\:AES-GCM-GMAC", in compliance with the
`DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification.
This plugin is explained in detail below.

.. _crypto-aes-gcm-gmac:

DDS\:Crypto\:AES-GCM-GMAC
^^^^^^^^^^^^^^^^^^^^^^^^^

The DDS\:Crypto\:AES-GCM-GMAC plugin provides authentication encryption using Advanced Encryption Standard (AES) in
Galois Counter Mode (`AES-GCM <https://csrc.nist.gov/publications/detail/sp/800-38d/final>`_).
It supports 128 bits and 256 bits AES key sizes.
It may also provide additional DataReader-specific Message Authentication Codes (MACs) using Galois MAC
(`AES-GMAC <https://csrc.nist.gov/publications/detail/sp/800-38d/final>`_).

The DDS\:Crypto\:AES-GCM-GMAC authentication plugin, can be activated setting the |DomainParticipantQos| |Property|
``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``.
Moreover, this plugin needs the activation of the :ref:`auth-pki-dh` plugin.
The DDS\:Crypto\:\AES-GCM-GMAC plugin is configured using the :ref:`access-permissions` plugin, i.e the cryptography
plugin is configured through the properties and configuration files of the access control plugin.
If the :ref:`access-permissions` plugin will not be used, you can configure the DDS\:Crypto\:AES-GCM-GMAC plugin
manually with the properties outlined in the following table.

+------------------------------------------+-------------------------------------------------+-------------------------+
| **Property name**                        | **Description**                                 | **Property Value**      |
+==========================================+=================================================+=========================+
| rtps.participant.rtps_protection_kind    | Encrypt whole RTPS messages                     | ``ENCRYPT``             |
+------------------------------------------+-------------------------------------------------+-------------------------+
| rtps.endpoint.submessage_protection_kind | Encrypt RTPS submessages of a particular entity | ``ENCRYPT``             |
+------------------------------------------+-------------------------------------------------+-------------------------+
| rtps.endpoint.payload_protection_kind    | Encrypt payload of a particular Writer          | ``ENCRYPT``             |
+------------------------------------------+-------------------------------------------------+-------------------------+

The following is an example of how to set the properties of |DomainParticipantQoS| for the DDS\:Crypto\:AES-GCM-GMAC
configuration.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // DDS_SECURITY_CRYPTO_PLUGIN_DOMAINPARTICIPANT                                                     |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DDS_SECURITY_CRYPTO_PLUGIN_DOMAINPARTICIPANT<-->                                               |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

Next example shows how to configure DataWriters to encrypt their RTPS submessages and the RTPS message payload, i.e.
the user data.
This is done by setting the DDS\:Crypto\:AES-GCM-GMAC properties corresponding to the DataWriters in the
|DataWriterQos|.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // DDS_SECURITY_CRYPTO_PLUGIN_DATAWRITER                                                            |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DDS_SECURITY_CRYPTO_PLUGIN_DATAWRITER<-->                                                      |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+


The last example shows how to configure DataReader to encrypt their RTPS submessages.
This is done by setting the DDS\:Crypto\:AES-GCM-GMAC properties corresponding to the DataReaders in the
|DataReaderQos|.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // DDS_SECURITY_CRYPTO_PLUGIN_DATAREADER                                                            |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DDS_SECURITY_CRYPTO_PLUGIN_DATAREADER<-->                                                      |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+
