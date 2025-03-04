.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _crypto-aes-gcm-gmac:

Cryptographic plugin: DDS\:Crypto\:AES-GCM-GMAC
-----------------------------------------------

The cryptographic plugin provides the tools and operations required to support encryption and decryption,
digests computation, message authentication codes computation and verification, key generation, and key exchange for
DomainParticipants, |DataWriters| and |DataReaders|.
Encryption can be applied over three different levels of DDS protocol:

* The whole RTPS messages.
* The RTPS submessages of a specific DDS Entity (DataWriter or DataReader).
* The payload (user data) of a particular DataWriter.

The authentication plugin implemented in Fast DDS is referred to as "DDS\:Crypto\:AES-GCM-GMAC", in compliance with the
`DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification.
This plugin is explained in detail below.

The DDS\:Crypto\:AES-GCM-GMAC plugin provides authentication encryption using Advanced Encryption Standard (AES) in
Galois Counter Mode (`AES-GCM <https://csrc.nist.gov/publications/detail/sp/800-38d/final>`_).
It supports 128 bits and 256 bits AES key sizes.
It may also provide additional DataReader-specific Message Authentication Codes (MACs) using Galois MAC
(`AES-GMAC <https://csrc.nist.gov/publications/detail/sp/800-38d/final>`_).

The DDS\:Crypto\:AES-GCM-GMAC authentication plugin, can be activated setting the |DomainParticipantQos|
|DomainParticipantQos::properties-api|
``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``.
Moreover, this plugin needs the activation of the :ref:`auth-pki-dh`.
The DDS\:Crypto\:\AES-GCM-GMAC plugin is configured using the :ref:`access-permissions`, i.e the cryptography
plugin is configured through the properties and configuration files of the access control plugin.

The following is an example of how to set the properties of DomainParticipantQoS for the DDS\:Crypto\:AES-GCM-GMAC
configuration.

.. tab-set-code::

    .. literalinclude:: /../code/DDSCodeTester.cpp
        :language: c++
        :start-after: // DDS_SECURITY_CRYPTO_PLUGIN_DOMAINPARTICIPANT
        :end-before: //!--
        :dedent: 8

    .. literalinclude:: /../code/XMLTester.xml
        :language: xml
        :start-after: <!-->DDS_SECURITY_CRYPTO_PLUGIN_DOMAINPARTICIPANT<-->
        :end-before: <!--><-->
