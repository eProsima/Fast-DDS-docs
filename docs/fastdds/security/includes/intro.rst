The `DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification includes five security builtin plugins.

1.  Authentication plugin: DDS\:Auth\:PKI-DH.
2.  Access Control plugin: DDS\:Access\:Permissions.
3.  Cryptographic plugin: DDS\:Crypto\:AES-GCM-GMAC.
4.  Logging plugin: DDS\:Logging\:DDS_LogTopic.
5.  Data Tagging: DDS\:Tagging\:DDS_Discovery.

In compliance with the `DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification, Fast DDS provides
secure communication by implementing pluggable security at three levels: a) DomainParticipants authentication
(DDS\:Auth\:PKI-DH), b) access control of Entities (DDS\:Access\:Permissions), and c) data encryption
(DDS\:Crypto\:AES-GCM-GMAC).
Furthermore, for the monitoring of the security plugins and logging relevant events, Fast DDS implements
the logging plugin (DDS\:Logging\:DDS_LogTopic).
Therefore, the current version of the API comes out with four security builtin plugins:

* :ref:`auth-pki-dh`: this plugin provides authentication for each DomainParticipant joining a DDS
  Domain using a trusted *Certificate Authority* (CA). Support mutual authentication between DomainParticipants and
  establish a shared secret.
* :ref:`access-permissions`: this plugin provides access control to DomainParticipants which
  perform protected operations.
* :ref:`crypto-aes-gcm-gmac`: this plugin provides authenticated encryption using Advanced Encryption
  Standard (AES) in Galois Counter Mode (AES-GCM).
* :ref:`logging-logtopic`: this plugin logs security events.

By default, Fast DDS does not compile any security support, but it can be activated adding ``-DSECURITY=ON`` at CMake
configuration step.
For more information about Fast DDS compilation, see :ref:`installation-from-sources`.

Security plugins can be activated through the :class:`DomainParticipantQos`
properties.
A :class:`Property` is defined by its name (:class:`std::string`) and its value
(:class:`std::string`).
This section describes how to configure each of these properties to set up the Fast DDS security plugins.
