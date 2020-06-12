The `DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification includes five security builtin plugins.

1.  Authentication plugin: :ref:`DDS\:Auth\:PKI-DH <auth-pki-dh>`.
    This plugin provides authentication for each |DomainParticipant| joining a DDS Domain using a trusted
    *Certificate Authority* (CA).
    Support mutual authentication between |DomainParticipants| and establish a shared secret.
2.  Access Control plugin: :ref:`DDS\:Access\:Permissions <access-permissions>`.
    This plugin provides access control to |DomainParticipants| which perform protected operations.
3.  Cryptographic plugin: :ref:`DDS\:Crypto\:AES-GCM-GMAC <crypto-aes-gcm-gmac>`.
    This plugin provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter Mode
    (AES-GCM).
4.  Logging plugin: :ref:`DDS\:Logging\:DDS_LogTopic <logging-logtopic>`.
    This plugin logs security events.
5.  Data Tagging: DDS\:Tagging\:DDS_Discovery.
    This plugin enables the addition of security labels to the data.
    Thus it is possible to specify classification levels of the data.
    In the DDS context it can be used as a complement to access control, creating an access control based on data
    tagging; for message prioritization; and to prevent its use by the middleware to be used instead by the
    application or service.

.. note::
  Currently the  DDS\:Tagging\:DDS_Discovery plugin is not implemented in Fast DDS.
  Its implementation is expected for future release of Fast DDS.

In compliance with the `DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification, Fast DDS provides
secure communication by implementing pluggable security at three levels: a) |DomainParticipants| authentication
(DDS\:Auth\:PKI-DH), b) access control of Entities (DDS\:Access\:Permissions), and c) data encryption
(DDS\:Crypto\:AES-GCM-GMAC).
Furthermore, for the monitoring of the security plugins and logging relevant events, Fast DDS implements
the logging plugin (DDS\:Logging\:DDS_LogTopic).

By default, Fast DDS does not compile any security support, but it can be activated adding ``-DSECURITY=ON`` at CMake
configuration step.
For more information about Fast DDS compilation, see :ref:`linux_sources` and :ref:`windows_sources`.

Security plugins can be activated through the |DomainParticipantQos| properties.
A |Property| is defined by its name (:class:`std::string`)
and its value (:class:`std::string`).

.. warning::
  For the full understanding of this documentation it is required the user to have basic knowledge of network security
  since terms like Certificate Authority (CA), Public Key Infrastructure (PKI), and Diffie-Hellman encryption protocol
  are not explained in detail.
  However, it is possible to configure basic system security settings, i.e. authentication, access control and
  encryption, to Fast DDS without this knowledge.

The following sections describe how to configure each of these properties to set up the Fast DDS security plugins.
