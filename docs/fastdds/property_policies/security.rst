.. include:: ../../03-exports/aliases.include
.. include:: ../../03-exports/aliases-api.include
.. include:: ../../03-exports/roles.include

.. _property_policies_security:

Security Plugins Settings
-------------------------

As described in the :ref:`security` section, the security
plugins admit a set of settings that can be configured.

Authentication plugin settings
******************************

The :ref:`DDS\:Auth\:PKI-DH <auth-pki-dh>` authentication plugin, can be activated setting the |DomainParticipantQos|
|DomainParticipantQos::properties-api|
``dds.sec.auth.plugin`` with the value ``builtin.PKI-DH``.
The following table outlines the properties used for the :ref:`DDS\:Auth\:PKI-DH <auth-pki-dh>` plugin configuration.

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
   * - ``identity_ca``
     - URI to the X.509 v3 certificate of the Identity CA in PEM format. |br|
       Supported URI schemes: file. |br|
   * - ``identity_certificate``
     - URI to an X.509 v3 certificate signed by the Identity CA in PEM format |br|
       containing the signed public key for the Participant. |br|
       Supported URI schemes: file.
   * - ``identity_crl`` *(optional)*
     - URI to a X.509 Certificate Revocation List (CRL). |br|
       Supported URI schemes: file.
   * - ``private_key``
     - URI to access the private Private Key for the Participant. |br|
       Supported URI schemes: file, :ref:`PKCS#11 <pkcs11-support>`.
   * - ``password`` *(optional)*
     - A password used to decrypt the *private_key*.  |br|
       If the *password* property is not present, then the value supplied in the |br|
       *private_key* property must contain the decrypted private key. |br|
       The *password* property is ignored if the *private_key* is given in PKCS#11 scheme.
   * - ``preferred_key_agreement`` *(optional)*
     - The preferred algorithm to use for generating the session's shared secret |br|
       at the end of the authentication phase. Supported values are: |br|
       a) ``DH``, ``DH+MODP-2048-256`` for  Diffie-Hellman Ephemeral with 2048-bit MODP Group parameters. |br|
       b) ``ECDH``, ``ECDH+prime256v1-CEUM`` for Elliptic Curve Diffie-Hellman Ephemeral with the NIST P-256 curve. |br|
       c) ``AUTO`` for selecting the key agreement based on the signature algorithm in the Identity CA's certificate. |br|
       Will default to ``AUTO`` if the property is not present.
   * - ``transmit_algorithms_as_legacy`` *(optional)*
     - Whether to transmit algorithm identifiers in non-standard legacy format. |br|
       Will default to ``false`` if the property is not present.

.. note::
  All properties listed above have the ``dds.sec.auth.builtin.PKI-DH."`` prefix.
  For example: ``dds.sec.auth.builtin.PKI-DH.identity_ca``. For examples
  and further information, please refer to the :ref:`auth-pki-dh` section.

Authentication handshake settings
*********************************

The authentication phase starts when discovery information is received
from the remote |DomainParticipants|. At this moment, the participant sends
a handshake request until a handshake response is received from the remote participant.
Some parameters are involved in the behavior of this exchange:

* ``max_handshake_requests`` controls the maximum number of handshake requests to be sent.

* ``initial_handshake_resend_period`` represents the initial waiting time (in milliseconds)
  for the first handshake request that has to be resent.

* ``handshake_resend_period_gain`` is the gain against which the period is multiplied
  between two handshake requests.

Hence, the period of time to wait for sending a new handshake request is computed at each
iteration as the period between the last two handshake requests multiplied by the gain
(so that the period increases).

The following table lists the
settings to configure the authentication handshake behavior within
the ``dds.sec.auth.builtin.PKI-DH`` plugin:

.. list-table::
   :header-rows: 1
   :align: left

   * - PropertyPolicyQos name
     - PropertyPolicyQos value
     - PropertyPolicyQos bounds
     - Default value
   * - ``max_handshake_requests``
     - ``<int>``
     - ``[1, max)``
     - ``10``
   * - ``initial_handshake_resend_period``
     - ``<int>``
     - ``[1, max)``
     - ``125``
   * - ``handshake_resend_period_gain``
     - ``<double>``
     - ``(1.0, max)``
     - ``1.5``

.. note::

    All listed properties have the ``dds.sec.auth.builtin.PKI-DH.`` prefix.
    For example: ``dds.sec.auth.builtin.PKI-DH.max_handshake_requests``.

The following is an example of how to set the properties of DomainParticipantQoS for the
authentication handshake configuration.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // DDS_SECURITY_AUTH_HANDSHAKE_PROPS                                                                |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DDS_SECURITY_AUTH_HANDSHAKE_PROPS<-->                                                          |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+

Cryptographic plugin settings
*****************************

The :ref:`DDS\:Crypto\:AES-GCM-GMAC <crypto-aes-gcm-gmac>` authentication plugin,
can be activated setting the |DomainParticipantQos| |DomainParticipantQos::properties-api|
``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``.
Moreover, this plugin needs the activation of the :ref:`auth-pki-dh`.
The :ref:`DDS\:Crypto\:AES-GCM-GMAC <crypto-aes-gcm-gmac>` plugin is configured using the
:ref:`access-permissions`, i.e the cryptography plugin is configured through the properties
and configuration files of the access control plugin.
For further information and examples in this regard please refer to :ref:`crypto-aes-gcm-gmac`.

Logging plugin settings
***********************
The :ref:`DDS\:Logging\:DDS_LogTopic <logging-logtopic>` authentication plugin,
can be activated setting the |DomainParticipantQos| |DomainParticipantQos::properties-api|
``dds.sec.log.plugin`` with the value ``builtin.DDS_LogTopic``.
The following table outlines the properties used for the DDS\:Logging\:DDS_LogTopic plugin configuration.
For further information and examples follow the dedicated documentation: :ref:`logging-logtopic`.
