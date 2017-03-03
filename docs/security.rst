.. |br| raw:: html

   <br />

Security
========

Fast RTPS can be configured to provide secure communications. For this purpose Fast RTPS implements pluggable security
at two levels: authentication of remote participants and encryption of data.

By default Fast RTPS doesn't compile security support. You can activate it adding ``-DSECURITY=ON`` at CMake
configuration step. More information about Fast RTPS compilation, see :ref:`installation-from-sources`.

Built-in plugins
----------------

Current version comes out with two security built-in plugins:

* **Auth:PKI-DH**: this plugin provides authentication using a trusted *Certificate Authority* (CA).
* **Crypto:AES-GCM-GMAC**: this plugin provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter
  Mode (AES-GCM).

.. _auth-pki-dh:

Auth:PKI-DH
^^^^^^^^^^^

This built-in plugin provides authentication between discovered participants. It is supplied by a trusted *Certificate
Authority* (CA) and uses ECDSA Digital Signature Algorithms to perform the mutual authentication. It also establishes a shared
secret using Elliptic Curve Diffie-Hellman (ECDH) Key Agreement Methods. This shared secret can be used by other
security plugins as :ref:`crypto-aes-gcm-gmac`.

You can activate and configure this plugin through :class:`eprosima::fastrtps::Participant` attributes using properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value (:class:`std::string`).
Next tables show you the properties used by this security plugin.

..
.. list-table:: **Property to activate Auth:PKI-DH**
   :header-rows: 1

   * - Property name
     - Property value
   * - dds.sec.auth.plugin
     - builtin.PKI-DH

.. list-table:: **Properties to configure Auth::PKI-DH**
   :header-rows: 1
   :align: left

   * - Property name |br|
       (all properties have "dds.sec.auth.builtin.PKI-DH." prefix)
     - Property value
   * - identity_ca
     - URI to the X509 certificate of the Identity CA. |br|
       Supported URI schemes: file. |br|
       The **file** schema shall refer to a X.509 v3 certificate in PEM format.
   * - identity_certificate
     - URI to a X509 certificate signed by the Identity CA in PEM format containing the signed public key for the Participant. |br|
       Supported URI schemes: file.
   * - identity_crl
     - URI to a X509 Certificate Revocation List (CRL). |br|
       Supported URI schemes: file.
   * - private_key
     - URI to access the private Private Key for the Participant. |br|
       Supported URI schemes: file.
   * - password
     - A password used to decrypt the private_key.

.. _crypto-aes-gcm-gmac:

Crypto:AES-GCM-GMAC
^^^^^^^^^^^^^^^^^^^

This built-in plugin provides authenticated encryption using AES in Galois Counter Mode (AES-GCM).
It also provide additional reader-specific message authentication codes (MACs) using Galois MAC (AES-GMAC).
This plugin needs the activation of the security plugin :ref:`auth-pki-dh`.

You can activate this plugin through :class:`eprosima::fastrtps::Participant` attributes using properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value (:class:`std::string`).
Next table show you the property used by this security plugin.

.. list-table:: **Property to activate Crypto:AES-GCM-GMAC**
   :header-rows: 1

   * - Property name
     - Property value
   * - dds.sec.crypto.plugin
     - builtin.AES-GCM-GMAC

