.. |br| raw:: html

   <br />

.. _security:

Security
========

Fast RTPS can be configured to provide secure communications. For this purpose Fast RTPS implements pluggable security
at two levels: authentication of remote participants and encryption of data.

By default Fast RTPS doesn't compile security support. You can activate it adding ``-DSECURITY=ON`` at CMake
configuration step. More information about Fast RTPS compilation, see :ref:`installation-from-sources`.

You can activate and configure security plugins through :class:`eprosima::fastrtps::Participant` attributes using properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value (:class:`std::string`).
Throughout this page there are tables showing you the properties used by each security plugin.

Authentication plugins
----------------------

They provide authentication on discovery of remote participants. When a remote participant is detected, Fast RTPS tries
to authenticate using the activated Authentication plugin. If the authentication process finishes successfully then both
participants matches and discovery protocol continues. On failure, the remote participant is rejected.

You can activate an Authentication plugin using Participant property ``dds.sec.auth.plugin``. Fast RTPS provides a
built-in Authentication plugin. More information on :ref:`auth-pki-dh`.

Cryptographic plugins
---------------------

They provide encryption support. Encryption can be applied over three different levels of RTPS protocol. Cryptographic
plugins can encrypt whole RTPS messages, RTPS submessages of a particular entity (Writer or Reader) or the payload
(user data) of a particular Writer. You can combine them.

You can activate an Cryptographic plugin using Participant property ``dds.sec.crypto.plugin``. Fast RTPS provides a
built-in Cryptographic plugin. More information on :ref:`crypto-aes-gcm-gmac`.

**Encrypt whole RTPS messages**

You can configure a Participant to encrypt all RTPS messages using Participant property ``rtps.participant.rtps_protection_kind``
with the value ``ENCRYPT``.

**Encrypt RTPS submessages of a particular entity**

You can configure an entity (Writer or Reader) to encrypt its RTPS submessages using Entity property ``rtps.endpoint.submessage_protection_kind``
with the value ``ÈNCRYPT``.

**Encrypt payload of a particular Writer**

You can configure a Writer to encrypt its payload using Writer property ``rtps.endpoint.payload_protection_kind`` with
the value ``ENCRYPT``.

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

You can activate this plugin using Participant property ``dds.sec.auth.plugin`` with the value ``builtin.PKI-DH``.
Next tables show you the Participant properties used by this security plugin.

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
   * - identity_crl *(optional)*
     - URI to a X509 Certificate Revocation List (CRL). |br|
       Supported URI schemes: file.
   * - private_key
     - URI to access the private Private Key for the Participant. |br|
       Supported URI schemes: file.
   * - password *(optional)*
     - A password used to decrypt the private_key.

Generation of x509 certificates
*******************************

You can generate you own x509 certificates using OpenSSL application. This section teaches you how to do this.

**Generate a certificate for the CA**

Wether you want to create your own CA certificate, first you have to write a configuration file with your CA
information.

.. code-block:: ini

    # File: maincaconf.cnf
    # OpenSSL example Certificate Authority configuration file

    ####################################################################
    [ ca ]
    default_ca = CA_default # The default ca section

    ####################################################################
    [ CA_default ]

    dir = . # Where everything is kept
    certs = $dir/certs # Where the issued certs are kept
    crl_dir = $dir/crl # Where the issued crl are kept
    database = $dir/index.txt # database index file.
    unique_subject = no # Set to 'no' to allow creation of
                        # several ctificates with same subject.
    new_certs_dir = $dir

    certificate = $dir/maincacert.pem # The CA certificate
    serial = $dir/serial # The current serial number
    crlnumber = $dir/crlnumber # the current crl number
                               # must be commented out to leave a V1 CRL
    crl = $dir/crl.pem # The current CRL
    private_key = $dir/maincakey.pem # The private key
    RANDFILE = $dir/private/.rand # private random number file

    name_opt = ca_default # Subject Name options
    cert_opt = ca_default # Certificate field options

    default_days= 1825 # how long to certify for
    default_crl_days = 30 # how long before next CRL
    default_md = sha256 # which md to use.
    preserve = no # keep passed DN ordering

    policy = policy_match

    # For the CA policy
    [ policy_match ]
    countryName = match
    stateOrProvinceName = match
    organizationName = match
    organizationalUnitName = optional
    commonName = supplied
    emailAddress = optional

    # For the 'anything' policy
    # At this point in time, you must list all acceptable 'object'
    # types.
    [ policy_anything ]
    countryName = optional
    stateOrProvinceName = optional
    localityName = optional
    organizationName = optional
    organizationalUnitName = optional
    commonName = supplied
    emailAddress = optional

    [ req ]
    prompt = no
    #default_bits = 1024
    #default_keyfile = privkey.pem
    distinguished_name= req_distinguished_name
    #attributes = req_attributes
    #x509_extensions = v3_ca # The extentions to add to the self signed cert
    string_mask = utf8only

    [ req_distinguished_name ]
    countryName = ES
    stateOrProvinceName = MA
    localityName = Tres Cantos
    0.organizationName = eProsima
    commonName = eProsima Main Test CA
    emailAddress = mainca@eprosima.com

After writing the configuration file, next commands generate the certificate using ECDSA.

.. code-block:: bash

    openssl ecparam -name prime256v1 > ecdsaparam

    openssl req -nodes -x509 -days 3650 -newkey ec:ecdsaparam -keyout maincakey.pem -out maincacert.pem -config maincaconf.cnf

**Generate a certificate for the Participant**

Wether you want to create your own certificate for your Participant, first you have to write a configuration file.

.. code-block:: ini

    # File: appconf.cnf

    prompt = no
    string_mask = utf8only
    distinguished_name = req_distinguished_name

    [ req_distinguished_name ]
    countryName = ES
    stateOrProvinceName = MA
    localityName = Tres Cantos
    organizationName = eProsima
    emailAddress = example@eprosima.com
    commonName = AppName

After writing the configuration file, next commands generate the certificate, using ECDSA, for your Participant.

.. code-block:: bash

    openssl ecparam -name prime256v1 > ecdsaparam

    openssl req -nodes -new -newkey ec:ecdsaparam -config appconf.cnf -keyout appkey.pem -out appreq.pem

    openssl ca -batch -create_serial -config maincaconf.cnf -days 3650 -in appreq.pem -out appcert.pem

.. _crypto-aes-gcm-gmac:

Crypto:AES-GCM-GMAC
^^^^^^^^^^^^^^^^^^^

This built-in plugin provides authenticated encryption using AES in Galois Counter Mode (AES-GCM).
It also provide additional reader-specific message authentication codes (MACs) using Galois MAC (AES-GMAC).
This plugin needs the activation of the security plugin :ref:`auth-pki-dh`.

You can activate this plugin using Participant property ``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``.

.. Como generar los ficheros PEM

Example
^^^^^^^

This example show you how to configure a Participant to activate and configure :ref:`auth-pki-dh` and
:ref:`crypto-aes-gcm-gmac` plugins. Also it configures Participant to encrypt its RTPS messages, Writer and Reader to
encrypt their RTPS submessages and Writer to encrypt the payload (user data).

**Participant attributes**

.. code-block:: c++

   eprosima::fastrtps::ParticipantAttributes part_attr;

   // Activate Auth:PKI-DH plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");

   // Configure Auth:PKI-DH plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca", "maincacert.pem");
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate", "appcert.pem");
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key", "appkey.pem");

   // Activate Crypto:AES-GCM-GMAC plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.crypto.plugin", "builtin.AES-GCM-GMAC");

   // Encrypt all RTPS submessages
   part_attr.rtps.properties.properties().emplace_back("rtps.participant.rtps_protection_kind", "ENCRYPT");

**Writer attributes**

.. code-block:: c++

   eprosima::fastrtps::PublisherAttributes pub_attr;

   // Encrypt RTPS submessages
   pub_attr.properties.properties().emplace_back("rtps.endpoint.submessage_protection_kind", "ENCRYPT");

   // Encrypt payload
   pub_attr.properties.properties().emplace_back("rtps.endpoint.payload_protection_kind", "ENCRYPT");

**Reader attributes**

.. code-block:: c++

   eprosima::fastrtps::SubscriberAttributes sub_attr;

   // Encrypt RTPS submessages
   sub_attr.properties.properties().emplace_back("rtps.endpoint.submessage_protection_kind", "ENCRYPT");
