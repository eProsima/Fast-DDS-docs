.. include:: ../includes/aliases.rst

.. _auth-pki-dh:


Authentication plugin: DDS:\Auth\:PKI-DH
----------------------------------------

This is the starting point for all the security mechanisms.
The authentication plugin provides the mechanisms and operations required for |DomainParticipants| authentication at
discovery.
If the security module was activated at Fast DDS compilation, when a |DomainParticipant| is either locally created or
discovered, it needs to be authenticated in order to be able to
communicate in a DDS Domain.
Therefore, when a |DomainParticipant| detects a remote |DomainParticipant|, both try to authenticate themselves using
the activated authentication plugin.
If the authentication process finishes successfully both |DomainParticipant| match and the discovery mechanism
continues.
On failure, the remote |DomainParticipant| is rejected.

The authentication plugin implemented in Fast DDS is referred to as "DDS:\Auth\:PKI-DH", in compliance with the
`DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification.
The DDS:\Auth\:PKI-DH plugin uses a trusted *Certificate Authority* (CA) and the ECDSA
Digital Signature Algorithms to perform the mutual authentication.
It also establishes a shared secret using Elliptic Curve Diffie-Hellman (ECDH) Key Agreement Methods.
This shared secret can be used by other security plugins as :ref:`crypto-aes-gcm-gmac`.

The DDS:\Auth\:PKI-DH authentication plugin, can be activated setting the |DomainParticipantQos| |Property|
``dds.sec.auth.plugin`` with the value ``builtin.PKI-DH``.
The following table outlines the properties used for the DDS:\Auth\:PKI-DH plugin configuration.

.. list-table::
   :header-rows: 1
   :align: left

   * - Property name
     - Property value
   * - identity_ca
     - URI to the X.509 v3 certificate of the Identity CA in PEM format. |br|
       Supported URI schemes: file. |br|
   * - identity_certificate
     - URI to an X.509 v3 certificate signed by the Identity CA in PEM format |br|
       containing the signed public key for the Participant. |br|
       Supported URI schemes: file.
   * - identity_crl *(optional)*
     - URI to a X.509 Certificate Revocation List (CRL). |br|
       Supported URI schemes: file.
   * - private_key
     - URI to access the private Private Key for the Participant. |br|
       Supported URI schemes: file.
   * - password *(optional)*
     - A password used to decrypt the *private_key*.  |br|
       If the *password* property is not present, then the value supplied in the |br|
       *private_key* property must contain the decrypted private key.

.. note::
  All listed properties have "dds.sec.auth.builtin.PKI-DH." prefix.
  For example: ``dds.sec.auth.builtin.PKI-DH.identity_ca``.

The following is an example of how to set the properties of |DomainParticipantQoS| for the DDS:\Auth\:PKI-DH plugin
configuration.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // DDS_SECURITY_AUTH_PLUGIN                                                                         |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DDS_SECURITY_AUTH_PLUGIN<-->                                                                   |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+


.. _generate_x509:

Generation of X.509 certificates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

An X.509 digital certificate is a document that has been encrypted and/or digitally signed according to
`RFC 5280 <https://tools.ietf.org/html/rfc5280>`_.
The X.509 certificate refers to the Public Key Infrastructure (PKI) certificate of the `IETF <https://ietf.org/>`_ ,
and specifies the standard
formats for public-key certificates and a certification route validation algorithm.
A simple way to generate these certificates for a proprietary PKI structure is through the
`OpenSSL <https://www.openssl.org/>`_ toolkit.
This section explains how to build a certificate infrastructure from the trusted CA certificate to the end-entity
certificate, i.e. the |DomainParticipant|.

Generating the CA certificate for self-signing
""""""""""""""""""""""""""""""""""""""""""""""

First, since multiple certificates will need to be issued, one for each of the |DomainParticipants|, a dedicated CA is
set up, and the CA's certificate is installed as the root key of all |DomainParticipants|.
Thus, the |DomainParticipants| will accept all certificates issued by our own CA.
To create a proprietary CA certificate, a configuration file must first be written with the CA information.
An example of the CA configuration file is shown below.
The OpenSSL commands shown in this example are compatible with both Linux and Windows Operating Systems (OS).
However, all other commands are only compatible with Linux OS.

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

After writing the configuration file, next commands generate the certificate using the
Elliptic Curve Digital Signature Algorithm (ECDSA).

.. code-block:: bash

    openssl ecparam -name prime256v1 > ecdsaparam

    openssl req -nodes -x509 \
      -days 3650 \
      -newkey ec:ecdsaparam \
      -keyout maincakey.pem \
      -out maincacert.pem \
      -config maincaconf.cnf

Generating the DomainParticipant certificate
""""""""""""""""""""""""""""""""""""""""""""

As was done for the CA, a |DomainParticipant| certificate configuration file needs to be created first.

.. code-block:: ini

    # File: partconf.cnf

    prompt = no
    string_mask = utf8only
    distinguished_name = req_distinguished_name

    [ req_distinguished_name ]
    countryName = ES
    stateOrProvinceName = MA
    localityName = Tres Cantos
    organizationName = eProsima
    emailAddress = example@eprosima.com
    commonName = DomainParticipantName

After writing the |DomainParticipant| certificate configuration file, next commands generate the X.509 certificate,
using ECDSA, for a |DomainParticipant|.

.. code-block:: bash

    openssl ecparam -name prime256v1 > ecdsaparam

    openssl req -nodes -new \
      -newkey ec:ecdsaparam \
      -config partconf.cnf \
      -keyout partkey.pem \
      -out partreq.pem

    openssl ca -batch -create_serial \
      -config maincaconf.cnf \
      -days 3650 \
      -in partreq.pem \
      -out partcert.pem

Generating the Certificate Revocation List (CRL)
""""""""""""""""""""""""""""""""""""""""""""""""

Finally, the CRL is created.
This is a list of the X.509 certificates revoked by the certificate issuing CA before they reach their expiration date.
Any certificate that is on this list will no longer be trusted.
To create a CRL using OpenSSL just run the following commands.

.. code-block:: bash

  echo -ne '00' > crlnumber

  openssl ca -gencrl \
    -config maincaconf.cnf \
    -cert maincacert.pem \
    -keyfile maincakey.pem \
    -out crl.pem

As an example, below is shown how to add the X.509 certificate of a |DomainParticipant| to the CRL.

.. code-block:: bash

  openssl ca \
    -config maincaconf.cnf \
    -cert maincacert.pem \
    -keyfile maincakey.pem \
    -revoke partcert.pem

  openssl ca -gencrl \
    -config maincaconf.cnf \
    -cert maincacert.pem \
    -keyfile maincakey.pem \
    -out crl.pem
