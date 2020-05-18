.. _dds_layer_security_auth_plugin:

Authentication plugin
----------------------

They provide authentication on the discovery of remote participants.
When a remote participant is detected, Fast RTPS tries to authenticate using the activated Authentication plugin.
If the authentication process finishes successfully then both participants match and discovery protocol continues.
On failure, the remote participant is rejected.

You can activate an Authentication plugin using Participant property ``dds.sec.auth.plugin``. Fast RTPS provides a
built-in Authentication plugin. More information on :ref:`auth-pki-dh`.


.. _auth-pki-dh:

DDS:\Auth\:PKI-DH
^^^^^^^^^^^^^^^^^

This built-in plugin provides authentication between discovered participants.
It is supplied by a trusted *Certificate Authority* (CA) and uses ECDSA Digital Signature Algorithms to perform the
mutual authentication.
It also establishes a shared secret using Elliptic Curve Diffie-Hellman (ECDH) Key Agreement Methods.
This shared secret can be used by other security plugins as :ref:`crypto-aes-gcm-gmac`.

You can activate this plugin using Participant property ``dds.sec.auth.plugin`` with the value ``builtin.PKI-DH``.
Next tables show you the Participant properties used by this security plugin.

.. list-table:: **Properties to configure Auth:PKI-DH**
   :header-rows: 1
   :align: left

   * - Property name :raw-html:`<br />`
       (all properties have "dds.sec.auth.builtin.PKI-DH." prefix)
     - Property value
   * - identity_ca
     - URI to the X509 certificate of the Identity CA. :raw-html:`<br />`
       Supported URI schemes: file. :raw-html:`<br />`
       The **file** schema shall refer to a X.509 v3 certificate in PEM format.
   * - identity_certificate
     - URI to an X509 certificate signed by the Identity CA in PEM format containing the signed public key for the
       Participant. :raw-html:`<br />`
       Supported URI schemes: file.
   * - identity_crl *(optional)*
     - URI to a X509 Certificate Revocation List (CRL). :raw-html:`<br />`
       Supported URI schemes: file.
   * - private_key
     - URI to access the private Private Key for the Participant. :raw-html:`<br />`
       Supported URI schemes: file.
   * - password *(optional)*
     - A password used to decrypt the private_key.

.. _generate_x509:

Generation of x509 certificates
"""""""""""""""""""""""""""""""

You can generate your own x509 certificates using OpenSSL application. This section teaches you how to do this.

**Generate a certificate for the CA**

When you want to create your own CA certificate, you first have to write a configuration file with your CA
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

When you want to create your own certificate for your Participant, you first have to write a configuration file.

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

