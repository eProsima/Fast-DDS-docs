.. |br| raw:: html

   <br />

.. _security:

Security
========

Fast RTPS can be configured to provide secure communications. For this purpose Fast RTPS implements pluggable security
at three levels: authentication of remote participants, access control of entities and encryption of data.

By default Fast RTPS doesn't compile security support. You can activate it adding ``-DSECURITY=ON`` at CMake
configuration step. For more information about Fast RTPS compilation, see :ref:`installation-from-sources`.

You can activate and configure security plugins through :class:`eprosima::fastrtps::Participant` attributes using properties.
A :class:`eprosima::fastrtps::rtps::Property` is defined by its name (:class:`std::string`) and its value (:class:`std::string`).
Throughout this page there are tables showing you the properties used by each security plugin.

Authentication plugins
----------------------

They provide authentication on discovery of remote participants. When a remote participant is detected, Fast RTPS tries
to authenticate using the activated Authentication plugin. If the authentication process finishes successfully then both
participants match and discovery protocol continues. On failure, the remote participant is rejected.

You can activate an Authentication plugin using Participant property ``dds.sec.auth.plugin``. Fast RTPS provides a
built-in Authentication plugin. More information on :ref:`auth-pki-dh`.

Access control plugins
----------------------

They provide validation of entities' permissions.
After a remote participant is authenticated, its permissions need to be validated and enforced.

Access rights that each entity has over a resource are described.
Main entity is the Participant and it is used to access or produce information on a Domain;
hence the Participant has to be allowed to run in a certain Domain.
Also a Participant is responsible for creating Publishers and Subscribers that communicate over a certain Topic.
Hence, a Participant has to have the permissions needed to create a Topic, to publish
through its Publishers certain Topics, and to subscribe via its Subscribers to certain Topics.
Access control plugin can configure the Cryptographic plugin because its usage is based on the Participant's
permissions.

You can activate an Access control plugin using Participant property ``dds.sec.access.plugin``.
Fast RTPS provides a built-in Access control plugin.
More information on :ref:`access-permissions`.

Cryptographic plugins
---------------------

They provide encryption support. Encryption can be applied over three different levels of RTPS protocol. Cryptographic
plugins can encrypt whole RTPS messages, RTPS submessages of a particular entity (Writer or Reader) or the payload
(user data) of a particular Writer. You can combine them.

You can activate an Cryptographic plugin using Participant property ``dds.sec.crypto.plugin``. Fast RTPS provides a
built-in Cryptographic plugin. More information on :ref:`crypto-aes-gcm-gmac`.

The Cryptographic plugin is configured by the Access control plugin.
If Access control will not be used, you can configure Cryptographic plugin manually with next properties:

**Encrypt whole RTPS messages**

You can configure a Participant to encrypt all RTPS messages using Participant property ``rtps.participant.rtps_protection_kind``
with the value ``ENCRYPT``.

**Encrypt RTPS submessages of a particular entity**

You can configure an entity (Writer or Reader) to encrypt its RTPS submessages using Entity property ``rtps.endpoint.submessage_protection_kind``
with the value ``ENCRYPT``.

**Encrypt payload of a particular Writer**

You can configure a Writer to encrypt its payload using Writer property ``rtps.endpoint.payload_protection_kind`` with
the value ``ENCRYPT``.

Built-in plugins
----------------

Current version comes out with three security built-in plugins:

* :ref:`auth-pki-dh`: this plugin provides authentication using a trusted *Certificate Authority* (CA).
* :ref:`access-permissions`: this plugin provides access control to Participants at the Domain and Topic level.
* :ref:`crypto-aes-gcm-gmac`: this plugin provides authenticated encryption using Advanced Encryption Standard (AES) in Galois Counter
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

.. list-table:: **Properties to configure Auth:PKI-DH**
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

.. _generate_x509:

Generation of x509 certificates
"""""""""""""""""""""""""""""""

You can generate you own x509 certificates using OpenSSL application. This section teaches you how to do this.

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

.. _access-permissions:

Access:Permissions
^^^^^^^^^^^^^^^^^^

This built-in plugin provides access control using a permissions document signed by a shared  *Certificate
Authority*. It is configured with three documents:

You can activate this plugin using Participant property ``dds.sec.access.plugin`` with the value
``builtin.Access-Permissions``.
Next table shows the Participant properties used by this security plugin.

.. list-table:: **Properties to configure Access:Permissions**
   :header-rows: 1
   :align: left

   * - Property name |br|
       (all properties have "dds.sec.access.builtin.Access-Permissions." prefix)
     - Property value
   * - permissions_ca
     - URI to the X509 certificate of the Permissions CA. |br|
       Supported URI schemes: file. |br|
       The **file** schema shall refer to a X.509 v3 certificate in PEM format.
   * - governance
     - URI to shared Governance Document signed by the Permissions CA in S/MIME format. |br|
       Supported URI schemes: file.
   * - permissions
     - URI to the Participant permissions document signed by the Permissions CA in S/MIME format. |br|
       Supported URI schemes: file.

Permissions CA Certificate
""""""""""""""""""""""""""

This is a X.509 certificate that contains the Public Key of the CA that will be used to sign the Domain Governance and
Domain Permissions documents.

Domain Governance Document
""""""""""""""""""""""""""

Domain Governance document is an XML document that specifies how the domain should be secured.
It shall be signed by the Permissions CA in S/MIME format.

The format of this document is defined in this `Governance XSD file`_. You can also find a `generic Governance XML
example`_.

.. _Governance XSD file: https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/governance.xsd
.. _generic Governance XML example: https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/SecureHelloWorldExample/certs/governance.xml


Domain Rules
************

Each domain rule is delimited by the ``<domain_rule>`` XML element tag. Each domain rule contains the following elements
and sections:

* Domains element
* Allow Unauthenticated Participants element
* Enable Join Access Control element
* Discovery Protection Kind element
* Liveliness Protection Kind element
* RTPS Protection Kind element
* Topic Access Rules section

The domain rules are evaluated in the same order as they appear in the document.
A rule only applies to a particular Participant if the domain section matches the domain to which the Participant belongs.
If multiple rules match, the first rule that matches is the only one that applies.

.. _domains_section:

Domains element
***************

This element is delimited by the XML element ``<domains>``.
The value in this element identifies the collection of Domains values to which the rule applies.

The ``<domains>`` element can contain a single domain identifier, for example:

.. code-block:: xml

   <domains>
       <id>1</id>
   </domain>

Or it can contain a range of domain identifiers, for example:

.. code-block:: xml

   <domains>
       <id_range>
           <min>1</min>
           <max>10</max>
       </id_range>
   </domain>

Or it can contain both, a list of domain identifiers and ranges of domain identifiers.

Allow Unauthenticated Participants element
******************************************

This element is delimited by the XML element ``<allow_unauthenticated_participants>``.
Indicates whether the matching of the Participant with a remote Participant requieres authentication.
If the value is ``false``, the Participant shall enforce the authentication of remote Participants and
disallow matching those that cannot be successfully authenticated.
If the value is ``true``, the Participant shall allow matching other Participants (event if the remote Participant cannot
authenticate) as long as there is not an already valid authentication with the same Participant's GUID.

Enable Join Access Control element
**********************************

This element is delimited by the XML element ``<enable_join_access_control>``.
Indicates whether the matching of the participant with a remote Participant requires authorization by the Access control
plugin.
If the value is ``false``, the Participant shall not check the permissions of the authenticated remote Participant.
If the value is ``true``, the Participant shall check the permissions of the authenticated remote Participant.

Discovery Protection Kind element
*********************************

This element is delimited by the XML element ``<discovery_protection_kind>``.
Indicates whether the secure channel of the endpoint discovery phase needs to be encrypted.
If the value is ``SIGN`` or ``ENCRYPT``, the secure channel shall be encrypted.
If the value is ``NONE``, it shall not.

Liveliness Protection Kind element
**********************************

This element is delimited by the XML element ``<liveliness_protection_kind>``.
Indicates whether the secure channel of the liveliness mechanism needs to be encrypted.
If the value is ``SIGN`` or ``ENCRYPT``, the secure channel shall be encrypted.
If the value is ``NONE``, it shall not.

RTPS Protection Kind element
****************************

This element is delimited by the XML element ``<rtps_protection_kind>``.
Indicates whether the whole RTPS Message needs to be encrypted. If the value is ``SIGN`` or ``ENCRYPT``, whole RTPS
Messages shall be encrypted.
If the value is ``NONE``, it shall not.

Topic Rule Section
******************

This element is delimited by the XML element ``<topic_rule>`` and appears within the Topic Access Rules Section whose
XML elmement is ``<topic_access_rules>``.

Each one contains the following elements:

* Topic expression
* Enable Discovery protection
* Enable Liveliness protection
* Enable Read Access Control element
* Enable Write Access Control element
* Metadata protection Kind
* Data protection Kind

The topic expression selects a set of Topic names.
The rule applies to any Publisher or Subscriber associated with a Topic whose name matches the Topic expression name.

The topic access rules are evaluated in the same order as they appear within the ``<topic_access_rules>`` section.
If multiple rules match, the first rule that matches is the only one that applies.

Topic expression element
************************

This element is delimited by the XML element ``<topic_expression>``.
The value in this element identifies the set of Topic names to which the rule applies.
The rule will apply to any Publisher and Subscriber associated with a Topic whose name matches the value.

The Topic name expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in *POSIX 1003.2-1992, Section B.6*.

Enable Discovery protection element
***********************************

This element is delimited by the XML element ``<enable_discovery_protection>``.
Indicates whether the entity related discovery information shall go through the secure channel of endpoint discovery
phase.
If the value is ``false``, the entity discovery information shall be sent by unsecure channel of discovery.
If the value is ``true``, the information shall be sent by the secure channel.

Enable Liveliness Protection element
************************************

This element is delimited by the XML element ``<enable_liveliness_protection>``.
Indicates whether the entity related liveliness information shall go through the secure channel of liveliness mechanism.
If the value is ``false``, the entity liveliness information shall be sent by unsecure channel of liveliness.
If the value is ``true``, the information shall be sent by the secure channel.

Enable Read Access Control element
**********************************

This element is delimited by the XML element ``<enable_read_access_control>``.
Indicates whether read access to the Topic is protected.
If the value is ``false``, then local Subscriber creation and remote Subscriber matching can proceed without further
access-control mechanisms imposed.
If the value is ``true``, they shall be checked using Access control plugin.

Enable Write Access Control element
***********************************

This element is delimited by the XML element ``<enable_write_access_control>``.
Indicates whether write access to the Topic is protected.
If the value is ``false``, then local Publisher creation and remote Publisher matching can proceed without further
access-control mechanisms imposed.
If the value is ``true``, they shall be checked using Access control plugin.

Metadata Protection Kind element
********************************

This element is delimited by the XML element ``<metadata_protection_kind>``.
Indicates whether entity's RTPS submessages shall be encrypted by Cryptographic plugin.
If the value is ``true``, the RTPS submessages shall be encrypted.
If the value is ``false``, they shall not.

Data Protection Kind element
****************************

This element is delimited by the XML element ``<data_protection_kind>``.
Indicates whether the data payload shall be encrypted by Cryptogpraphic plugin.
If the value is ``true``, the data payload shall be encrypted.
If the value is ``false``, the data payload shall not.

Participant permissions document
""""""""""""""""""""""""""""""""

The permissions document is an XML document containing the permissions of the Participant and binding them to its
distinguished name.
The permissions document shall be signed by the Permissions CA in S/MIME format.


The format of this document is defined in this `Permissions XSD file`_. You can also find a `generic Permissions XML
example`_.

.. _Permissions XSD file: https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/permissions.xsd
.. _generic Permissions XML example: https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/SecureHelloWorldExample/certs/permissions.xml

Grant Section
*************

This section is delimited by the ``<grant>`` XML element tag.
Each grant section contains three sections:

* Subject name
* Validaty
* Rules

Subject name
************

This section is delimited by XML element ``<subject_name>``.
The subject name identifies the Participant to which the permissions apply.
Each subject name can only appear in a single ``<permissions>`` section within the XML Permissions document.
The contents of the subject name element  shall be the x.509 subject name for the Participant as is given in the
Authorization Certificate.

Validity
********

This section is delimited by the XML element ``<validity>``.
It reflects the valid dates for the permissions.

Rules
*****

This section contains the permissions assigned to the Participant.
The rules are applied in the same order that appear in the document.
If the criteria for the rule matched the Domain join and/or publish or subscribe operation that is being attempted,
then the allow or deny decision is applied.
If the criteria for a rule does not match the operation being attempted, the evaluation shall proceed to the next rule.
If all rules have been examined without a match, then the decision specified by the ``<default>`` rule is applied.
The default rule, if present, must appear after all allow and deny rules.
If the default rule is not present, the implied default decision is DENY.

For the grant to match there shall be a match of the topics and partitions criteria.

Allow rules are delimited by the XML element ``<allow_rule>``. Deny rules are delimited by the XML element
``<deny_rule>``. Both contains the same element children.


Domains Section
***************

This section is delimited by the XML element ``<domains>``.
The value in this element identifies the collection of Domain values to which the rule applies.
The syntax is the same as for the :ref:`domains_section` of the Governance document.

Format of the Allowed/Denied Actions sections
*********************************************

The sections for each of the three action kinds have similar format.
The only difference is the name of the XML element used to delimit the action:

* The Allow/Deny Publish Action is delimited by the ``<publish>`` XML element.
* The Allow/Deny Subscribe Action is delimited by the ``<subscribe>`` XML element.
* The Allow/Deny Relay Action is delimited by the ``<relay>`` XML element.

Each action contains two conditions.

* Allowed/Denied Topics Condition
* Allowed/Denied Partitions Condition

Topics condition
****************

This section is delimited by the ``<topics>`` XML element.
It defines the Topic names that must be matched for the allow/deny rule to apply.
Topic names may be given explicitly or by means of Topic name expressions.
Each topic name of topic-name expressions appears separately in a ``<topic>`` sub-element within the ``<topics>``
element.

The Topic name expressión syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in *POSIX 1003.1-1992, Section B.6*.

.. code-block:: xml

   <topics>
       <topic>Plane<topic>
       <topic>Hel*<topic>
   <topics>

Partitions condition
********************

This section is delimited by the ``<partitions>`` XML element.
It limits the set Partitions names that may be associated with the (publish, subscribe, relay) action for the rule to
apply.
Partition names expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in *POSIX 1003.2-1992, Section B.6*.
If there is no ``<partitions>`` section within a rule, then the default "empty string" partition is assumed.

.. code-block:: xml

   <partitions>
       <partition>A</partition>
       <partition>B*</partition>
   </partitions>

Signing documents using x509 certificate
""""""""""""""""""""""""""""""""""""""""

Governance document and Permissions document have to be signed by a X509 certificate.
Generation of a X509 certificate is explained in :ref:`generate_x509`.
Next commands sign the necessary documents for Access:Permissions plugin.

.. code-block:: bash

   # Governance document: governance.xml
   openssl smime -sign -in governance.xml -text -out governance.smime -signer maincacert.pem -inkey maincakey.pem

   # Permissions document: permissions.xml
   openssl smime -sign -in permissions.xml -text -out permissions.smime -signer maincacert.pem -inkey maincakey.pem



.. _crypto-aes-gcm-gmac:

Crypto:AES-GCM-GMAC
^^^^^^^^^^^^^^^^^^^

This built-in plugin provides authenticated encryption using AES in Galois Counter Mode (AES-GCM).
It also provides additional reader-specific message authentication codes (MACs) using Galois MAC (AES-GMAC).
This plugin needs the activation of the security plugin :ref:`auth-pki-dh`.

You can activate this plugin using Participant property ``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``.

.. Como generar los ficheros PEM

Example: configuring the :class:`Participant`
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This example show you how to configure a Participant to activate and configure :ref:`auth-pki-dh`,
:ref:`access-permissions` and :ref:`crypto-aes-gcm-gmac` plugins.

**Participant attributes**

.. code-block:: c++

   eprosima::fastrtps::ParticipantAttributes part_attr;

   // Activate Auth:PKI-DH plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.plugin", "builtin.PKI-DH");

   // Configure Auth:PKI-DH plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_ca", "maincacert.pem");
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.identity_certificate", "appcert.pem");
   part_attr.rtps.properties.properties().emplace_back("dds.sec.auth.builtin.PKI-DH.private_key", "appkey.pem");

   // Activate Access:Permissions plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.access.plugin", "builtin.Access-Permissions");

   // Configure Access:Permissions plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.access.builtin.Access-Permissions.permissions_ca",
       "maincacet.pem");
   part_attr.rtps.properties.properties().emplace_back("dds.sec.access.builtin.Access-Permissions.governance",
       "governance.xml");
   part_attr.rtps.properties.properties().emplace_back("dds.sec.access.builtin.Access-Permissions.permissions",
       "permissions.xml");

   // Activate Crypto:AES-GCM-GMAC plugin
   part_attr.rtps.properties.properties().emplace_back("dds.sec.crypto.plugin", "builtin.AES-GCM-GMAC");

This example show you how to configure a Participant to activate and configure :ref:`auth-pki-dh` and
:ref:`crypto-aes-gcm-gmac` plugins, without and Access control plugin.
It also configures Participant to encrypt its RTPS messages, Writer and Reader to encrypt their RTPS submessages and
Writer to encrypt the payload (user data).

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
