.. include:: ../includes/aliases.rst

.. _dds_layer_security_access_control_plugin:

Access control plugin
----------------------

The access control plugin provides the mechanisms and operations required for validating the DomainParticipant
permissions.
After a remote DomainParticipant is authenticated, its permissions need to be validated and enforced.

Access rights that each DomainParticipant has over a resource are defined using the access control plugin.
For the proper functioning of a DomainParticipant in a DDS Domain, the DomainParticipant must be authorized to operate
in that specific domain.
The DomainParticipant is responsible for creating the |DataWriters| and |DataReaders| that communicate over a certain
|Topic|.
Hence, a DomainParticipant must have the permissions needed to create a |Topic|, to publish
through its |DataWriters| under defined |Topics|, and to subscribe via its |DataReaders| to other |Topics|.
Access control plugin can configure the Cryptographic plugin as its usage is based on the DomainParticipant's
permissions.

The authentication plugin implemented in Fast DDS is referred to as "DDS\:Access\:Permissions", in compliance with the
`DDS Security <https://www.omg.org/spec/DDS-SECURITY/1.1/>`_ specification.
This plugin is explained in detail below.


.. _access-permissions:

DDS\:Access\:Permissions
^^^^^^^^^^^^^^^^^^^^^^^^

This builtin plugin provides access control using a permissions document signed by a trusted CA.
The DDS\:Access\:Permissions plugin requires three documents for its configuration which contents are explained
in detail below.

   1. The Permissions CA certificate.
   2. The Domain governance signed by the Permissions CA.
   3. The DomainParticipant permissions signed by the Permissions CA.

The DDS\:Access\:Permissions authentication plugin, can be activated setting the |DomainParticipantQos| |Property|
``dds.sec.auth.plugin`` with the value ``builtin.Access-Permissions``.
The following table outlines the properties used for the DDS\:Access\:Permissions plugin configuration.

.. list-table:: **Properties to configure the DDS:Access:Permissions plugin**
   :header-rows: 1
   :align: left

   * - Property name |br|
       (all properties have |br| "dds.sec.access.builtin.Access-Permissions." |br| prefix)
     - Property value
   * - permissions_ca
     - URI to the X509 certificate of the Permissions CA. |br|
       Supported URI schemes: file. |br|
       The file schema shall refer to an X.509 v3 certificate in PEM format.
   * - governance
     - URI to shared Governance Document signed by the Permissions CA |br| in S/MIME format. |br|
       Supported URI schemes: file.
   * - permissions
     - URI to the Participant permissions document signed by the |br| Permissions CA in S/MIME format. |br|
       Supported URI schemes: file.

The following is an example of how to set the properties of |DomainParticipantQoS| for the DDS\:Access\:Permissions
configuration.

+----------------------------------------------------------------------------------------------------------------------+
| **C++**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/DDSCodeTester.cpp                                                                       |
|    :language: c++                                                                                                    |
|    :start-after: // DDS_SECURITY_ACCESS_CONTROL_PLUGIN                                                               |
|    :end-before: //!--                                                                                                |
|    :dedent: 8                                                                                                        |
+----------------------------------------------------------------------------------------------------------------------+
| **XML**                                                                                                              |
+----------------------------------------------------------------------------------------------------------------------+
| .. literalinclude:: /../code/XMLTester.xml                                                                           |
|    :language: xml                                                                                                    |
|    :start-after: <!-->DDS_SECURITY_ACCESS_CONTROL_PLUGIN<-->                                                         |
|    :end-before: <!--><-->                                                                                            |
+----------------------------------------------------------------------------------------------------------------------+


.. _permissions_ca_cert:

Permissions CA Certificate
""""""""""""""""""""""""""

This is an X.509 certificate that contains the Public Key of the CA that will be used to sign the
:ref:`domain_governance_doc` and the :ref:`domainparticipant_permissions_doc`.


.. _domain_governance_doc:

Domain Governance Document
""""""""""""""""""""""""""

Domain Governance document is an XML document that specifies the mechanisms to secure the DDS Domain.
It shall be signed by the Permissions CA in S/MIME format.
The XML scheme of this document is defined in :ref:`domain_governance_xsd_rst`.
The following is an example of the Domain Governance XML file contents.

.. literalinclude:: /../code/GovernanceTester.xml
   :language: xml
   :start-after: <!-->GOVERNANCE_EXAMPLE<-->
   :end-before: <!--><-->
   :linenos:

The `Governance XSD file <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/governance.xsd>`_ and
the
`Governance XML example <https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/SecureHelloWorldExample/certs/governance.xml>`_
can also be downloaded from the `*eProsima Fast DDS* Github repository <https://github.com/eProsima/Fast-RTPS>`_.

.. toctree::
   :hidden:

   governance.rst

Domain Rules
************

It allows the application of rules to the DDS Domain.
Each domain rule is delimited by the ``<domain_rule>`` XML element tag.
The following table summarizes the elements and sections that each domain rule may contain.

+----------------+---------------------------------------+------------------------------------------+------------------+
| **Type**       | **Name**                              | **XML element tag**                      | **Values**       |
+================+=======================================+==========================================+==================+
| Element        | `Domains`_                            | ``<domains>``                            | ``false``        |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``true``         |
|                +---------------------------------------+------------------------------------------+------------------+
|                | `Allow Unauthenticated Participants`_ | ``<allow_unauthenticated_participants>`` | ``false``        |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``true``         |
|                +---------------------------------------+------------------------------------------+------------------+
|                | `Enable Join Access Control`_         | ``<enable_join_access_control>``         | ``SIGN``         |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``ENCRYPT``      |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``NONE``         |
|                +---------------------------------------+------------------------------------------+------------------+
|                | `Discovery Protection Kind`_          | ``<discovery_protection_kind>``          | ``SIGN``         |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``ENCRYPT``      |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``NONE``         |
|                +---------------------------------------+------------------------------------------+------------------+
|                | `Liveliness Protection Kind`_         | ``<liveliness_protection_kind>``         | ``SIGN``         |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``ENCRYPT``      |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``NONE``         |
|                +---------------------------------------+------------------------------------------+------------------+
|                | `RTPS Protection Kind`_               | ``<rtps_protection_kind>``               | ``SIGN``         |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``ENCRYPT``      |
|                |                                       |                                          +------------------+
|                |                                       |                                          | ``NONE``         |
+----------------+---------------------------------------+------------------------------------------+------------------+
| Section        | `Topic Access Rules <topic_rules>`_   | ``<topic_access_rules>``                 | ``<topic_rule>`` |
+----------------+---------------------------------------+------------------------------------------+------------------+


The domain rules are evaluated in the same order as they appear in the document.
A rule only applies to a particular DomainParticipant if the domain section matches the DDS
:cpp:var:`Domain_Id <eprosima::fastdds::dds::DomainId_t>` to which the DomainParticipant belongs.
If multiple rules match, the first rule that matches is the only one that applies.
The following describes the possible configurations of each of the elements and sections listed above that are
contained in the domain rules.

.. _domains_section:

Domains
#######

This element is delimited by the ``<domains>`` XML element tag.
The value in this element identifies the collection of DDS Domains to which the rule applies.
The ``<domains>`` element can contain:

* A single domain identifier:

.. literalinclude:: /../code/GovernanceTester.xml
   :language: xml
   :start-after: <!-->GOVERNANCE_DOMAIN_ID
   :end-before: <!--><-->
   :dedent: 12

* A range of domain identifiers:

.. literalinclude:: /../code/GovernanceTester.xml
   :language: xml
   :start-after: <!-->GOVERNANCE_RANGE_DOMAINS
   :end-before: <!--><-->
   :dedent: 12

Or a combination of both, a list of domain identifiers and ranges of domain identifiers.

Allow Unauthenticated Participants
##################################

This element is delimited by the ``<allow_unauthenticated_participants>`` XML element tag.
It indicates whether the matching of a DomainParticipant with a remote DomainParticipant requires authentication.
The possible values for this element are:

*  ``false``: the DomainParticipant shall enforce the authentication of remote DomainParticipants and
   disallow matching those that cannot be successfully authenticated.
*  ``true``: the DomainParticipant shall allow matching other DomainParticipants (event if the remote DomainParticipant
   cannot authenticate) as long as there is not an already valid authentication with the same DomainParticipant's GUID.

Enable Join Access Control
##########################

This element is delimited by the ``<enable_join_access_control>`` XML element tag.
Indicates whether the matching of the participant with a remote DomqainParticipant requires authorization by the
DDS\:Access\:Permissions plugin.
Its possible values are:

*  ``false``: the DomainParticipant shall not check the permissions of the authenticated remote DomainParticipant.
*  ``true``: the DomainParticipant shall check the permissions of the authenticated remote DomainParticipant.

Discovery Protection Kind
#########################

This element is delimited by the ``<discovery_protection_kind>`` XML element tag.
Indicates whether the secure channel of the endpoint discovery phase needs to be encrypted.
The possible values are:

*  ``SIGN`` or ``ENCRYPT``: the secure channel shall be encrypted.
*  ``NONE``: the secure channel shall not be encrypted.

Liveliness Protection Kind
##########################

This element is delimited by the ``<liveliness_protection_kind>`` XML element tag.
Indicates whether the secure channel of the liveliness mechanism needs to be encrypted.
The possible values are:

*  ``SIGN`` or ``ENCRYPT``: the secure channel shall be encrypted.
*  ``NONE``: the secure channel shall not be encrypted.

RTPS Protection Kind
####################

This element is delimited by the ``<rtps_protection_kind>`` XML element tag.
Indicates whether the whole RTPS Message needs to be encrypted.
The possible values are:

*  ``SIGN`` or ``ENCRYPT``: whole RTPS Messages shall be encrypted.
*  ``NONE``: RTPS Messages shall not be encrypted.

.. _topic_rules:

Topic Rule
##########

This element is delimited by the ``<topic_rule>`` XML element tag and appears within the Topic Access Rules Section
whose XML element tag is ``<topic_access_rules>``.
The following table summarizes the elements and sections that each domain rule may contain.

+---------------------------------------+------------------------------------------+-----------------------------------+
| **Elements**                          | **XML element tag**                      | **Values**                        |
+=======================================+==========================================+===================================+
| `Topic expression`_                   | ``<topic_expression>``                   | Topic name                        |
+---------------------------------------+------------------------------------------+-----------------------------------+
| `Enable Discovery Protection`_        | ``<enable_discovery_protection>``        | ``false``                         |
|                                       |                                          +-----------------------------------+
|                                       |                                          | ``true``                          |
+---------------------------------------+------------------------------------------+-----------------------------------+
| `Enable Liveliness Protection`_       | ``<enable_liveliness_protection>``       | ``false``                         |
|                                       |                                          +-----------------------------------+
|                                       |                                          | ``true``                          |
+---------------------------------------+------------------------------------------+-----------------------------------+
| `Enable Read Access Control`_         | ``<enable_read_access_control>``         | ``false``                         |
|                                       |                                          +-----------------------------------+
|                                       |                                          | ``true``                          |
+---------------------------------------+------------------------------------------+-----------------------------------+
| `Enable Write Access Control`_        | ``<enable_write_access_control>``        | ``false``                         |
|                                       |                                          +-----------------------------------+
|                                       |                                          | ``true``                          |
+---------------------------------------+------------------------------------------+-----------------------------------+
| `Metadata protection Kind`_           | ``<metadata_protection_kind>``           | ``true``                          |
|                                       |                                          +-----------------------------------+
|                                       |                                          | ``false``                         |
+---------------------------------------+------------------------------------------+-----------------------------------+
| `Data protection Kind`_               | ``<data_protection_kind>``               | ``true``                          |
|                                       |                                          +-----------------------------------+
|                                       |                                          | ``false``                         |
+---------------------------------------+------------------------------------------+-----------------------------------+

The topic expression within the rules selects a set of Topic names.
The rule applies to any |DataReader| or |DataWriter| associated with a |Topic| whose name matches the |Topic| expression
name.
The topic access rules are evaluated in the same order as they appear within the ``<topic_access_rules>`` section.
If multiple rules match, the first rule that matches is the only one that applies.

   .. _Topic expression:

*  **Topic expression**

   This element is delimited by the ``<topic_expression>`` XML element tag.
   The value in this element identifies the set of Topic names to which the rule applies.
   The rule applies to any |DataReader| or |DataWriter| associated with a |Topic| whose name matches the value.

   The Topic name expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
   specified in `IEEE 1003.1-2017 <https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html>`_.


.. _Enable Discovery Protection:

*  **Enable Discovery Protection**

   This element is delimited by the ``<enable_discovery_protection>`` XML element tag.
   Indicates whether the entity related discovery information shall go through the secure channel of endpoint discovery
   phase.

   *  ``false``: the entity discovery information shall be sent by an unsecured channel of discovery.
   *  ``true``: the information shall be sent by the secure channel.


.. _Enable Liveliness Protection:

*  **Enable Liveliness Protection**

   This element is delimited by the ``<enable_liveliness_protection>`` XML element tag.
   Indicates whether the entity related liveliness information shall go through the secure channel of liveliness
   mechanism.

   *  ``false``: the entity liveliness information shall be sent by an unsecured channel of liveliness.
   *  ``true``: the information shall be sent by the secure channel.


.. _Enable Read Access Control:

*  **Enable Read Access Control**

   This element is delimited by the ``<enable_read_access_control>`` XML element tag.
   Indicates whether read access to the Topic is protected.

   *  ``false``: then local Subscriber creation and remote Subscriber matching can proceed without further
      access-control mechanisms imposed.
   *  ``true``: they shall be checked using Access control plugin.


.. _Enable Write Access Control:

*  **Enable Write Access Control**

   This element is delimited by the ``<enable_write_access_control>`` XML element tag.
   Indicates whether write access to the Topic is protected.

   *  ``false``: then local Publisher creation and remote Publisher matching can proceed without further
      access-control mechanisms imposed.
   *  ``true``: they shall be checked using Access control plugin.


.. _Metadata Protection Kind:

*  **Metadata Protection Kind**

   This element is delimited by the ``<metadata_protection_kind>`` XML element tag.
   Indicates whether the entity's RTPS submessages shall be encrypted by the Cryptographic plugin.

   *  ``true``: the RTPS submessages shall be encrypted.
   *  ``false``: they shall not.


.. _Data Protection Kind:

*  **Data Protection Kind**

   This element is delimited by the ``<data_protection_kind>`` XML element tag.
   Indicates whether the data payload shall be encrypted by the Cryptographic plugin.

   *  ``true``: the data payload shall be encrypted.
   *  ``false``: the data payload shall not.


.. _domainparticipant_permissions_doc:

DomainParticipant Permissions Document
""""""""""""""""""""""""""""""""""""""

The permissions document is an XML file which contains the permissions of a DomainParticipant and binds them to the
DomainParticipant distinguished name defined in the DDS\:Auth\:PKI-DH plugin.
The permissions document shall be signed by the Permissions CA in S/MIME format.
The XML scheme of this document is defined in :ref:`domainparticipant_permissions_xsd_rst`.
The following is an example of the DomainParticipant Permissions XML file contents.

.. literalinclude:: /../code/PermissionsTester.xml
   :language: xml
   :start-after: <!-->PERMISSIONS_EXAMPLE<-->
   :end-before: <!--><-->
   :linenos:

The `Permissions XSD file <https://github.com/eProsima/Fast-RTPS/blob/master/resources/xsd/governance.xsd>`_ and
the
`Permissions XML example <https://github.com/eProsima/Fast-RTPS/blob/master/examples/C%2B%2B/SecureHelloWorldExample/certs/governance.xml>`_
can also be downloaded from the `*eProsima Fast DDS* Github repository <https://github.com/eProsima/Fast-RTPS>`_.

.. toctree::
   :hidden:

   permissions.rst

Grant Section
*************

This section is delimited by the ``<grant>`` XML element tag.
Each grant section contains three sections:

* Subject name
* Validity
* Rules

Subject name
############

This section is delimited by XML element ``<subject_name>``.
The subject name identifies the DomainParticipant to which the permissions apply.
Each subject name can only appear in a single ``<permissions>`` section within the XML Permissions document.
The contents of the subject name element shall be the X.509 subject name of the DomainParticipant that was given in the
authorization X.509 Certificate.

Validity
########

This section is delimited by the XML element ``<validity>``.
It reflects the valid dates for the permissions.

Rules
#####

This section contains the permissions assigned to the DomainParticipant.
The rules are applied in the same order that appears in the document.
If the criteria for the rule matched the Domain join, publish or subscribe operation that is being attempted,
then the *allow* or *deny* decision is applied.
If the criteria for a rule does not match the operation being attempted, the evaluation shall proceed to the next rule.
If all rules have been examined without a match, then the decision specified by the ``<default>`` rule is applied.
The default rule, if present, must appear after all *allow* and *deny* rules.
If the default rule is not present, the implied default decision is ``DENY``.

For the grant to match there shall be a match of the topics and partitions criteria.

Allow rules are delimited by the XML element ``<allow_rule>``.
Deny rules are delimited by the XML element``<deny_rule>``.
Both contain the same element children.


Domains Section
***************

This section is delimited by the XML element ``<domains>``.
The value in this element identifies the collection of DDS Domains to which the rule applies.
The syntax is the same as for the :ref:`domains_section` of the :ref:`domain_governance_doc`.


Format of the Allowed/Denied Actions sections
*********************************************

The sections for each of the three actions have a similar format.
The only difference is the name of the XML element used to delimit the action:

+---------------------------------------+------------------------------------------------------------------------------+
| **Action**                            | **XML element tag**                                                          |
+=======================================+==============================================================================+
| Allow/Deny Publish                    | ``<publish>``                                                                |
+---------------------------------------+------------------------------------------------------------------------------+
| Allow/Deny Subscribe                  | ``<subscribe>``                                                              |
+---------------------------------------+------------------------------------------------------------------------------+
| Allow/Deny Relay                      | ``<relay>``                                                                  |
+---------------------------------------+------------------------------------------------------------------------------+

Each action contains two conditions.

* Allowed/Denied `Topics Condition`_
* Allowed/Denied `Partitions Condition`_

Topics Condition
################

This section is delimited by the ``<topics>`` XML element.
It defines the Topic names that must be matched for the allow/deny rule to apply.
Topic names may be given explicitly or by means of Topic name expressions.
Each explicit topic name or Topic name expressions appears separately in a ``<topic>`` sub-element within the
``<topics>`` element.

The Topic name expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in

.. literalinclude:: /../code/PermissionsTester.xml
   :language: xml
   :start-after: <!-->PERMISSIONS_TOPIC_CONDITION
   :end-before: <!--><-->

Partitions Condition
####################

This section is delimited by the ``<partitions>`` XML element.
It limits the set Partitions names that may be associated with the (publish, subscribe, relay) action for the rule to
apply.
Partition names expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in `IEEE 1003.1-2017 <https://pubs.opengroup.org/onlinepubs/9699919799/functions/fnmatch.html>`_.
If there is no ``<partitions>`` section within a rule, then the default "empty string" partition is assumed.

.. literalinclude:: /../code/PermissionsTester.xml
   :language: xml
   :start-after: <!-->PERMISSIONS_PARTITION_CONDITION
   :end-before: <!--><-->

Signing documents using x509 certificate
""""""""""""""""""""""""""""""""""""""""

:ref:`domain_governance_doc` and :ref:`domainparticipant_permissions_doc` have to be signed using an X.509 certificate.
Generation of an X.509 certificate is explained in :ref:`generate_x509`.
Next commands sign the necessary documents for its use by the DDS\:Access\:Permissions plugin.

.. code-block:: bash

   # Governance document: governance.xml
   openssl smime -sign -in governance.xml -text -out governance.smime -signer maincacert.pem -inkey maincakey.pem

   # Permissions document: permissions.xml
   openssl smime -sign -in permissions.xml -text -out permissions.smime -signer maincacert.pem -inkey maincakey.pem
