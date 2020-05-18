.. _dds_layer_security_access_control_plugin:

Access control plugin
----------------------

They provide validation of entities' permissions.
After a remote participant is authenticated, its permissions need to be validated and enforced.

Access rights that each entity has over a resource are described.
Main entity is the Participant and it is used to access or produce information on a Domain;
hence the Participant has to be allowed to run in a certain Domain.
Also, a Participant is responsible for creating Publishers and Subscribers that communicate over a certain Topic.
Hence, a Participant has to have the permissions needed to create a Topic, to publish
through its Publishers certain Topics, and to subscribe via its Subscribers to certain Topics.
Access control plugin can configure the Cryptographic plugin because its usage is based on the Participant's
permissions.

You can activate an Access control plugin using Participant property ``dds.sec.access.plugin``.
Fast RTPS provides a built-in Access control plugin.
More information on :ref:`access-permissions`.


.. _access-permissions:

DDS\:Access\:Permissions
^^^^^^^^^^^^^^^^^^^^^^^^

This built-in plugin provides access control using a permissions document signed by a shared  *Certificate
Authority*. It is configured with three documents:

You can activate this plugin using Participant property ``dds.sec.access.plugin`` with the value
``builtin.Access-Permissions``.
Next table shows the Participant properties used by this security plugin.

.. list-table:: **Properties to configure Access:Permissions**
   :header-rows: 1
   :align: left

   * - Property name :raw-html:`<br />`
       (all properties have "dds.sec.access.builtin.Access-Permissions." prefix)
     - Property value
   * - permissions_ca
     - URI to the X509 certificate of the Permissions CA. :raw-html:`<br />`
       Supported URI schemes: file. :raw-html:`<br />`
       The **file** schema shall refer to an X.509 v3 certificate in PEM format.
   * - governance
     - URI to shared Governance Document signed by the Permissions CA in S/MIME format. :raw-html:`<br />`
       Supported URI schemes: file.
   * - permissions
     - URI to the Participant permissions document signed by the Permissions CA in S/MIME format. :raw-html:`<br />`
       Supported URI schemes: file.

Permissions CA Certificate
""""""""""""""""""""""""""

This is an X.509 certificate that contains the Public Key of the CA that will be used to sign the Domain Governance and
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
A rule only applies to a particular Participant if the domain section matches the domain to which the Participant
belongs.
If multiple rules match, the first rule that matches is the only one that applies.

.. _domains_section:

Domains element
***************

This element is delimited by the XML element ``<domains>``.
The value in this element identifies the collection of Domains values to which the rule applies.

The ``<domains>`` element can contain a single domain identifier, for example:

.. literalinclude:: /../code/GovernanceTester.xml
   :language: xml
   :start-after: <!-->GOVERNANCE_DOMAIN_ID
   :end-before: <!--><-->

Or it can contain a range of domain identifiers, for example:

.. literalinclude:: /../code/GovernanceTester.xml
   :language: xml
   :start-after: <!-->GOVERNANCE_RANGE_DOMAINS
   :end-before: <!--><-->

Or it can contain both, a list of domain identifiers and ranges of domain identifiers.

Allow Unauthenticated Participants element
******************************************

This element is delimited by the XML element ``<allow_unauthenticated_participants>``.
Indicates whether the matching of the Participant with a remote Participant requires authentication.
If the value is ``false``, the Participant shall enforce the authentication of remote Participants and
disallow matching those that cannot be successfully authenticated.
If the value is ``true``, the Participant shall allow matching other Participants (event if the remote Participant
cannot authenticate) as long as there is not an already valid authentication with the same Participant's GUID.

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
XML element is ``<topic_access_rules>``.

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
If the value is ``false``, the entity discovery information shall be sent by an unsecured channel of discovery.
If the value is ``true``, the information shall be sent by the secure channel.

Enable Liveliness Protection element
************************************

This element is delimited by the XML element ``<enable_liveliness_protection>``.
Indicates whether the entity related liveliness information shall go through the secure channel of liveliness mechanism.
If the value is ``false``, the entity liveliness information shall be sent by an unsecured channel of liveliness.
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
Indicates whether the entity's RTPS submessages shall be encrypted by the Cryptographic plugin.
If the value is ``true``, the RTPS submessages shall be encrypted.
If the value is ``false``, they shall not.

Data Protection Kind element
****************************

This element is delimited by the XML element ``<data_protection_kind>``.
Indicates whether the data payload shall be encrypted by the Cryptographic plugin.
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
* Validity
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
The rules are applied in the same order that appears in the document.
If the criteria for the rule matched the Domain join and/or publish or subscribe operation that is being attempted,
then the allow or deny decision is applied.
If the criteria for a rule does not match the operation being attempted, the evaluation shall proceed to the next rule.
If all rules have been examined without a match, then the decision specified by the ``<default>`` rule is applied.
The default rule, if present, must appear after all allow and deny rules.
If the default rule is not present, the implied default decision is DENY.

For the grant to match there shall be a match of the topics and partitions criteria.

Allow rules are delimited by the XML element ``<allow_rule>``. Deny rules are delimited by the XML element
``<deny_rule>``. Both contain the same element children.


Domains Section
***************

This section is delimited by the XML element ``<domains>``.
The value in this element identifies the collection of Domain values to which the rule applies.
The syntax is the same as for the :ref:`domains_section` of the Governance document.

Format of the Allowed/Denied Actions sections
*********************************************

The sections for each of the three action kinds have a similar format.
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

The Topic name expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in *POSIX 1003.1-1992, Section B.6*.

.. literalinclude:: /../code/PermissionsTester.xml
   :language: xml
   :start-after: <!-->PERMISSIONS_TOPIC_CONDITION
   :end-before: <!--><-->

Partitions condition
********************

This section is delimited by the ``<partitions>`` XML element.
It limits the set Partitions names that may be associated with the (publish, subscribe, relay) action for the rule to
apply.
Partition names expression syntax and matching shall use the syntax and rules of the POSIX ``fnmatch()`` function as
specified in *POSIX 1003.2-1992, Section B.6*.
If there is no ``<partitions>`` section within a rule, then the default "empty string" partition is assumed.

.. literalinclude:: /../code/PermissionsTester.xml
   :language: xml
   :start-after: <!-->PERMISSIONS_PARTITION_CONDITION
   :end-before: <!--><-->

Signing documents using x509 certificate
""""""""""""""""""""""""""""""""""""""""

Governance document and Permissions document have to be signed by an X509 certificate.
Generation of an X509 certificate is explained in :ref:`generate_x509`.
Next commands sign the necessary documents for Access:Permissions plugin.

.. code-block:: bash

   # Governance document: governance.xml
   openssl smime -sign -in governance.xml -text -out governance.smime -signer maincacert.pem -inkey maincakey.pem

   # Permissions document: permissions.xml
   openssl smime -sign -in permissions.xml -text -out permissions.smime -signer maincacert.pem -inkey maincakey.pem


