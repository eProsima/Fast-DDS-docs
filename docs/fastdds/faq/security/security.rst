.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include
.. include:: ../../../03-exports/roles.include

.. _freq_security_questions:

Security Frequently Asked Questions
===================================


 .. collapse::  Why is Fast DDS communication secure?




    |br|

    Because it implements pluggable security at three levels: authentication, access control, and data encryption. For further information, see :ref:`security`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  Is the security support configured by default?




    |br|

    No. It must be activated using ``-DSECURITY=ON`` at the CMake configuration step. For further information, see :ref:`security`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Authentication
--------------

 .. collapse::  What is the purpose of authentication?




    |br|

    When a |DomainParticipant-api| is either locally created or discovered, it needs to be authenticated in order to be able to communicate in a DDS Domain. For further information, see :ref:`auth-pki-dh`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What happens if the authentication fails?




    |br|

    The remote |DomainParticipant-api| is rejected, therefore communication cannot take place in the DDS Domain for this DomainParticipant. For further information, see :ref:`auth-pki-dh`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Auth:PKI-DH authentication plugin activated?




    |br|

    By setting the |DomainParticipantQos::properties-api| ``dds.sec.auth.plugin`` with the value`` ``builtin.PKI-DH``. For further information, see :ref:`auth-pki-dh`.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


Access control
--------------

 .. collapse::  What is the purpose of access control?




    |br|

    Provides the mechanisms and operations required to validate the DomainParticipant permissions and define access rights over a resource. For further information, see :ref:`access-permissions`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Access:Permissions authentication plugin activated?




    |br|

    By setting the |DomainParticipantQos::properties-api| ``dds.sec.access.plugin`` with the value ``builtin.Access-Permissions``. For further information, see :ref:`access-permissions`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  Can a DomainParticipant match with a remote DomainParticipant without authentication?




    |br|

    Yes. This can be delimited by the ``<allow_unauthenticated_participants>`` XML element tag. When it is set to true, the DomainParticipant can match other DomainParticipants without authentication. For further information, see :ref:`allow_unauthenticated_section`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  Can the secure channel of the endpoint discovery phase be encrypted?




    |br|

    Yes, if the ``<discovery_protection_kind>`` XML element is set to ENCRYPT. This is also applicable for Liveliness and RTPS. For further information, see :ref:`access-permissions`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the access to topics managed?




    |br|

    By applying topic rules to any DataReader or DataWriter associated with a topic that matches the |Topic-api| expression name. For further information, see :ref:`topic_rules`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What is the purpose of a DomainParticipant Permissions Document in the DDS:Auth:PKI-DH plugin?




    |br|

    The permissions document is an XML file that contains the permissions of a DomainParticipant and binds them to the DomainParticipant distinguished name defined in the ``DDS:Auth:PKI-DH`` plugin. For further information, see :ref:`domainparticipant_permissions_doc`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What are the main components of a DomainParticipant Permissions document in DDS?




    |br|

    There are several sections. Grant Section, delimited by the ``<grant>`` XML element tag, including the subject name, validity, and rules. Domains sections, delimited by the XML element ``<domains>``, identifying the collection of DDS Domains to which the rule applies. Allowed/Denied Actions sections for publishing, subscribing, relaying, topics, and partitions. For further information, see :ref:`domainparticipant_permissions_doc`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


Data encryption
---------------

 .. collapse::  What is the function of the cryptographic plugin in the context of DDS?




    |br|

    The cryptographic plugin provides the tools and operations required to support encryption and decryption, digests computation, message authentication codes computation and verification, key generation, and key exchange for DomainParticipants, DataWriters, and DataReaders. For further information, see :ref:`crypto-aes-gcm-gmac`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Crypto:AES-GCM-GMAC authentication plugin activated?




    |br|

    By setting the |DomainParticipantQos::properties-api| ``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``. Moreover, this plugin needs the activation of the Authentication plugin: ``DDS:Auth:PKI-DH`` and the ``DDS:Access:Permissions``. For further information, see :ref:`crypto-aes-gcm-gmac`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Logging
-------

 .. collapse::  What is the function of the logging plugin in Fast DDS?




    |br|

    The logging plugin provides the necessary operations to log the security events triggered by the other security plugins supported by Fast DDS. For further information, see :ref:`logging-logtopic`.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Logging:DDS_LogTopic authentication plugin activated?




    |br|

    By setting the |DomainParticipantQos::properties-api| ``dds.sec.log.plugin`` with the value ``builtin.DDS_LogTopic``. For further information, see :ref:`logging-logtopic`.

|
