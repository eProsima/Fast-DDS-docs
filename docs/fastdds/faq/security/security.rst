.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_security_questions:

Security Frequently Asked Questions
===================================


 .. collapse::  Why is Fast DDS communication secure?




    :Answer:

    Because it implements pluggable security at three levels: authentication, access control, and data encryption.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  Is the security support configured by default?




    :Answer:

    No. It must be activated using ``-DSECURITY=ON`` at the CMake configuration step.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Authentication
--------------

 .. collapse::  What is the purpose of authentication?




    :Answer:

    When a DomainParticipant is either locally created or discovered, it needs to be authenticated in order to be able to communicate in a DDS Domain.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What happens if the authentication fails?




    :Answer:

    The remote DomainParticipant is rejected, therefore communication cannot take place in the DDS Domain for this DomainParticipant.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Auth:PKI-DH authentication plugin activated?




    :Answer:

    By setting the ``DomainParticipantQos`` ``properties()`` ``dds.sec.auth.plugin with the value`` ``builtin.PKI-DH``.


-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


Access control
--------------

 .. collapse::  What is the purpose of access control?




    :Answer:

    Provides the mechanisms and operations required to validate the DomainParticipant permissions and define access rights over a resource.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Access:Permissions authentication plugin activated?




    :Answer:

    By setting the ``DomainParticipantQos`` ``properties()`` ``dds.sec.access.plugin`` with the value ``builtin.Access-Permissions``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  Can a DomainParticipant match with a remote DomainParticipant without authentication?




    :Answer:

    Yes. This can be delimited by the ``<allow_unauthenticated_participants>`` XML element tag. When it is set to true, the DomainParticipant can match other DomainParticipants without authentication.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  Can the secure channel of the endpoint discovery phase be encrypted?




    :Answer:

    Yes, if the ``<discovery_protection_kind>`` XML element is set to ENCRYPT. This is also applicable for Liveliness and RTPS.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the access to topics managed?




    :Answer:

    By applying topic rules to any DataReader or DataWriter associated with a topic that matches the Topic expression name.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What is the purpose of a DomainParticipant Permissions Document in the DDS:Auth:PKI-DH plugin?




    :Answer:

    The permissions document is an XML file that contains the permissions of a DomainParticipant and binds them to the DomainParticipant distinguished name defined in the ``DDS:Auth:PKI-DH`` plugin.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What are the main components of a DomainParticipant Permissions document in DDS?




    :Answer:

    There are several sections. Grant Section, delimited by the ``<grant>`` XML element tag, including the subject name, validity, and rules. Domains sections, delimited by the XML element ``<domains>``, identifying the collection of DDS Domains to which the rule applies. Allowed/Denied Actions sections for publishing, subscribing, relaying, topics, and partitions.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


Data encryption
---------------

 .. collapse::  What is the function of the cryptographic plugin in the context of DDS?




    :Answer:

    The cryptographic plugin provides the tools and operations required to support encryption and decryption, digests computation, message authentication codes computation and verification, key generation, and key exchange for DomainParticipants, DataWriters, and DataReaders.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Crypto:AES-GCM-GMAC authentication plugin activated?




    :Answer:

    By setting the ``DomainParticipantQos`` ``properties()`` ``dds.sec.crypto.plugin`` with the value ``builtin.AES-GCM-GMAC``. Moreover, this plugin needs the activation of the Authentication plugin: ``DDS:Auth:PKI-DH`` and the ``DDS:Access:Permissions``.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

Logging
-------

 .. collapse::  What is the function of the logging plugin in Fast DDS?




    :Answer:

    The logging plugin provides the necessary operations to log the security events triggered by the other security plugins supported by Fast DDS.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How is the DDS:Logging:DDS_LogTopic authentication plugin activated?




    :Answer:

    By setting the ``DomainParticipantQos`` ``properties()`` ``dds.sec.log.plugin`` with the value ``builtin.DDS_LogTopic``.

|
