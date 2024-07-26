.. _freq_security_questions:

Security Frequently Asked Questions
===================================

.. raw:: html

   <style>
       .question-box {
           background-color: #e7f3fe;
           border-left: 6px solid #2196F3;
           padding: 10px;
           margin: 10px 0;
       }
       .answer-box {
           background-color: #f9f9f9;
           border-left: 6px solid #4CAF50;
           padding: 10px;
           margin: 10px 0 20px 0;
       }
   </style>

.. raw:: html

   <div class="question-box">
       <b>Q: Why is Fast DDS communication secure?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Because it implements pluggable security at three levels: authentication, access control, and data encryption.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: Is the security support configured by default?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> No. It must be activated using "-DSECURITY=ON" at the CMake configuration step.
   </div>

.. raw:: html

    <h2>Authentication</h2>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of authentication?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> When a DomainParticipant is either locally created or discovered, it needs to be authenticated in order to be able to communicate in a DDS Domain.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What happens if the authentication fails?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The remote DomainParticipant is rejected, therefore communication cannot take place in the DDS Domain for this DomainParticipant.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the DDS:Auth:PKI-DH authentication plugin activated?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> By setting the DomainParticipantQos properties() dds.sec.auth.plugin with the value builtin.PKI-DH.
   </div>

.. raw:: html

    <h2>Access control</h2>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of access control?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Provides the mechanisms and operations required to validate the DomainParticipant permissions and define access rights over a resource.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the DDS:Access:Permissions authentication plugin activated?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> By setting the DomainParticipantQos properties() dds.sec.access.plugin with the value builtin.Access-Permissions.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: Can a DomainParticipant match with a remote DomainParticipant without authentication?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Yes. This can be delimited by the < allow_unauthenticated_participants > XML element tag. When it is set to true, the DomainParticipant can match other DomainParticipants without authentication.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: Can the secure channel of the endpoint discovery phase be encrypted?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Yes, if the < discovery_protection_kind > XML element is set to ENCRYPT. This is also applicable for Liveliness and RTPS.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the access to topics managed?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> By applying topic rules to any DataReader or DataWriter associated with a topic that matches the Topic expression name.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of a DomainParticipant Permissions Document in the DDS:Auth:PKI-DH plugin?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The permissions document is an XML file that contains the permissions of a DomainParticipant and binds them to the DomainParticipant distinguished name defined in the DDS:Auth:PKI-DH plugin.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the main components of a DomainParticipant Permissions document in DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> There are several sections. Grant Section, delimited by the < grant > XML element tag, including the subject name, validity, and rules. Domains sections, delimited by the XML element < domains >, identifying the collection of DDS Domains to which the rule applies. Allowed/Denied Actions sections for publishing, subscribing, relaying, topics, and partitions.
   </div>


.. raw:: html

    <h2>Data encryption</h2>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the function of the cryptographic plugin in the context of DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The cryptographic plugin provides the tools and operations required to support encryption and decryption, digests computation, message authentication codes computation and verification, key generation, and key exchange for DomainParticipants, DataWriters, and DataReaders.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the DDS:Crypto:AES-GCM-GMAC authentication plugin activated?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> By setting the DomainParticipantQos properties() dds.sec.crypto.plugin with the value builtin.AES-GCM-GMAC. Moreover, this plugin needs the activation of the Authentication plugin: DDS:Auth:PKI-DH and the DDS:Access:Permissions.
   </div>

.. raw:: html

    <h2>Logging</h2>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the function of the logging plugin in Fast DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The logging plugin provides the necessary operations to log the security events triggered by the other security plugins supported by Fast DDS.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the DDS:Logging:DDS_LogTopic authentication plugin activated?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> By setting the DomainParticipantQos properties() dds.sec.log.plugin with the value builtin.DDS_LogTopic.
   </div>
