.. _freq_xml_profiles_questions:

XML PROFILES Frequently Asked Questions
=======================================


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
       <b>Q: How are XML profiles defined?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The XML profiles are defined within the < dds > element, and in turn, within the < profiles > XML elements.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What happens if an XML file is loaded multiple times?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> If the same XML profile file is loaded multiple times, the second loading of the file will result in an error together with the consequent error log.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is stand-alone profiles definition?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The element defining the XML profile is the root element of the XML file.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is root profile definition?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The element defining the XML profile is the child element of another element.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What do DomainParticipant profiles do?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The DomainParticipant profiles allow the definition of the configuration of DomainParticipants through XML files. These profiles are defined within the < participant > XML tags.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the domain participant XML attributes?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The <participant> element has two attributes defined: profile_name and is_default_profile.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the DomainParticipant configured?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The <participant> element has two child elements: <domainId> and <rtps>. All the DomainParticipant configuration options belong to the <rtps> element, except for the DDS DomainId which is defined by the <domainId> element.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What do DataWriter profiles do?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The DataWriter profiles allow for configuring DataWriters from an XML file. These profiles are defined within the < data_writer > XML tags.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the XML attributes of the DataWriter?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The <data_writer> element has two attributes defined: profile_name and is_default_profile.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What do DataReader profiles do?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The DataReader profiles allow declaring DataReaders from an XML file. These profiles are defined within the < data_reader > XML tags.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the XML attributes of the DataReader?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The <data_reader> element has two attributes defined: profile_name and is_default_profile.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What do Topic profiles do?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The <topic> element has two attributes defined: profile_name and is_default_profile.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the available XML elements for configuring transport layer parameters in Fast DDS, and what is their purpose?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> This section defines the XML elements available for configuring the transport layer parameters in *Fast DDS*. These elements are defined within the XML tag "< transports_descriptors >". The "< transport_descriptors >" can contain one or more "< transport_descriptor >" XML elements. Each "< transport_descriptor >" element defines a configuration for a specific type of transport protocol.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: Are log profiles available?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> eProsima Fast DDS allows for registering and configuring Log consumers using XML configuration files. The logging profiles are defined within the <log> XML tags. The <log> element has two child elements: <use_default> and <consumer>.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the advantage of having Dynamic Language Binding support in XML files?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The topic data types can be modified without the need to modify the source code of the DDS application.
   </div>