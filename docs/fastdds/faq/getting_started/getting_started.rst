.. _freq_getting_started_questions:

Frequently Asked Getting Started Questions
==========================================

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
       <b>Q: What is DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Data Distribution Service (DDS) is a data-centric communication protocol used for distributed software application communications.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the key entities of a DDS? </b>
   </div>
   <div class="answer-box">
       <b>A: There are four basic entities: Publisher, Subscriber, Topic, Domain.</b> 
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary purpose of using Quality of Service (QoS) in DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> DDS uses QoS to define the behavioral characteristics of DDS Entities. QoS are comprised of individual QoS policies (objects of type deriving from QoSPolicy).
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary responsibility of a DDS Publisher entity?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> <b>Publisher</b>. It is the DCPS entity in charge of the creation and configuration of the <b>DataWriters</b> it implements. The <b>DataWriter</b> is the entity in charge of the actual publication of the messages. Each one will have an assigned <b>Topic</b> under which the messages are published.
   </div>

   
.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary function of a DDS Domain?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> <b>Domain</b>. This is the concept used to link all publishers and subscribers, belonging to one or more applications, which exchange data under different topics. These individual applications that participate in a domain are called <b>DomainParticipant</b>. The DDS Domain is identified by a domain ID.
   </div>

.. raw:: html
    
   <div class="question-box">
       <b>Q: What is RTPS?</b>
   </div>
   <div class="answer-box">
       <b>A: The Real-Time Publish Subscribe (RTPS) protocol, developed to support DDS applications, is a publication-subscription communication middleware over transports such as UDP/IP. </b> 
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What type of communication does RTPS support?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It is designed to support both unicast and multicast communications.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the maximum number of endpoints that a single RTPSParticipant can have?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> A RTPSParticipant can have any number of writer and reader endpoints.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary mechanism by which RTPS participants exchange data, and what role do topics play in this process?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Communication revolves around <b>Topics</b>, which define and label the data being exchanged. The topics do not belong to a specific participant. The participant, through the RTPSWriters, makes changes in the data published under a topic, and through the RTPSReaders receives the data associated with the topics to which it subscribes. The communication unit is called <b>Change</b>, which represents an update in the data that is written under a Topic. <b>RTPSReaders/RTPSWriters</b> register these changes on their <b>History</b>, a data structure that serves as a cache for recent changes.
   </div>
