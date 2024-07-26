.. _freq_rtps_layer_questions:

RTPS LAYER Frequently Asked Questions
=====================================

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
       <b>Q: What is the primary function of the RTPS Layer in <i>eprosima Fast DDS</i>, and how does it differ from the DDS Layer?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The lower level RTPS Layer of <i>eprosima Fast DDS</i> serves as an implementation of the protocol defined in the RTPS standard. This layer provides more control over the internals of the communication protocol than the DDS Layer, so advanced users have finer control over the library's functionalities.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary responsibility of an "RTPSParticipant" in the context of the RTPS Layer?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> As the RTPS standard specifies, "RTPSWriters" and "RTPSReaders" are always associated with a "History" element. In the DDS Layer, its creation and management is hidden, but in the RTPS Layer, you have full control over its creation and configuration.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of creating a "WriterHistory" when creating an RTPS Writer?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Writers are created with "RTPSDomain::createRTPSWriter()" and configured with a "WriterAttributes" structure. They also need a "WriterHistory" which is configured with a "HistoryAttributes" structure.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of using the "History" element in the RTPS Layer?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> In the RTPS Protocol, Readers and Writers save the data about a topic in their associated Histories. Each piece of data is represented by a Change, which <i>eprosima Fast DDS</i> implements as "CacheChange_t". Changes are always managed by the History.
   </div>
