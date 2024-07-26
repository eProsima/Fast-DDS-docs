.. _freq_statistics_module_questions:

Statistics Module Frequently Asked Questions
============================================

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
       <b>Q: What is the purpose of the statistics module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Fast DDS Statistics module is an extension of Fast DDS that enables the recollection of data concerning the DDS communication.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How does the statistics module work?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The collected data is published using DDS over dedicated topics using builtin DataWriters within the Statistics module.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the consequence of compiling the statistics module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It may entail affecting the application's performance.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How can we activate the statistics module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It can be activated using the -DFASTDDS_STATISTICS=ON at CMake configuration step.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How can we start collecting data?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> In order to start collecting data in one of the statistics topics, the corresponding statistics DataWriter should be enabled.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How can we enable the statistics DataWriter?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It can either be done automatically or be enabled at run time using one of two methods: enable_statistics_datawriter() or enable_statistics_datawriter_with_profile().
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the monitor service?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Monitor Service targets any application implementing the subscription side of the Monitor Service Status Topic, giving the possibility of retrieving the Monitoring Information of the local entities.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What Information can the monitor service carry?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It can carry information about the monitoring information of the local entities of a particular DomainParticipant.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How can the monitor service be activated?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Monitor Service can be activated using the -DFASTDDS_STATISTICS=ON at CMake configuration step.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: At which layers can the monitor service be enabled?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It can be programmatically enabled in both DDS Layer and RTPS Layer through the enable_monitor_service() and disable_monitor_service() calls.
   </div>


