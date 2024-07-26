.. _freq_persistence_service_questions:

PERSISTENCE SERVICE Frequently Asked Questions
==============================================


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
       <b>Q: What is persistence in eProsima Fast DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Persistence is a mechanism that allows recovering a previous state on starting the DDS. This is done by configuring the DataWriter's history to be stored in a persistent database, so that the DataWriter can load its history from it on creation. Furthermore, DataReaders can be configured to store the last notified change in the database, so that they can recover their state on creation.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: Why is it useful?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It adds robustness to applications in case of unexpected shutdowns.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the persistence database managed?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> A persistence plugin must be configured for managing the database using the property "dds.persistence.plugin".
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What type of database does the persistence plugin use?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It uses SQLite3 API.
   </div>
