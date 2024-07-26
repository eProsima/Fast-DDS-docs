.. _freq_logging_questions:

Logging Frequently Asked Questions
==================================

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
       <b>Q: What are the functionalities of the logging module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> There are three logging levels, for info, warnings and errors; message filtering and output to STDOUT, STDERR and log files.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the main classes in the logging module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The "Log" class is in charge of the logging operations and provides configuration APIs to set different logging configuration aspects and logging filtering at various levels. The "LogConsumer" class includes the member functions that derived classes should overload to consume log entries.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How does the logging module prevent blocking of the application thread when a logging operation is performed?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> When it is created, the logging module creates a thread that awakens every time an entry is added to the queue and falls back into idle state once the work is done.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How is the logging of messages handled?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> It is handled by three macros, for log messages with Log::Kind::Info, Log::Kind::Warning, and Log::Kind::Info verbosities. These macros produce a log entry showing a message and some meta information.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of assigning a category to log entries in the logging module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The category component can be used to filter log entries so that only those categories specified in the filter are consumed.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the three different filtering possibilities provided by the *eProsima Fast DDS* logging module for log entry filtering?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> *Fast DDS* provides three different filtering possibilities: Category Filtering, File Name Filtering, Content Filtering.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the order in which log entry filters are applied when consuming logs in *eProsima Fast DDS*?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Filters are applied in the specific order presented above, meaning that file name filtering is only applied to the entries that pattern-match the category filter, and content filtering is only applied to the entries that pattern-match both category and file name filters.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary method for filtering log entries by their Category component in the *eProsima Fast DDS* logging module?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Log entries can be filtered upon consumption according to their Category component using regular expressions. Each time an entry is ready to be consumed, the category filter is applied using "std::regex_search()". To set a category filter, the member function "Log::SetCategoryFilter()" is used.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of setting a file name filter in the context of log entry consumption?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Log entries can be filtered upon consumption according to their File Context component using regular expressions. Each time an entry is ready to be consumed, the file name filter is applied using "std::regex_search()". To set a file name filter, the member function "Log::SetFilenameFilter()" is used.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the method for filtering log entries in terms of their message component?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Log entries can be filtered upon consumption according to their Message component using regular expressions. Each time an entry is ready to be consumed, the content filter is applied using "std::regex_search()". To set a content filter, the member function "Log::SetErrorStringFilter()" is used.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are consumers?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Consumers are classes that take a Log::Entry and produce a log output accordingly.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the function of a "StdoutErrConsumer" in the context of log output?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> *StdoutErrConsumer:* Outputs log entries to STDOUT or STDERR depending on the given threshold.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary function of the "FileConsumer" class in the context of logging?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> *FileConsumer:* Outputs log entries to a user specified file.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary function of the "StdoutConsumer" class, as described in the provided text?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> "StdoutConsumer" outputs log entries to the STDOUT stream following the convention specified in the Log Entry Specification. It is the default logging module if the CMake option "LOG_CONSUMER_DEFAULT" is set to "STDOUT".
   </div>

