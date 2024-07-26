.. _freq_env_variables_questions:

Environment Variables Frequently Asked Questions
================================================

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
       <b>Q: What are the most important environment variables that affect the behavior of Fast DDS?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> FASTDDS_DEFAULT_PROFILES_FILE, SKIP_DEFAULT_XML, FASTDDS_BUILTIN_TRANSPORTS, ROS_DISCOVERY_SERVER, ROS_SUPER_CLIENT, FASTDDS_STATISTICS, FASTDDS_ENVIRONMENT_FILE.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of the "FASTDDS_DEFAULT_PROFILES_FILE" environment variable?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Defines the location of the default profile configuration XML file.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What happens when the variable "SKIP_DEFAULT_XML" is set to 1?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Skips looking for a default profile configuration XML file. If this variable is set to 1, Fast DDS will load the configuration parameters directly from the classes' definitions without looking for the DEFAULT_FASTDDS_PROFILES.xml in the working directory.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the primary purpose of the FASTDDS_BUILTIN_TRANSPORTS environment variable?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Setting this variable allows to modify the builtin transports that are initialized during the DomainParticipant creation.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of the "ROS_DISCOVERY_SERVER" environment variable?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Setting this variable configures the DomainParticipant to connect to one or more servers using the Discovery Server discovery mechanism.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What happens to a "DomainParticipant" when its discovery protocol is set to "SIMPLE" and "ROS_SUPER_CLIENT" is set to TRUE?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> If the DomainParticipant's discovery protocol is set to SIMPLE, and ROS_SUPER_CLIENT is set to TRUE, the participant is automatically promoted to a SUPER_CLIENT.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of setting the "FASTDDS_STATISTICS" environment variable, according to the provided information?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Setting this variable configures the DomainParticipant to enable the statistics DataWriters which topics are contained in the list set in this environment variable.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What happens when you set the "FASTDDS_ENVIRONMENT_FILE" environment variable to a JSON file?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Setting this environment variable to an existing json file allows to load the environment variables from the file instead of from the environment.
   </div>
