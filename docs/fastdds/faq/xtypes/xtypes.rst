.. _freq_xtypes_questions:

XTypes Frequently Asked Questions
=================================


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
       <b>Q: What are XTypes?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Extensible and Dynamic Topic Types for DDS specification. This specification defines the following concepts: DDS supported type system; type representation, including IDL and TypeObject representations; data representation over the wire; language binding; DDS builtin mechanism to automatically discover remote data types.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How does the remote data type discovery mechanism work?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The remote data type discovery mechanism is based on the exchange of the data type information optimized in order to reduce the required bandwidth.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What prerequisites are required for the remote data type discovery feature to work?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The local data types must be registered into the ITypeObjectRegistry. TypeInformation should be received with the DomainParticipant's endpoint discovery information. A DomainParticipant that does not inform about its TypeInformation would not trigger the remote data type discovery mechanism.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is the purpose of the Dynamic Language Binding API?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> The Dynamic Language Binding API allows to define data types at runtime instead of having the types predefined as it is required by the Plain Language Binding.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: How can complex data types be managed?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Dynamic Language Binding provides two possible approaches for managing complex data types: DynamicData::get_complex_value and DynamicData::set_complex_value; DynamicData::loan_value, which allows to loan a reference to a DynamicData to work with preventing the data copy.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What is serialization?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> Serialization is a crucial process in data distribution services, as it converts complex data structures into a format that can be easily transmitted and reconstructed across different platforms and programming environments.
   </div>

.. raw:: html

   <div class="question-box">
       <b>Q: What are the formats supported for the serialization?</b>
   </div>
   <div class="answer-box">
       <b>A:</b> A Dynamic Type can be serialized to its IDL representation and to JSON (primitives, strings, enumerations, bitmasks, sequences, arrays, maps, structures, unions, bitsets).
   </div>

