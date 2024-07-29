.. include:: ../../../03-exports/aliases.include
.. include:: ../../../03-exports/aliases-api.include

.. _freq_xtypes_questions:

XTypes Frequently Asked Questions
=================================


 .. collapse::  What are XTypes?
    
    \ 


    :Answer:

    Extensible and **Dynamic Topic Types** for DDS specification. This specification defines the following concepts: DDS supported type system; type representation, including IDL and TypeObject representations; data representation over the wire; language binding; DDS builtin mechanism to automatically discover remote data types.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How does the remote data type discovery mechanism work?

    \ 


    :Answer:

    The remote data type discovery mechanism is based on the exchange of the data type information optimized in order to reduce the required bandwidth.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What prerequisites are required for the remote data type discovery feature to work?
    
    \ 


    :Answer:

    The local data types must be registered into the ``ITypeObjectRegistry``. TypeInformation should be received with the DomainParticipant's endpoint discovery information. A DomainParticipant that does not inform about its TypeInformation would not trigger the remote data type discovery mechanism.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What is the purpose of the Dynamic Language Binding API?
        
    \ 


    :Answer:

    The **Dynamic Language Binding API** allows to define data types at runtime instead of having the types predefined as it is required by the **Plain Language Binding**.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  How can complex data types be managed?

    \ 


    :Answer:

    Dynamic Language Binding provides two possible approaches for managing complex data types: ``DynamicData::get_complex_value`` and ``DynamicData::set_complex_value``; ``DynamicData::loan_value``, which allows to loan a reference to a DynamicData to work with preventing the data copy.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What is serialization?

    \ 


    :Answer:

    Serialization is a crucial process in data distribution services, as it converts complex data structures into a format that can be easily transmitted and reconstructed across different platforms and programming environments.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

 .. collapse::  What are the formats supported for the serialization?

    \ 

    
    :Answer:
    
    A Dynamic Type can be serialized to its IDL representation and to JSON (primitives, strings, enumerations, bitmasks, sequences, arrays, maps, structures, unions, bitsets).

| 

